package rclgo_test

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// spinSettle is how long the tests wait after starting Spin so that the wait
// set is blocked in rcl_wait before new entities are created.
const spinSettle = 300 * time.Millisecond

// spinTimeout bounds every wait for an entity to be serviced.
const spinTimeout = 10 * time.Second

// startSpin runs spin in a goroutine and returns a function that stops it and
// returns its error. The test fails if Spin returns before it is stopped.
//
// Spin is also stopped in a cleanup, which runs before the cleanup of any
// context created earlier with newSpinTestContext, so that a failing test does
// not close a context while Spin is still running on it.
func startSpin(t *testing.T, spin func(context.Context) error) (stop func() error) {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	errs := make(chan error, 1)
	go func() { errs <- spin(ctx) }()
	var (
		once    sync.Once
		stopErr error
	)
	stop = func() error {
		once.Do(func() {
			cancel()
			select {
			case err := <-errs:
				if !errors.Is(err, context.Canceled) {
					stopErr = err
				}
			case <-time.After(spinTimeout):
				stopErr = errors.New("Spin did not return after cancel")
			}
		})
		return stopErr
	}
	t.Cleanup(func() { stop() }) //nolint:errcheck
	select {
	case err := <-errs:
		errs <- err // let the cleanup's stop collect it without waiting
		t.Fatalf("Spin returned early: %v", err)
	case <-time.After(spinSettle):
	}
	return stop
}

func newSpinTestContext(t *testing.T) *rclgo.Context {
	t.Helper()
	rclctx, err := newDefaultRCLContext()
	require.NoError(t, err)
	t.Cleanup(func() { rclctx.Close() })
	return rclctx
}

// requireSubscriptionServiced publishes on topic from node until the
// subscription created by subscribe delivers a message.
func requireSubscriptionServiced(t *testing.T, node *rclgo.Node, topic string, subscribe func(rclgo.SubscriptionCallback)) {
	t.Helper()
	received := make(chan string, 1)
	subscribe(func(s *rclgo.Subscription) {
		msg := std_msgs_msg.NewString()
		if _, err := s.TakeMessage(msg); err == nil {
			select {
			case received <- msg.Data:
			default:
			}
		}
	})
	pub, err := node.NewPublisher(topic, std_msgs_msg.StringTypeSupport, nil)
	require.NoError(t, err)
	msg := std_msgs_msg.NewString()
	msg.Data = "gopher"
	deadline := time.After(spinTimeout)
	tick := time.NewTicker(100 * time.Millisecond)
	defer tick.Stop()
	for {
		require.NoError(t, pub.Publish(msg))
		select {
		case data := <-received:
			require.Equal(t, "gopher", data)
			return
		case <-deadline:
			t.Fatal("subscription created after Spin started was never serviced")
		case <-tick.C:
		}
	}
}

// requireServiceAndClientServiced creates a service and a client on node and
// calls the service until a response arrives. Both the service (request) and
// the client (response) must be serviced by the running Spin.
func requireServiceAndClientServiced(t *testing.T, node *rclgo.Node, name string) {
	t.Helper()
	_, err := node.NewService(name, example_interfaces_srv.AddTwoIntsTypeSupport, nil,
		func(_ *rclgo.ServiceInfo, rm types.Message, srs rclgo.ServiceResponseSender) {
			req := rm.(*example_interfaces_srv.AddTwoInts_Request)
			resp := example_interfaces_srv.NewAddTwoInts_Response()
			resp.Sum = req.A + req.B
			srs.SendResponse(resp) //nolint:errcheck
		})
	require.NoError(t, err)
	client, err := node.NewClient(name, example_interfaces_srv.AddTwoIntsTypeSupport, nil)
	require.NoError(t, err)

	req := example_interfaces_srv.NewAddTwoInts_Request()
	req.A, req.B = 2, 3
	deadline := time.Now().Add(spinTimeout)
	for time.Now().Before(deadline) {
		// Requests sent before discovery completes are dropped, so retry
		// with a short per-call timeout.
		callCtx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
		resp, _, err := client.Send(callCtx, req)
		cancel()
		if err == nil {
			require.EqualValues(t, 5, resp.(*example_interfaces_srv.AddTwoInts_Response).Sum)
			return
		}
	}
	t.Fatal("service or client created after Spin started was never serviced")
}

func TestContextSpinServicesEntitiesCreatedAfterSpinStarts(t *testing.T) {
	t.Run("subscription", func(t *testing.T) {
		rclctx := newSpinTestContext(t)
		stop := startSpin(t, rclctx.Spin)
		node, err := rclctx.NewNode("late_sub", "/spin_test")
		require.NoError(t, err)
		requireSubscriptionServiced(t, node, "late_topic", func(cb rclgo.SubscriptionCallback) {
			_, err := node.NewSubscription("late_topic", std_msgs_msg.StringTypeSupport, nil, cb)
			require.NoError(t, err)
		})
		require.NoError(t, stop())
	})

	t.Run("timer", func(t *testing.T) {
		rclctx := newSpinTestContext(t)
		stop := startSpin(t, rclctx.Spin)
		fired := make(chan struct{}, 1)
		_, err := rclctx.NewTimer(50*time.Millisecond, func(*rclgo.Timer) {
			select {
			case fired <- struct{}{}:
			default:
			}
		})
		require.NoError(t, err)
		select {
		case <-fired:
		case <-time.After(spinTimeout):
			t.Fatal("timer created after Spin started never fired")
		}
		require.NoError(t, stop())
	})

	t.Run("service and client", func(t *testing.T) {
		rclctx := newSpinTestContext(t)
		stop := startSpin(t, rclctx.Spin)
		node, err := rclctx.NewNode("late_srv", "/spin_test")
		require.NoError(t, err)
		requireServiceAndClientServiced(t, node, "late_add")
		require.NoError(t, stop())
	})

	t.Run("entity added to a node that existed before Spin", func(t *testing.T) {
		rclctx := newSpinTestContext(t)
		node, err := rclctx.NewNode("early_node", "/spin_test")
		require.NoError(t, err)
		stop := startSpin(t, rclctx.Spin)
		requireSubscriptionServiced(t, node, "early_node_topic", func(cb rclgo.SubscriptionCallback) {
			_, err := node.NewSubscription("early_node_topic", std_msgs_msg.StringTypeSupport, nil, cb)
			require.NoError(t, err)
		})
		require.NoError(t, stop())
	})
}

func TestNodeSpinServicesEntitiesCreatedAfterSpinStarts(t *testing.T) {
	t.Run("subscription", func(t *testing.T) {
		rclctx := newSpinTestContext(t)
		node, err := rclctx.NewNode("node_spin_sub", "/spin_test")
		require.NoError(t, err)
		stop := startSpin(t, node.Spin)
		requireSubscriptionServiced(t, node, "node_spin_topic", func(cb rclgo.SubscriptionCallback) {
			_, err := node.NewSubscription("node_spin_topic", std_msgs_msg.StringTypeSupport, nil, cb)
			require.NoError(t, err)
		})
		require.NoError(t, stop())
	})

	t.Run("service and client", func(t *testing.T) {
		rclctx := newSpinTestContext(t)
		node, err := rclctx.NewNode("node_spin_srv", "/spin_test")
		require.NoError(t, err)
		stop := startSpin(t, node.Spin)
		requireServiceAndClientServiced(t, node, "node_spin_add")
		require.NoError(t, stop())
	})
}

// A subscription created without a callback is polled by its owner with
// TakeMessage (skyhook does this for command ACKs). Spin must neither call the
// nil callback nor take the owner's messages, also when the subscription is
// created after Spin started.
func TestSpinLeavesPollOnlySubscriptionsToTheirOwner(t *testing.T) {
	for _, tc := range []struct {
		name      string
		subscribe func(node *rclgo.Node, topic string) (take func() (string, bool))
	}{
		{"generated wrapper with nil callback", func(node *rclgo.Node, topic string) func() (string, bool) {
			sub, err := std_msgs_msg.NewStringSubscription(node, topic, nil, nil)
			require.NoError(t, err)
			return func() (string, bool) {
				msg := std_msgs_msg.NewString()
				info, err := sub.TakeMessage(msg)
				return msg.Data, err == nil && info != nil
			}
		}},
		{"subscription with nil callback", func(node *rclgo.Node, topic string) func() (string, bool) {
			sub, err := node.NewSubscription(topic, std_msgs_msg.StringTypeSupport, nil, nil)
			require.NoError(t, err)
			return func() (string, bool) {
				msg := std_msgs_msg.NewString()
				info, err := sub.TakeMessage(msg)
				return msg.Data, err == nil && info != nil
			}
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			rclctx := newSpinTestContext(t)
			node, err := rclctx.NewNode("poll_only", "/spin_test")
			require.NoError(t, err)
			stop := startSpin(t, rclctx.Spin)
			take := tc.subscribe(node, "poll_only_topic")
			pub, err := node.NewPublisher("poll_only_topic", std_msgs_msg.StringTypeSupport, nil)
			require.NoError(t, err)
			msg := std_msgs_msg.NewString()
			msg.Data = "ack"
			deadline := time.Now().Add(spinTimeout)
			for {
				require.NoError(t, pub.Publish(msg))
				time.Sleep(20 * time.Millisecond)
				if data, ok := take(); ok {
					require.Equal(t, "ack", data)
					break
				}
				if time.Now().After(deadline) {
					t.Fatal("poll-only subscription never received a message")
				}
			}
			require.NoError(t, stop(), "Spin failed while a poll-only subscription existed")
		})
	}
}
