package rclgo_test

import (
	"context"
	"errors"
	"fmt"
	"os"
	"runtime"
	"strings"
	"sync"
	"testing"
	"time"

	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Go-specific tests: properties that come from goroutines, the scheduler, the
// garbage collector and cgo rather than from ROS.

// rcl keeps its error state per OS thread, and a goroutine can move to another
// thread between two cgo calls. The text of an rcl error must still be the
// text of that call's failure, never another goroutine's.
func TestRCLErrorTextBelongsToItsCall(t *testing.T) {
	const (
		goroutines = 16
		calls      = 300
	)
	var wg sync.WaitGroup
	wrong := make(chan string, goroutines)
	for g := range goroutines {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := range calls {
				sub := fmt.Sprintf("g%d_c%d", g, i)
				_, err := rclgo.ExpandTopicName("{"+sub+"}/x", "node", "/ns", nil)
				if err == nil || !strings.Contains(err.Error(), "unknown substitution: {"+sub+"}") {
					select {
					case wrong <- fmt.Sprintf("{%s}: %v", sub, err):
					default:
					}
					return
				}
			}
		}()
	}
	wg.Wait()
	close(wrong)
	for w := range wrong {
		t.Errorf("error text from another call: %s", w)
	}
}

// Independent contexts may be created, used and closed from many goroutines
// at once; process-wide state (logging, type registries) must cope.
func TestContextsInParallel(t *testing.T) {
	const workers, rounds = 8, 3
	var wg sync.WaitGroup
	errs := make(chan error, workers*rounds)
	for w := range workers {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for r := range rounds {
				if err := pubSubOnce(fmt.Sprintf("parallel_%d_%d", w, r)); err != nil {
					errs <- fmt.Errorf("worker %d round %d: %w", w, r, err)
					return
				}
			}
		}()
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		t.Error(err)
	}
}

// pubSubOnce creates a context with a publisher and a subscription, spins,
// waits for one message and closes everything.
func pubSubOnce(name string) (err error) {
	rclctx, err := newDefaultRCLContext()
	if err != nil {
		return err
	}
	defer func() { err = errors.Join(err, rclctx.Close()) }()
	node, err := rclctx.NewNode(name, "/go_runtime_test")
	if err != nil {
		return err
	}
	got := make(chan struct{}, 1)
	_, err = node.NewSubscription("t", std_msgs_msg.StringTypeSupport, nil, func(s *rclgo.Subscription) {
		if _, err := s.TakeMessage(std_msgs_msg.NewString()); err == nil {
			select {
			case got <- struct{}{}:
			default:
			}
		}
	})
	if err != nil {
		return err
	}
	pub, err := node.NewPublisher("t", std_msgs_msg.StringTypeSupport, nil)
	if err != nil {
		return err
	}
	ctx, cancel := context.WithCancel(context.Background())
	spinDone := make(chan error, 1)
	go func() { spinDone <- rclctx.Spin(ctx) }()
	defer func() {
		cancel()
		<-spinDone
	}()
	deadline := time.After(spinTimeout)
	for {
		if err := pub.Publish(std_msgs_msg.NewString()); err != nil {
			return err
		}
		select {
		case <-got:
			return nil
		case <-deadline:
			return errors.New("message never received")
		case <-time.After(20 * time.Millisecond):
		}
	}
}

// After everything is closed, no goroutine started by rclgo may be left.
func TestNoGoroutineLeaks(t *testing.T) {
	runtime.GC()
	baseline := runtime.NumGoroutine()
	func() {
		f := newActionFixture(t, "leaks", nil, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
			sender, err := goal.Accept()
			if err != nil {
				return nil, err
			}
			fb := example_interfaces_action.NewFibonacci_Feedback()
			sender.Send(fb) //nolint:errcheck
			return fibResult(1), nil
		})
		resp, id := f.sendGoal(1)
		require.True(t, resp.Accepted)
		require.EqualValues(t, rclgo.GoalSucceeded, f.getResult(id).Status)
		_, err := f.ctx.NewTimer(time.Millisecond, func(*rclgo.Timer) {})
		require.NoError(t, err)
		require.NoError(t, pubSubOnce("leaks_pubsub"))
		require.NoError(t, f.stop())
		require.NoError(t, f.ctx.Close())
	}()
	deadline := time.Now().Add(spinTimeout)
	for runtime.NumGoroutine() > baseline && time.Now().Before(deadline) {
		time.Sleep(50 * time.Millisecond)
	}
	if n := runtime.NumGoroutine(); n > baseline {
		buf := make([]byte, 1<<20)
		t.Fatalf("%d goroutines left over (baseline %d):\n%s", n-baseline, baseline, buf[:runtime.Stack(buf, true)])
	}
}

// A panic in a callback unwinds Spin to its caller, which may recover; Spin's
// holds are released, so the context can still be closed.
func TestPanicInCallbackReachesSpinCaller(t *testing.T) {
	rclctx, err := newDefaultRCLContext()
	require.NoError(t, err)
	node, err := rclctx.NewNode("panicking", "/go_runtime_test")
	require.NoError(t, err)
	_, err = node.NewSubscription("t", std_msgs_msg.StringTypeSupport, nil, func(s *rclgo.Subscription) {
		s.TakeMessage(std_msgs_msg.NewString()) //nolint:errcheck
		panic("callback failed")
	})
	require.NoError(t, err)
	pub, err := node.NewPublisher("t", std_msgs_msg.StringTypeSupport, nil)
	require.NoError(t, err)
	recovered := make(chan any, 1)
	go func() {
		defer func() { recovered <- recover() }()
		rclctx.Spin(context.Background()) //nolint:errcheck
	}()
	deadline := time.After(spinTimeout)
	var r any
	for r == nil {
		require.NoError(t, pub.Publish(std_msgs_msg.NewString()))
		select {
		case r = <-recovered:
		case <-deadline:
			t.Fatal("callback never ran")
		case <-time.After(20 * time.Millisecond):
		}
	}
	require.Equal(t, "callback failed", r)
	require.NoError(t, requireReturns(t, "Context.Close after a panic in Spin", rclctx.Close))
}

// Nodes created and closed concurrently from many goroutines (each node
// registers a /rosout publisher in rcl's process-wide table).
func TestNodesCreatedAndClosedConcurrently(t *testing.T) {
	if os.Getenv("RMW_IMPLEMENTATION") == "rmw_zenoh_cpp" {
		// rmw_zenoh_cpp 0.2.11 aborts the process (an uncaught
		// zenoh::ZException from rcl_shutdown, "Failed to close the session")
		// when many contexts shut down concurrently; plain rcl in C with
		// pthreads does the same, so this is not an rclgo defect.
		t.Skip("rmw_zenoh_cpp aborts on concurrent context shutdown (upstream)")
	}
	var wg sync.WaitGroup
	errs := make(chan error, 16)
	for g := range 16 {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := range 25 {
				ctx, err := rclgo.NewContext(0, nil)
				if err == nil {
					_, err = ctx.NewNode(fmt.Sprintf("concurrent_%d_%d", g, i), "/go_runtime_test")
					err = errors.Join(err, ctx.Close())
				}
				if err != nil {
					errs <- err
					return
				}
			}
		}()
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		t.Error(err)
	}
}
