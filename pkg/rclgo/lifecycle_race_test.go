package rclgo_test

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"testing"
	"time"

	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/stretchr/testify/require"
)

// Entities created and graph queries made while their context closes either
// succeed or fail with ErrClosed; none uses a finalized node, and none
// outlives the context (rclcpp keeps a node alive while entities are created
// on it in the same way).
func TestEntityCreationRacingClose(t *testing.T) {
	for round := 0; round < 20; round++ {
		rclctx, err := newDefaultRCLContext()
		require.NoError(t, err)
		node, err := rclctx.NewNode(fmt.Sprintf("racing_close_%d", round), "/race_test")
		require.NoError(t, err)

		create := []func(i int) error{
			func(i int) error {
				_, err := node.NewPublisher(fmt.Sprintf("pub%d", i), std_msgs_msg.StringTypeSupport, nil)
				return err
			},
			func(i int) error {
				_, err := node.NewSubscription(fmt.Sprintf("sub%d", i), std_msgs_msg.StringTypeSupport, nil, func(*rclgo.Subscription) {})
				return err
			},
			func(i int) error {
				_, err := node.NewClient(fmt.Sprintf("cli%d", i), example_interfaces_srv.AddTwoIntsTypeSupport, nil)
				return err
			},
			func(int) error { _, err := node.GetTopicNamesAndTypes(true); return err },
			func(int) error { _, _, err := node.GetNodeNames(); return err },
			func(int) error {
				_, err := rclctx.NewTimer(time.Hour, func(*rclgo.Timer) {})
				return err
			},
		}
		var (
			wg   sync.WaitGroup
			mu   sync.Mutex
			errs []error
		)
		start := make(chan struct{})
		for _, f := range create {
			wg.Add(1)
			go func() {
				defer wg.Done()
				<-start
				for i := 0; ; i++ {
					if err := f(i); err != nil {
						mu.Lock()
						errs = append(errs, err)
						mu.Unlock()
						return
					}
				}
			}()
		}
		close(start)
		time.Sleep(time.Duration(round%5) * time.Millisecond)
		require.NoError(t, rclctx.Close())
		wg.Wait()
		for _, err := range errs {
			require.ErrorIs(t, err, rclgo.ErrClosed, "round %d", round)
		}
	}
}

// Closing a node makes its Node.Spin return, like closing a context does for
// Context.Spin.
func TestNodeCloseStopsNodeSpin(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("close_stops_spin", "/race_test")
	require.NoError(t, err)
	_, err = node.NewSubscription("topic", std_msgs_msg.StringTypeSupport, nil, func(*rclgo.Subscription) {})
	require.NoError(t, err)
	done := make(chan error, 1)
	go func() { done <- node.Spin(context.Background()) }()
	time.Sleep(spinSettle)
	require.NoError(t, node.Close())
	select {
	case err := <-done:
		require.NoError(t, err)
	case <-time.After(closeTimeout):
		t.Fatal("Node.Spin did not return after the node was closed")
	}
	require.ErrorIs(t, node.Spin(context.Background()), rclgo.ErrClosed)
}

// Requests sent while their client is closed return instead of waiting for a
// response forever.
func TestClientSendRacingClose(t *testing.T) {
	for round := 0; round < 20; round++ {
		rclctx := newSpinTestContext(t)
		node, err := rclctx.NewNode(fmt.Sprintf("send_racing_close_%d", round), "/race_test")
		require.NoError(t, err)
		client, err := node.NewClient("no_server", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
		require.NoError(t, err)
		const senders = 8
		done := make(chan error, senders)
		for i := 0; i < senders; i++ {
			go func() {
				_, _, err := client.Send(context.Background(), example_interfaces_srv.NewAddTwoInts_Request())
				done <- err
			}()
		}
		time.Sleep(time.Duration(round%4) * time.Millisecond)
		require.NoError(t, client.Close())
		for i := 0; i < senders; i++ {
			select {
			case err := <-done:
				require.Error(t, err)
				if !errors.Is(err, rclgo.ErrClosed) {
					require.Contains(t, err.Error(), "closed before a response")
				}
			case <-time.After(closeTimeout):
				t.Fatalf("round %d: Send did not return after Close", round)
			}
		}
	}
}
