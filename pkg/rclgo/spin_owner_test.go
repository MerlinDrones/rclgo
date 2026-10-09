package rclgo_test

import (
	"sync/atomic"
	"testing"
	"time"

	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/stretchr/testify/require"
)

// When two Spins could service one entity (Context.Spin and Node.Spin on the
// same node), only the first one does, so each message is handled once and no
// callback runs for a message the other Spin already took. When the owner
// stops, the other Spin takes over.
func TestOneSpinServicesAnEntity(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("two_spins", "/spin_owner_test")
	require.NoError(t, err)
	var received, empty atomic.Int64
	keepAll := qos.NewDefault()
	keepAll.History = qos.HistoryKeepAll
	_, err = node.NewSubscription("shared", std_msgs_msg.StringTypeSupport, &rclgo.SubscriptionOptions{Qos: keepAll},
		func(s *rclgo.Subscription) {
			if info, err := s.TakeMessage(std_msgs_msg.NewString()); err == nil && info != nil {
				received.Add(1)
			} else {
				empty.Add(1)
			}
		})
	require.NoError(t, err)
	pub, err := node.NewPublisher("shared", std_msgs_msg.StringTypeSupport, &rclgo.PublisherOptions{Qos: keepAll})
	require.NoError(t, err)

	stopContextSpin := startSpin(t, rclctx.Spin)
	stopNodeSpin := startSpin(t, node.Spin)

	publish := func(n int64) {
		t.Helper()
		start := received.Load()
		deadline := time.Now().Add(spinTimeout)
		for i := int64(0); i < n; i++ {
			require.NoError(t, pub.Publish(std_msgs_msg.NewString()))
			time.Sleep(time.Millisecond)
		}
		for received.Load()-start < n && time.Now().Before(deadline) {
			time.Sleep(10 * time.Millisecond)
		}
		require.Equal(t, n, received.Load()-start, "messages received")
	}
	publish(200)
	require.Zero(t, empty.Load(), "callbacks that found no message: two Spins serviced the subscription")

	// The owner (Context.Spin, started first) stops; Node.Spin takes over.
	require.NoError(t, stopContextSpin())
	publish(20)
	require.NoError(t, stopNodeSpin())
}
