package rclgo_test

import (
	"context"
	"fmt"
	"testing"
	"time"

	action_msgs_srv "github.com/merlindrones/rclgo/pkg/msgs/action_msgs/srv"
	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Every public call on a closed entity, or on an entity whose node or context
// is closed, fails with an error matching ErrClosed, so callers can tell
// "closed" from other failures with one errors.Is.
func TestEveryCallAfterCloseIsErrClosed(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("errclosed", "/errclosed_test")
	require.NoError(t, err)
	ctx := context.Background()
	check := func(what string, err error) {
		t.Helper()
		require.ErrorIs(t, err, rclgo.ErrClosed, what)
	}

	pub, err := node.NewPublisher("t", std_msgs_msg.StringTypeSupport, nil)
	require.NoError(t, err)
	sub, err := node.NewSubscription("t", std_msgs_msg.StringTypeSupport, nil, nil)
	require.NoError(t, err)
	srv, err := node.NewService("s", example_interfaces_srv.AddTwoIntsTypeSupport, nil,
		func(*rclgo.ServiceInfo, types.Message, rclgo.ServiceResponseSender) {})
	require.NoError(t, err)
	cli, err := node.NewClient("s", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
	require.NoError(t, err)
	actionClient, err := node.NewActionClient("a", example_interfaces_action.FibonacciTypeSupport, nil)
	require.NoError(t, err)
	actionServer, err := node.NewActionServer("a", rclgo.NewAction(example_interfaces_action.FibonacciTypeSupport,
		func(context.Context, *rclgo.GoalHandle) (types.Message, error) { return nil, nil }), nil)
	require.NoError(t, err)
	timer, err := rclctx.NewTimer(time.Hour, func(*rclgo.Timer) {})
	require.NoError(t, err)
	clock, err := rclctx.NewClock(rclgo.ClockTypeROSTime)
	require.NoError(t, err)
	ws, err := rclctx.NewWaitSet()
	require.NoError(t, err)

	for _, c := range []interface{ Close() error }{pub, sub, srv, cli, actionClient, actionServer, timer, clock, ws} {
		require.NoError(t, c.Close())
		check(fmt.Sprintf("second Close of %T", c), c.Close())
	}
	check("Publish", pub.Publish(std_msgs_msg.NewString()))
	check("PublishSerialized", pub.PublishSerialized([]byte{0, 1, 0, 0}))
	_, err = pub.GetSubscriptionCount()
	check("GetSubscriptionCount", err)
	_, err = sub.TakeMessage(std_msgs_msg.NewString())
	check("TakeMessage", err)
	_, _, err = sub.TakeSerializedMessage()
	check("TakeSerializedMessage", err)
	_, err = sub.GetPublisherCount()
	check("GetPublisherCount", err)
	_, _, err = cli.Send(ctx, example_interfaces_srv.NewAddTwoInts_Request())
	check("Client.Send", err)
	_, _, err = actionClient.SendGoal(ctx, example_interfaces_action.NewFibonacci_Goal())
	check("SendGoal", err)
	_, _, err = actionClient.WatchGoal(ctx, example_interfaces_action.NewFibonacci_Goal(), nil)
	check("WatchGoal", err)
	_, err = actionClient.GetResult(ctx, &types.GoalID{})
	check("GetResult", err)
	_, err = actionClient.CancelGoal(ctx, action_msgs_srv.NewCancelGoal_Request())
	check("CancelGoal", err)
	check("WatchFeedback", <-actionClient.WatchFeedback(ctx, &types.GoalID{}, func(context.Context, types.Message) {}))
	check("WatchStatus", <-actionClient.WatchStatus(ctx, &types.GoalID{}, func(context.Context, types.Message) {}))
	_, err = timer.GetTimeUntilNextCall()
	check("GetTimeUntilNextCall", err)
	check("Timer.Reset", timer.Reset())
	check("EnableROSTimeOverride", clock.EnableROSTimeOverride())
	check("DisableROSTimeOverride", clock.DisableROSTimeOverride())
	check("SetROSTimeOverride", clock.SetROSTimeOverride(time.Second))
	check("WaitSet.Run", ws.Run(ctx))

	// A closed node: its constructors and graph queries.
	require.NoError(t, node.Close())
	check("second Node.Close", node.Close())
	check("Node.Spin", node.Spin(ctx))
	_, err = node.NewPublisher("t2", std_msgs_msg.StringTypeSupport, nil)
	check("NewPublisher", err)
	_, err = node.NewSubscription("t2", std_msgs_msg.StringTypeSupport, nil, nil)
	check("NewSubscription", err)
	_, err = node.NewService("s2", example_interfaces_srv.AddTwoIntsTypeSupport, nil,
		func(*rclgo.ServiceInfo, types.Message, rclgo.ServiceResponseSender) {})
	check("NewService", err)
	_, err = node.NewClient("s2", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
	check("NewClient", err)
	_, err = node.NewActionClient("a2", example_interfaces_action.FibonacciTypeSupport, nil)
	check("NewActionClient", err)
	_, err = node.NewActionServer("a2", rclgo.NewAction(example_interfaces_action.FibonacciTypeSupport,
		func(context.Context, *rclgo.GoalHandle) (types.Message, error) { return nil, nil }), nil)
	check("NewActionServer", err)
	_, err = node.GetTopicNamesAndTypes(true)
	check("GetTopicNamesAndTypes", err)
	_, _, err = node.GetNodeNames()
	check("GetNodeNames", err)
	_, err = node.GetPublishersInfoByTopic("/t", true)
	check("GetPublishersInfoByTopic", err)
	_, err = node.GetSubscriptionsInfoByTopic("/t", true)
	check("GetSubscriptionsInfoByTopic", err)
	_, err = node.GetPublisherNamesAndTypesByNode(true, "errclosed", "/errclosed_test")
	check("GetPublisherNamesAndTypesByNode", err)
	_, err = node.GetServiceNamesAndTypesByNode("errclosed", "/errclosed_test")
	check("GetServiceNamesAndTypesByNode", err)

	// A closed context: its constructors and Spin.
	require.NoError(t, rclctx.Close())
	check("second Context.Close", rclctx.Close())
	check("Context.Spin", rclctx.Spin(ctx))
	_, err = rclctx.NewNode("n2", "/errclosed_test")
	check("NewNode", err)
	_, err = rclctx.NewTimer(time.Hour, func(*rclgo.Timer) {})
	check("NewTimer", err)
	_, err = rclctx.NewClock(rclgo.ClockTypeROSTime)
	check("NewClock", err)
	_, err = rclctx.NewWaitSet()
	check("NewWaitSet", err)
}

// A Send waiting for its response returns ErrClosed when the client is closed.
func TestPendingSendWokenByCloseIsErrClosed(t *testing.T) {
	_, _, cli := newAddService(t, "pending_closed", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		if req.A == 0 && req.B == 0 {
			respondSum(req, send) //nolint:errcheck
		} // other requests are never answered
	})
	errs := make(chan error, 1)
	go func() {
		_, _, err := cli.Send(context.Background(), &example_interfaces_srv.AddTwoInts_Request{A: 1})
		errs <- err
	}()
	time.Sleep(100 * time.Millisecond) // the request is pending (if not, Send fails with ErrClosed too)
	require.NoError(t, cli.Close())
	select {
	case err := <-errs:
		require.ErrorIs(t, err, rclgo.ErrClosed)
	case <-time.After(actionTimeout):
		t.Fatal("pending Send not woken by Close")
	}
}

// A feedback or status watch with a context that never ends returns ErrClosed
// when its action client is closed, instead of waiting forever.
func TestActionWatchesEndWhenClientCloses(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("watch_close", "/errclosed_test")
	require.NoError(t, err)
	client, err := node.NewActionClient("w", example_interfaces_action.FibonacciTypeSupport, nil)
	require.NoError(t, err)
	feedback := client.WatchFeedback(context.Background(), nil, func(context.Context, types.Message) {})
	status := client.WatchStatus(context.Background(), &types.GoalID{}, func(context.Context, types.Message) {})
	require.NoError(t, client.Close())
	for name, errc := range map[string]<-chan error{"feedback": feedback, "status": status} {
		select {
		case err := <-errc:
			require.ErrorIs(t, err, rclgo.ErrClosed, name)
		case <-time.After(actionTimeout):
			t.Fatalf("%s watch did not end when the client was closed", name)
		}
	}
}
