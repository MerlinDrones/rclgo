package rclgo_test

import (
	"context"
	"errors"
	"sync/atomic"
	"testing"
	"time"

	action_msgs_srv "github.com/merlindrones/rclgo/pkg/msgs/action_msgs/srv"
	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Corners of the action protocol, after rcl_action's test_action_server and
// rclcpp_action's test_server: cancelling every goal, cancelling by
// timestamp, cancelling a finished goal, and a server going away while a
// client waits for a result.

// blockingFixture runs goals that wait until they are canceled; started
// receives one value per goal that started executing.
func blockingFixture(t *testing.T, name string) (*actionFixture, chan struct{}) {
	t.Helper()
	started := make(chan struct{}, 16)
	f := newActionFixture(t, name, nil, func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		started <- struct{}{}
		<-ctx.Done()
		return fibResult(0), nil
	})
	return f, started
}

func cancelRequest(f *actionFixture, id *types.GoalID, stamp time.Duration) *action_msgs_srv.CancelGoal_Response {
	f.t.Helper()
	req := action_msgs_srv.NewCancelGoal_Request()
	if id != nil {
		req.GoalInfo.GoalId.Uuid = *id
	}
	req.GoalInfo.Stamp.Sec = int32(stamp / time.Second)
	req.GoalInfo.Stamp.Nanosec = uint32(stamp % time.Second)
	ctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
	defer cancel()
	resp, err := f.client.CancelGoal(ctx, req)
	require.NoError(f.t, err)
	return resp.(*action_msgs_srv.CancelGoal_Response)
}

// A cancel request with a zero goal ID and a zero stamp cancels every goal.
func TestActionCancelAllGoals(t *testing.T) {
	f, started := blockingFixture(t, "cancel_all")
	var ids []*types.GoalID
	for i := 0; i < 3; i++ {
		resp, id := f.sendGoal(int32(i))
		require.True(t, resp.Accepted)
		<-started
		ids = append(ids, id)
	}
	resp := cancelRequest(f, nil, 0)
	require.Equal(t, action_msgs_srv.CancelGoal_Response_ERROR_NONE, resp.ReturnCode)
	require.Len(t, resp.GoalsCanceling, 3)
	for _, id := range ids {
		require.EqualValues(t, rclgo.GoalCanceled, f.getResult(id).Status)
	}
}

// A cancel request with a zero goal ID and a stamp cancels the goals
// accepted at or before the stamp, and leaves later ones running.
func TestActionCancelByStamp(t *testing.T) {
	f, started := blockingFixture(t, "cancel_stamp")
	early, earlyID := f.sendGoal(1)
	<-started
	time.Sleep(50 * time.Millisecond) // distinct acceptance stamps
	_, lateID := f.sendGoal(2)
	<-started
	stamp := time.Duration(early.Stamp.Sec)*time.Second + time.Duration(early.Stamp.Nanosec)
	resp := cancelRequest(f, nil, stamp)
	require.Equal(t, action_msgs_srv.CancelGoal_Response_ERROR_NONE, resp.ReturnCode)
	require.Len(t, resp.GoalsCanceling, 1)
	require.Equal(t, *earlyID, types.GoalID(resp.GoalsCanceling[0].GoalId.Uuid))
	require.EqualValues(t, rclgo.GoalCanceled, f.getResult(earlyID).Status)
	resp = cancelRequest(f, lateID, 0)
	require.Equal(t, action_msgs_srv.CancelGoal_Response_ERROR_NONE, resp.ReturnCode, "the later goal was still running")
}

// Cancelling a goal that already finished is refused with
// ERROR_GOAL_TERMINATED.
func TestActionCancelFinishedGoal(t *testing.T) {
	f := newActionFixture(t, "cancel_finished", nil, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		return fibResult(1), nil
	})
	_, id := f.sendGoal(1)
	require.EqualValues(t, rclgo.GoalSucceeded, f.getResult(id).Status)
	resp := cancelRequest(f, id, 0)
	require.Equal(t, action_msgs_srv.CancelGoal_Response_ERROR_GOAL_TERMINATED, resp.ReturnCode)
	require.Empty(t, resp.GoalsCanceling)
}

// A client waiting for a result when its server goes away returns at its
// own deadline; a new server under the same name answers for goals it never
// had with an unknown status.
func TestActionServerGoneWhileClientWaits(t *testing.T) {
	f, started := blockingFixture(t, "server_gone")
	_, id := f.sendGoal(1)
	<-started
	errs := make(chan error, 1)
	go func() {
		ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		defer cancel()
		_, err := f.client.GetResult(ctx, id)
		errs <- err
	}()
	time.Sleep(200 * time.Millisecond) // the result request is pending
	require.NoError(t, f.server.Close())
	select {
	case err := <-errs:
		require.True(t, errors.Is(err, context.DeadlineExceeded), "GetResult with the server gone: %v", err)
	case <-time.After(actionTimeout):
		t.Fatal("GetResult did not return at its deadline")
	}

	_, err := f.node.NewActionServer("server_gone", rclgo.NewAction(example_interfaces_action.FibonacciTypeSupport,
		func(context.Context, *rclgo.GoalHandle) (types.Message, error) { return nil, errors.New("unused") }), nil)
	require.NoError(t, err)
	requireEventually(t, "result from the new server", func() error {
		ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
		defer cancel()
		resp, err := f.client.GetResult(ctx, id)
		if err != nil {
			return err
		}
		require.EqualValues(t, rclgo.GoalUnknown, resp.(*example_interfaces_action.Fibonacci_GetResult_Response).Status)
		return nil
	})
}

// Closing the node from a callback that runs on the Spin goroutine neither
// deadlocks nor crashes; Spin returns.
func TestCloseNodeFromCallbackOnSpin(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("close_from_callback", "/action_test")
	require.NoError(t, err)
	var closed atomic.Bool
	closeErr := make(chan error, 1)
	_, err = node.NewSubscription("close_trigger", std_msgs_msg.StringTypeSupport, nil, func(s *rclgo.Subscription) {
		if closed.CompareAndSwap(false, true) {
			closeErr <- node.Close()
		}
	})
	require.NoError(t, err)
	pub, err := node.NewPublisher("close_trigger", std_msgs_msg.StringTypeSupport, nil)
	require.NoError(t, err)
	spinDone := make(chan error, 1)
	go func() { spinDone <- node.Spin(context.Background()) }()
	requireEventually(t, "callback ran", func() error {
		if closed.Load() {
			return nil
		}
		pub.Publish(std_msgs_msg.NewString()) //nolint:errcheck
		return errors.New("not yet")
	})
	select {
	case err := <-closeErr:
		require.NoError(t, err, "Node.Close from the callback")
	case <-time.After(actionTimeout):
		t.Fatal("Node.Close from a callback on the Spin goroutine deadlocked")
	}
	select {
	case err := <-spinDone:
		require.NoError(t, err, "Node.Spin after its node was closed")
	case <-time.After(actionTimeout):
		t.Fatal("Node.Spin did not return after its node was closed from a callback")
	}
}
