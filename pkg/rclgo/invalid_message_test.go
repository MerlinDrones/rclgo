package rclgo_test

import (
	"context"
	"errors"
	"testing"
	"time"

	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	test_msgs "github.com/merlindrones/rclgo/pkg/msgs/test_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// A nil message, or one of another type than the entity was created for, is
// refused with ErrInvalidMessage instead of panicking in generated conversion
// code (which, outside a recovered goroutine, crashes the process).

func TestInvalidMessagesRefused(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("invalid_messages", "/invalid_test")
	require.NoError(t, err)
	pub, err := node.NewPublisher("invalid", std_msgs_msg.StringTypeSupport, nil)
	require.NoError(t, err)
	sub, err := node.NewSubscription("invalid", std_msgs_msg.StringTypeSupport, nil, nil)
	require.NoError(t, err)
	cli, err := node.NewClient("invalid_add", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
	require.NoError(t, err)

	var nilString *std_msgs_msg.String
	for name, msg := range map[string]types.Message{
		"nil":        nil,
		"typed nil":  nilString,
		"wrong type": test_msgs.NewBasicTypes(),
	} {
		t.Run(name, func(t *testing.T) {
			require.ErrorIs(t, pub.Publish(msg), rclgo.ErrInvalidMessage, "Publish")
			_, err := sub.TakeMessage(msg)
			require.ErrorIs(t, err, rclgo.ErrInvalidMessage, "TakeMessage")
			ctx, cancel := context.WithTimeout(context.Background(), time.Second)
			defer cancel()
			_, _, err = cli.Send(ctx, msg)
			require.ErrorIs(t, err, rclgo.ErrInvalidMessage, "Client.Send")
		})
	}
	t.Run("serialized-only type support", func(t *testing.T) {
		ts, err := rclgo.LoadDynamicMessageTypeSupport("std_msgs", "String")
		require.NoError(t, err)
		dpub, err := node.NewPublisher("invalid_dynamic", ts, nil)
		require.NoError(t, err)
		require.ErrorIs(t, dpub.Publish(std_msgs_msg.NewString()), rclgo.ErrInvalidMessage)
		dsub, err := node.NewSubscription("invalid_dynamic", ts, nil, nil)
		require.NoError(t, err)
		_, err = dsub.TakeMessage(std_msgs_msg.NewString())
		require.ErrorIs(t, err, rclgo.ErrInvalidMessage)
		buf, err := rclgo.Serialize(&std_msgs_msg.String{Data: "still works"})
		require.NoError(t, err)
		require.NoError(t, dpub.PublishSerialized(buf), "the serialized API still works")
	})
}

// A service handler answering with a response of the wrong type gets an error.
func TestServiceResponseOfWrongTypeRefused(t *testing.T) {
	responseErr := make(chan error, 1)
	_, _, cli := newAddService(t, "wrong_response", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		if req.A == 0 && req.B == 0 { // the fixture's readiness probe
			respondSum(req, send) //nolint:errcheck
			return
		}
		responseErr <- send.SendResponse(std_msgs_msg.NewString())
	})
	go callAdd(cli, 1, 2, time.Second) //nolint:errcheck
	select {
	case err := <-responseErr:
		require.ErrorIs(t, err, rclgo.ErrInvalidMessage)
	case <-time.After(actionTimeout):
		t.Fatal("request never arrived")
	}
}

// A hand-written action whose ExecuteGoal returns a result of the wrong type
// aborts the goal; the process survives and the client gets an answer.
func TestActionResultOfWrongTypeAbortsGoal(t *testing.T) {
	f := newActionFixture(t, "wrong_result", nil, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		return std_msgs_msg.NewString(), nil
	})
	_, id := f.sendGoal(1)
	require.EqualValues(t, rclgo.GoalAborted, f.getResult(id).Status)
}

// Feedback that is nil or of the wrong type is refused with an error, also
// from a goroutine the action started itself (outside ExecuteGoal's recover).
func TestActionFeedbackOfWrongTypeRefused(t *testing.T) {
	feedbackErrs := make(chan error, 2)
	f := newActionFixture(t, "wrong_feedback", nil, func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		sender, err := goal.Accept()
		if err != nil {
			return nil, err
		}
		done := make(chan struct{})
		go func() {
			defer close(done)
			feedbackErrs <- sender.Send(nil)
			feedbackErrs <- sender.Send(std_msgs_msg.NewString())
		}()
		<-done
		return fibResult(1), nil
	})
	_, id := f.sendGoal(1)
	require.EqualValues(t, rclgo.GoalSucceeded, f.getResult(id).Status)
	for i := 0; i < 2; i++ {
		require.ErrorIs(t, <-feedbackErrs, rclgo.ErrInvalidMessage)
	}
}

// A goal of the wrong type is refused before it is sent.
func TestActionGoalOfWrongTypeRefused(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("wrong_goal", "/invalid_test")
	require.NoError(t, err)
	client, err := node.NewActionClient("wrong_goal", example_interfaces_action.FibonacciTypeSupport, nil)
	require.NoError(t, err)
	_, _, err = client.SendGoal(context.Background(), std_msgs_msg.NewString())
	require.ErrorIs(t, err, rclgo.ErrInvalidMessage)
	_, _, err = client.SendGoal(context.Background(), nil)
	require.True(t, errors.Is(err, rclgo.ErrInvalidMessage), "nil goal: %v", err)
}
