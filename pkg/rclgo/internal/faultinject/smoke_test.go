package faultinject_test

import (
	"context"
	"errors"
	"strings"
	"testing"
	"time"

	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/faultinject"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"go.uber.org/goleak"
)

func TestMain(m *testing.M) {
	rclgotest.UseIsolatedDomain()
	goleak.VerifyTestMain(m)
}

// A call from rclgo is routed through its wrapper: the chosen call fails with
// the rcl error message, the next one does not.
func TestInjectedFaultReachesCaller(t *testing.T) {
	node := rclgotest.Node(t, "smoke", "/faultinject")
	pub, err := node.NewPublisher("smoke", std_msgs_msg.StringTypeSupport, nil)
	if err != nil {
		t.Fatal(err)
	}
	faultinject.Arm(1, "rcl_publish", faultinject.RetError)
	err = pub.Publish(std_msgs_msg.NewString())
	calls, failed := faultinject.Disarm()
	if err == nil || !strings.Contains(err.Error(), "injected fault in rcl_publish") {
		t.Fatalf("Publish with an injected fault: %v", err)
	}
	if calls != 1 || failed != "rcl_publish" {
		t.Fatalf("calls %d, failed %q", calls, failed)
	}
	if err := pub.Publish(std_msgs_msg.NewString()); err != nil {
		t.Fatalf("Publish after Disarm: %v", err)
	}
}

// A node that cannot be finalized is leaked with its context, rather than
// freed while rcl's rosout map or the middleware still use them.
func TestNodeFiniFailureLeaksNodeAndContext(t *testing.T) {
	for _, fn := range []string{"rcl_node_fini", "rcl_logging_rosout_fini_publisher_for_node"} {
		t.Run(fn, func(t *testing.T) {
			ctx, err := rclgo.NewContext(0, nil)
			if err != nil {
				t.Fatal(err)
			}
			node, err := ctx.NewNode("leaked", "/faultinject")
			if err != nil {
				t.Fatal(err)
			}
			faultinject.Arm(1, fn, faultinject.RetError)
			err = node.Close()
			faultinject.Disarm()
			if err == nil || !strings.Contains(err.Error(), "not finalized; its memory is leaked") {
				t.Fatalf("node.Close: %v", err)
			}
			if err := ctx.Close(); err == nil || !strings.Contains(err.Error(), "context not finalized") {
				t.Fatalf("ctx.Close: %v", err)
			}
			// New nodes with the same name, in new contexts, keep working.
			for i := 0; i < 20; i++ {
				if errs := scenarioNode(t); len(errs) > 0 {
					t.Fatalf("after the leak: %v", errs)
				}
			}
		})
	}
}

// Spin returns after its context is canceled even if waking it fails once.
func TestSpinReturnsWhenWakingFails(t *testing.T) {
	ctx := rclgotest.Context(t)
	spinCtx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- ctx.Spin(spinCtx) }()
	time.Sleep(100 * time.Millisecond)
	faultinject.Arm(1, "rcl_trigger_guard_condition", faultinject.RetError)
	cancel()
	select {
	case err := <-done:
		if _, failed := faultinject.Disarm(); failed == "" {
			t.Fatal("the trigger was not made to fail")
		}
		if !errors.Is(err, context.Canceled) {
			t.Fatalf("Spin: %v", err)
		}
	case <-time.After(5 * time.Second):
		faultinject.Disarm()
		t.Fatal("Spin did not return")
	}
}

// A result request is answered even if the goal's final state can't be set
// in rcl: it gets the state the server decided instead of waiting forever.
func TestResultAnsweredWhenFinalStateUpdateFails(t *testing.T) {
	node := rclgotest.Node(t, "result_fault", "/faultinject")
	release := make(chan struct{})
	_, err := example_interfaces_action.NewFibonacciServer(node, "fib", example_interfaces_action.NewFibonacciAction(
		func(_ context.Context, goal *example_interfaces_action.FibonacciGoalHandle) (*example_interfaces_action.Fibonacci_Result, error) {
			if _, err := goal.Accept(); err != nil {
				return nil, err
			}
			<-release
			return example_interfaces_action.NewFibonacci_Result(), nil
		}), nil)
	if err != nil {
		t.Fatal(err)
	}
	client, err := example_interfaces_action.NewFibonacciClient(node, "fib", nil)
	if err != nil {
		t.Fatal(err)
	}
	rclgotest.Spin(t, node.Context().Spin)
	var id *types.GoalID
	rclgotest.Eventually(t, "goal accepted", func() error {
		ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
		defer cancel()
		resp, goalID, err := client.SendGoal(ctx, example_interfaces_action.NewFibonacci_Goal())
		if err == nil && !resp.Accepted {
			err = errors.New("goal rejected")
		}
		id = goalID
		return err
	})
	results := make(chan *example_interfaces_action.Fibonacci_GetResult_Response, 1)
	go func() {
		resp, err := client.GetResult(context.Background(), id)
		if err != nil {
			t.Error(err)
		}
		results <- resp
	}()
	time.Sleep(200 * time.Millisecond) // the result request is waiting
	faultinject.Arm(1, "rcl_action_update_goal_state", faultinject.RetError)
	close(release)
	select {
	case resp := <-results:
		if _, failed := faultinject.Disarm(); failed == "" {
			t.Fatal("the state update was not made to fail")
		}
		if resp != nil && resp.Status != int8(rclgo.GoalSucceeded) {
			t.Fatalf("status %d, want SUCCEEDED (what the server decided)", resp.Status)
		}
	case <-time.After(10 * time.Second):
		faultinject.Disarm()
		t.Fatal("the result request was never answered")
	}
}
