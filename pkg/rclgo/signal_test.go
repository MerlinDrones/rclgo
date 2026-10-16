package rclgo_test

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"os/signal"
	"slices"
	"strings"
	"sync"
	"syscall"
	"testing"
	"time"

	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

// Signals, after system_tests' test_sigint_handler: a node that stops on
// SIGINT or SIGTERM with signal.NotifyContext, as rclgo programs should, exits
// cleanly whatever it is doing, and leaves the ROS graph at once. rclgo
// installs no signal handler of its own; without one, Go's default applies and
// the process ends without finalizing rcl.

const signalHelperNode = "sig_helper"

// TestSignalHelper is the node TestSignals runs in a child process.
func TestSignalHelper(t *testing.T) {
	mode := os.Getenv("RCLGO_SIGNAL_HELPER")
	if mode == "" {
		t.Skip("the child process of TestSignals")
	}
	os.Setenv("ROS_DOMAIN_ID", os.Getenv("RCLGO_SIGNAL_DOMAIN")) //nolint:errcheck
	ctx := context.Background()
	if mode != "no-handler" {
		var stop context.CancelFunc
		ctx, stop = signal.NotifyContext(ctx, syscall.SIGINT, syscall.SIGTERM)
		defer stop()
	}
	rctx, err := rclgo.NewContext(0, nil)
	if err != nil {
		t.Fatal(err)
	}
	defer func() {
		if err := rctx.Close(); err != nil {
			t.Errorf("Close: %v", err)
		}
		fmt.Println("CLOSED")
	}()
	node, err := rctx.NewNode(signalHelperNode, "/signal_test")
	if err != nil {
		t.Fatal(err)
	}
	pub, err := node.NewPublisher("sig_chatter", std_msgs_msg.StringTypeSupport, nil)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := rctx.NewTimer(50*time.Millisecond, func(*rclgo.Timer) { pub.Publish(std_msgs_msg.NewString()) }); err != nil { //nolint:errcheck
		t.Fatal(err)
	}
	spinErr := make(chan error, 1)
	go func() { spinErr <- rctx.Spin(ctx) }()

	switch mode {
	case "spin", "no-handler":
		fmt.Println("READY")
	case "service-call":
		cli, err := node.NewClient("sig_never_answers", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
		if err != nil {
			t.Fatal(err)
		}
		fmt.Println("READY")
		_, _, err = cli.Send(ctx, example_interfaces_srv.NewAddTwoInts_Request())
		if !errors.Is(err, context.Canceled) {
			t.Errorf("Send: %v, want context.Canceled", err)
		}
	case "action-goal":
		cli, err := node.NewActionClient("sig_never_finishes", example_interfaces_action.FibonacciTypeSupport, nil)
		if err != nil {
			t.Fatal(err)
		}
		// Goals sent before discovery completes are dropped: probe until the
		// server answers.
		for {
			c, cancel := context.WithTimeout(ctx, 300*time.Millisecond)
			_, _, err := cli.SendGoal(c, example_interfaces_action.NewFibonacci_Goal())
			cancel()
			if err == nil {
				break
			}
			if ctx.Err() != nil {
				t.Fatalf("no action server before the signal: %v", err)
			}
		}
		var once sync.Once
		_, _, err = cli.WatchGoal(ctx, example_interfaces_action.NewFibonacci_Goal(), func(context.Context, types.Message) {
			once.Do(func() { fmt.Println("READY") })
		})
		if !errors.Is(err, context.Canceled) {
			t.Errorf("WatchGoal: %v, want context.Canceled", err)
		}
	default:
		t.Fatalf("unknown mode %q", mode)
	}
	select {
	case err := <-spinErr:
		if !errors.Is(err, context.Canceled) {
			t.Errorf("Spin: %v, want context.Canceled", err)
		}
	case <-time.After(rclgotest.Timeout):
		t.Error("Spin did not return after the signal")
	}
}

// syncBuffer is a bytes.Buffer safe for a process writing while the test
// reads.
type syncBuffer struct {
	mu sync.Mutex
	b  bytes.Buffer
}

func (s *syncBuffer) Write(p []byte) (int, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.b.Write(p)
}

func (s *syncBuffer) String() string {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.b.String()
}

func helperInGraph(t *testing.T, observer *rclgo.Node) bool {
	t.Helper()
	names, _, err := observer.GetNodeNames()
	if err != nil {
		t.Fatalf("GetNodeNames: %v", err)
	}
	return slices.Contains(names, signalHelperNode)
}

func TestSignals(t *testing.T) {
	observer := rclgotest.Node(t, "sig_observer", "/signal_test")
	// The service never answers and the goal never finishes, so the helper is
	// blocked in them when the signal arrives.
	_, err := observer.NewService("sig_never_answers", example_interfaces_srv.AddTwoIntsTypeSupport, nil,
		func(*rclgo.ServiceInfo, types.Message, rclgo.ServiceResponseSender) {})
	if err != nil {
		t.Fatal(err)
	}
	_, err = observer.NewActionServer("sig_never_finishes", rclgo.NewAction(example_interfaces_action.FibonacciTypeSupport,
		func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
			fs, err := goal.Accept()
			if err != nil {
				return nil, err
			}
			for {
				fs.Send(example_interfaces_action.NewFibonacci_Feedback()) //nolint:errcheck
				select {
				case <-ctx.Done():
					return nil, ctx.Err()
				case <-time.After(50 * time.Millisecond):
				}
			}
		}), nil)
	if err != nil {
		t.Fatal(err)
	}
	rclgotest.Spin(t, observer.Context().Spin)

	for _, tc := range []struct {
		mode string
		sig  syscall.Signal
	}{
		{"spin", syscall.SIGINT},
		{"spin", syscall.SIGTERM},
		{"service-call", syscall.SIGINT},
		{"action-goal", syscall.SIGINT},
		{"action-goal", syscall.SIGTERM},
		{"no-handler", syscall.SIGINT},
	} {
		t.Run(tc.mode+"/"+tc.sig.String(), func(t *testing.T) {
			var out syncBuffer
			cmd := exec.Command(os.Args[0], "-test.run=^TestSignalHelper$", "-test.v")
			cmd.Env = append(os.Environ(), "RCLGO_SIGNAL_HELPER="+tc.mode, "RCLGO_SIGNAL_DOMAIN="+os.Getenv("ROS_DOMAIN_ID"))
			cmd.Stdout, cmd.Stderr = &out, &out
			cmd.SysProcAttr = &syscall.SysProcAttr{Pdeathsig: syscall.SIGKILL}
			if err := cmd.Start(); err != nil {
				t.Fatal(err)
			}
			var waitErr error
			exited := make(chan struct{})
			go func() {
				waitErr = cmd.Wait()
				close(exited)
			}()
			t.Cleanup(func() {
				cmd.Process.Kill() //nolint:errcheck
				<-exited
			})

			rclgotest.Eventually(t, "helper ready and in the graph", func() error {
				if !strings.Contains(out.String(), "READY") || !helperInGraph(t, observer) {
					return fmt.Errorf("not yet; output:\n%s", out.String())
				}
				return nil
			})
			// Let the helper block in the call or goal it is waiting for.
			time.Sleep(200 * time.Millisecond)
			sent := time.Now()
			if err := cmd.Process.Signal(tc.sig); err != nil {
				t.Fatal(err)
			}
			select {
			case <-exited:
			case <-time.After(5 * time.Second):
				t.Fatalf("still running 5s after %v; output:\n%s", tc.sig, out.String())
			}
			exitTime := time.Since(sent)

			if tc.mode == "no-handler" {
				var exitErr *exec.ExitError
				if !errors.As(waitErr, &exitErr) || exitErr.Sys().(syscall.WaitStatus).Signal() != tc.sig {
					t.Fatalf("without a handler the process should die of %v; got %v\n%s", tc.sig, waitErr, out.String())
				}
				if strings.Contains(out.String(), "CLOSED") {
					t.Errorf("rcl was finalized without a handler")
				}
				t.Logf("killed by %v after %v; rcl not finalized", tc.sig, exitTime)
				return
			}
			if waitErr != nil {
				t.Fatalf("exit after %v: %v; output:\n%s", tc.sig, waitErr, out.String())
			}
			if !strings.Contains(out.String(), "CLOSED") {
				t.Fatalf("exited without closing its context; output:\n%s", out.String())
			}
			// A cleanly finalized participant announces that it leaves.
			deadline := time.Now().Add(3 * time.Second)
			for helperInGraph(t, observer) {
				if time.Now().After(deadline) {
					t.Fatalf("the node is still in the graph 3s after exiting")
				}
				time.Sleep(20 * time.Millisecond)
			}
			t.Logf("exited %v after %v and left the graph %v after the signal", tc.sig, exitTime, time.Since(sent))
		})
	}
}
