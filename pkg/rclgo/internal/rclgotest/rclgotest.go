// Package rclgotest provides helpers for rclgo's own tests: contexts, Spin,
// discovery waits, service calls and running the ros2 command-line tools
// against rclgo nodes.
package rclgotest

import (
	"context"
	"errors"
	"math/rand"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"testing"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

// Timeout bounds every wait in these helpers.
const Timeout = 10 * time.Second

// UseIsolatedDomain moves the process to a random DDS domain so that tests do
// not see other ROS traffic on the machine. Call it from TestMain before any
// context is created; subprocesses started with ROS2 inherit it.
func UseIsolatedDomain() {
	// 0-101 are valid on Linux; avoid 0, the default most systems use.
	os.Setenv("ROS_DOMAIN_ID", strconv.Itoa(1+rand.Intn(101))) //nolint:gosec,errcheck
}

// Context returns a new context that is closed when the test ends.
func Context(t testing.TB) *rclgo.Context {
	t.Helper()
	ctx, err := rclgo.NewContext(0, nil)
	if err != nil {
		t.Fatalf("NewContext: %v", err)
	}
	t.Cleanup(func() { ctx.Close() })
	return ctx
}

// Node returns a node on a new context.
func Node(t testing.TB, name, namespace string) *rclgo.Node {
	t.Helper()
	n, err := Context(t).NewNode(name, namespace)
	if err != nil {
		t.Fatalf("NewNode(%q, %q): %v", name, namespace, err)
	}
	return n
}

// Spin runs spin until the test ends (or stop is called) and fails the test if
// Spin returns an error other than its context being canceled.
func Spin(t testing.TB, spin func(context.Context) error) (stop func()) {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- spin(ctx) }()
	stopped := false
	stop = func() {
		if stopped {
			return
		}
		stopped = true
		cancel()
		select {
		case err := <-done:
			if err != nil && !errors.Is(err, context.Canceled) {
				t.Errorf("Spin: %v", err)
			}
		case <-time.After(Timeout):
			t.Errorf("Spin did not return after cancel")
		}
	}
	t.Cleanup(stop)
	return stop
}

// Eventually polls f until it returns nil, failing the test with f's last
// error after Timeout. Use it for anything that depends on discovery.
func Eventually(t testing.TB, what string, f func() error) {
	t.Helper()
	deadline := time.Now().Add(Timeout)
	for {
		err := f()
		if err == nil {
			return
		}
		if time.Now().After(deadline) {
			t.Fatalf("%s: %v", what, err)
		}
		time.Sleep(50 * time.Millisecond)
	}
}

// Call sends req on client and returns the response. Requests sent before the
// service is discovered are lost, so it retries with short timeouts until
// Timeout.
func Call(t testing.TB, client *rclgo.Client, req types.Message) types.Message {
	t.Helper()
	var resp types.Message
	Eventually(t, "service call", func() error {
		ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
		defer cancel()
		var err error
		resp, _, err = client.Send(ctx, req)
		return err
	})
	return resp
}

// RequireROS2 skips the test if the ros2 command-line tools are not
// available or the tests run with -short (each ros2 call takes about a second).
func RequireROS2(t testing.TB) {
	t.Helper()
	if testing.Short() {
		t.Skip("ros2 CLI tests are skipped with -short")
	}
	if _, err := exec.LookPath("ros2"); err != nil {
		t.Skip("ros2 command-line tools not found; source a ROS 2 setup.bash")
	}
}

// ROS2 runs the ros2 command-line tool with args and returns its combined
// output. Each run discovers the graph from scratch (use --no-daemon), so
// while ros2 reports that the node or its services are not found yet, it
// retries until Timeout.
func ROS2(t testing.TB, args ...string) (string, error) {
	t.Helper()
	deadline := time.Now().Add(Timeout)
	for {
		out, err := ros2Once(t, args...)
		notYet := strings.Contains(out, "Node not found") || strings.Contains(out, "Wait for service timed out")
		if !notYet || time.Now().After(deadline) {
			return out, err
		}
		time.Sleep(200 * time.Millisecond)
	}
}

func ros2Once(t testing.TB, args ...string) (string, error) {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), Timeout)
	defer cancel()
	out, err := exec.CommandContext(ctx, "ros2", args...).CombinedOutput()
	if ctx.Err() != nil {
		t.Fatalf("ros2 %s timed out:\n%s", strings.Join(args, " "), out)
	}
	return string(out), err
}

// EventuallyROS2 runs ros2 with args until it succeeds and its output
// contains want; ros2 needs to discover the node first.
func EventuallyROS2(t testing.TB, want string, args ...string) string {
	t.Helper()
	var out string
	Eventually(t, "ros2 "+strings.Join(args, " "), func() error {
		var err error
		out, err = ROS2(t, args...)
		if err != nil {
			return errors.New(err.Error() + ": " + out)
		}
		if !strings.Contains(out, want) {
			return errors.New("output does not contain " + strconv.Quote(want) + ": " + out)
		}
		return nil
	})
	return out
}
