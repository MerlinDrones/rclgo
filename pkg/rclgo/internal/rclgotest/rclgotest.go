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
	"slices"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"testing"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

// Timeout bounds every wait in these helpers.
const Timeout = 10 * time.Second

// UseIsolatedDomain isolates the tests from other ROS traffic: it limits
// discovery to this machine (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, unless
// the environment sets another range) and moves the process to the domain in
// RCLGO_TEST_DOMAIN_ID, or else to a random one. Call it from TestMain before
// any context is created; subprocesses started with ROS2 inherit both.
func UseIsolatedDomain() {
	os.Setenv("ROS_DOMAIN_ID", strconv.Itoa(TestDomainID())) //nolint:errcheck
	if os.Getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") == "" {
		os.Setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST") //nolint:errcheck
	}
}

// TestDomainID returns the domain pinned with RCLGO_TEST_DOMAIN_ID, or a
// random one in 1-101 (valid on Linux; 0 is avoided, as most systems use it).
func TestDomainID() int {
	if id, err := strconv.Atoi(os.Getenv("RCLGO_TEST_DOMAIN_ID")); err == nil {
		return id
	}
	return 1 + rand.Intn(101) //nolint:gosec
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
		Unavailable(t, "ros2 command-line tools not found; source a ROS 2 setup.bash")
	}
}

// Unavailable skips the test because something it needs (the ros2 tools, a
// ROS package, rclpy) is missing, or fails it if RCLGO_REQUIRE_ROS2 is set,
// so that a run meant to cover interoperability can't pass by skipping it.
func Unavailable(t testing.TB, format string, args ...any) {
	t.Helper()
	if os.Getenv("RCLGO_REQUIRE_ROS2") != "" {
		t.Fatalf("RCLGO_REQUIRE_ROS2 is set: "+format, args...)
	}
	t.Skipf(format, args...)
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

// daemonStops records the tests that will stop the ros2 daemon when they end.
var daemonStops sync.Map

// stopDaemon stops the ros2 daemon of this process's domain and RMW. ros2
// starts the daemon in the background without waiting for it, so it may not
// be up yet: retry for a few seconds until one is stopped.
func stopDaemon() {
	deadline := time.Now().Add(5 * time.Second)
	for {
		out, err := exec.Command("ros2", "daemon", "stop").CombinedOutput()
		if err != nil || strings.Contains(string(out), "has been stopped") || time.Now().After(deadline) {
			return
		}
		time.Sleep(300 * time.Millisecond)
	}
}

func ros2Once(t testing.TB, args ...string) (string, error) {
	t.Helper()
	// Without --no-daemon (some verbs, e.g. service call, have no such
	// option), ros2 starts a daemon on the test's domain that outlives the
	// test process, and later test runs that get the same domain see its
	// node and topics. Stop it when the test ends.
	if !slices.Contains(args, "--no-daemon") {
		if _, loaded := daemonStops.LoadOrStore(t, true); !loaded {
			t.Cleanup(func() {
				daemonStops.Delete(t)
				stopDaemon()
			})
		}
	}
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

// Process is a program started by Start, typically a ROS node in another
// language. Its output is collected; it is stopped when the test ends.
type Process struct {
	t   testing.TB
	cmd *exec.Cmd
	mu  sync.Mutex
	out strings.Builder
}

func (p *Process) Write(b []byte) (int, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.out.Write(b)
}

// Output returns everything the process has written so far.
func (p *Process) Output() string {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.out.String()
}

// Signal sends sig to the process itself, not to its process group.
func (p *Process) Signal(sig os.Signal) error {
	return p.cmd.Process.Signal(sig)
}

// WaitFor waits until the process's output contains s and fails the test
// after Timeout.
func (p *Process) WaitFor(s string) {
	p.t.Helper()
	Eventually(p.t, "output "+strconv.Quote(s)+" from "+p.cmd.Path, func() error {
		if strings.Contains(p.Output(), s) {
			return nil
		}
		return errors.New("not yet; output so far:\n" + p.Output())
	})
}

// Start runs name with args in its own process group. When the test ends the
// group gets SIGINT, as Ctrl-C would, and is killed if it is still running a
// few seconds later.
func Start(t testing.TB, name string, args ...string) *Process {
	t.Helper()
	p := &Process{t: t, cmd: exec.Command(name, args...)}
	p.cmd.Stdout, p.cmd.Stderr = p, p
	// Output goes to a pipe, which Python and rcutils would otherwise buffer.
	p.cmd.Env = append(os.Environ(), "PYTHONUNBUFFERED=1", "RCUTILS_LOGGING_BUFFERED_STREAM=0")
	// Pdeathsig: the process dies with the test binary even if the binary
	// crashes before its cleanup runs.
	p.cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true, Pdeathsig: syscall.SIGKILL}
	if err := p.cmd.Start(); err != nil {
		t.Fatalf("start %s: %v", name, err)
	}
	done := make(chan struct{})
	go func() {
		p.cmd.Wait() //nolint:errcheck
		close(done)
	}()
	t.Cleanup(func() {
		pgid := -p.cmd.Process.Pid
		syscall.Kill(pgid, syscall.SIGINT) //nolint:errcheck
		select {
		case <-done:
		case <-time.After(5 * time.Second):
			syscall.Kill(pgid, syscall.SIGKILL) //nolint:errcheck
			<-done
		}
	})
	return p
}

// RunROS2Node starts an executable of a ROS package (an rclcpp or rclpy node),
// as `ros2 run pkg executable args...` would but without the ros2 wrapper
// process, so that the node itself is the child that dies with the test. It
// skips the test if the ros2 tools or the package are missing.
func RunROS2Node(t testing.TB, pkg, executable string, args ...string) *Process {
	t.Helper()
	RequireROS2(t)
	out, err := exec.Command("ros2", "pkg", "prefix", pkg).CombinedOutput()
	if err != nil {
		Unavailable(t, "ROS package %s not installed: %s", pkg, out)
	}
	path := strings.TrimSpace(string(out)) + "/lib/" + pkg + "/" + executable
	if _, err := os.Stat(path); err != nil {
		Unavailable(t, "executable %s not found: %v", path, err)
	}
	return Start(t, path, args...)
}

// Python starts a Python 3 script (an rclpy program); -P keeps the script's
// directory off the module path.
func Python(t testing.TB, script string, args ...string) *Process {
	t.Helper()
	RequireROS2(t)
	if out, err := exec.Command("python3", "-P", "-c", "import rclpy").CombinedOutput(); err != nil {
		Unavailable(t, "rclpy not importable: %s", out)
	}
	return Start(t, "python3", append([]string{"-P", script}, args...)...)
}
