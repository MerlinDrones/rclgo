package rclgo_test

import (
	"fmt"
	"io"
	"os"
	"runtime"
	"sync"
	"sync/atomic"
	"testing"
	"time"
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/stretchr/testify/require"
)

// countingHandler is an output handler that counts messages from loggers
// whose name starts with prefix and passes everything else on.
func countingHandler(prefix string, n *atomic.Int64) rclgo.LoggingOutputHandler {
	return func(loc unsafe.Pointer, sev int, name unsafe.Pointer, ts int64, format, args unsafe.Pointer) {
		if len(prefix) > 0 && hasPrefixC(name, prefix) {
			n.Add(1)
			return
		}
		rclgo.DefaultLoggingOutputHandler(loc, sev, name, ts, format, args)
	}
}

func hasPrefixC(p unsafe.Pointer, prefix string) bool {
	b := unsafe.Slice((*byte)(p), len(prefix))
	for i := range b {
		if b[i] == 0 || b[i] != prefix[i] {
			return false
		}
	}
	return true
}

// Logging from many goroutines at once delivers every message to the output
// handler, while loggers change level and nodes come and go.
func TestLoggingFromManyGoroutines(t *testing.T) {
	rclctx := newSpinTestContext(t)
	prefix := fmt.Sprintf("manylog%d", time.Now().UnixNano())
	var n atomic.Int64
	rclgo.SetLoggingOutputHandler(countingHandler(prefix, &n))
	t.Cleanup(func() { rclgo.SetLoggingOutputHandler(nil) })

	const goroutines, each = 16, 200
	var wg sync.WaitGroup
	for g := 0; g < goroutines; g++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			l := rclgo.GetLogger(fmt.Sprintf("%s.g%d", prefix, g))
			for i := 0; i < each; i++ {
				require.NoError(t, l.Infof("message %d", i))
				if i%50 == 0 {
					require.NoError(t, l.SetLevel(rclgo.LogSeverityDebug))
				}
			}
		}()
	}
	wg.Add(1)
	go func() { // nodes, with their /rosout publishers, come and go meanwhile
		defer wg.Done()
		for i := 0; i < 10; i++ {
			node, err := rclctx.NewNode(fmt.Sprintf("manylog_node_%d", i), "/logging_test")
			require.NoError(t, err)
			require.NoError(t, node.Logger().Info("from a node"))
			require.NoError(t, node.Close())
		}
	}()
	wg.Wait()
	require.EqualValues(t, goroutines*each, n.Load())
}

// A panic in the output handler is printed to stderr and doesn't reach the
// logging call, and logging keeps working afterwards on other threads (the
// panic must not leave the C logging mutex locked).
func TestLoggingHandlerPanic(t *testing.T) {
	newSpinTestContext(t)
	prefix := fmt.Sprintf("paniclog%d", time.Now().UnixNano())
	var n atomic.Int64
	counting := countingHandler(prefix, &n)
	rclgo.SetLoggingOutputHandler(func(loc unsafe.Pointer, sev int, name unsafe.Pointer, ts int64, format, args unsafe.Pointer) {
		if hasPrefixC(name, prefix+".bad") {
			panic("handler panic")
		}
		counting(loc, sev, name, ts, format, args)
	})
	t.Cleanup(func() { rclgo.SetLoggingOutputHandler(nil) })

	r, w, err := os.Pipe()
	require.NoError(t, err)
	stderr := os.Stderr
	os.Stderr = w
	// The panicking call runs on its own OS thread, which stays alive while
	// another thread logs: a lock left held by that thread would block it.
	logged, release := make(chan any), make(chan struct{})
	go func() {
		runtime.LockOSThread()
		defer func() {
			logged <- recover()
			<-release
			runtime.UnlockOSThread()
		}()
		rclgo.GetLogger(prefix + ".bad").Info("boom") //nolint:errcheck
	}()
	defer close(release)
	require.Nil(t, <-logged, "the handler's panic reached the logging call")
	os.Stderr = stderr
	require.NoError(t, w.Close())
	out, err := io.ReadAll(r)
	require.NoError(t, err)
	require.Contains(t, string(out), "the logging output handler panicked: handler panic")

	done := make(chan struct{})
	go func() {
		defer close(done)
		rclgo.GetLogger(prefix + ".good").Info("other goroutine") //nolint:errcheck
		rclgo.GetLogger(prefix + ".good").Info("again")           //nolint:errcheck
	}()
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		t.Fatal("logging from another thread blocked after the handler panicked")
	}
	require.EqualValues(t, 2, n.Load())
}
