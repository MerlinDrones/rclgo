package rclgo_test

import (
	"os"
	"strconv"
	"strings"
	"testing"

	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/stretchr/testify/require"
)

// residentBytes returns the process's resident set size.
func residentBytes(t *testing.T) int64 {
	t.Helper()
	b, err := os.ReadFile("/proc/self/statm")
	require.NoError(t, err)
	pages, err := strconv.ParseInt(strings.Fields(string(b))[1], 10, 64)
	require.NoError(t, err)
	return pages * int64(os.Getpagesize())
}

// Deserialize must free the C buffer it copies the input into.
func TestDeserializeDoesNotLeak(t *testing.T) {
	msg := std_msgs_msg.NewString()
	msg.Data = strings.Repeat("x", 64<<10)
	buf, err := rclgo.Serialize(msg)
	require.NoError(t, err)
	deserialize := func() {
		_, err := rclgo.Deserialize(buf, std_msgs_msg.StringTypeSupport)
		require.NoError(t, err)
	}
	for range 100 { // warm up allocators
		deserialize()
	}
	before := residentBytes(t)
	const calls = 2000
	for range calls {
		deserialize()
	}
	grown := residentBytes(t) - before
	// A leak of the 64 KiB buffer per call grows the heap by about 128 MiB.
	require.Less(t, grown, int64(32<<20), "resident memory grew by %d MiB over %d calls", grown>>20, calls)
}

// A failing NewClock must close only the clock, not the context.
func TestNewClockFailureKeepsContext(t *testing.T) {
	rclctx, err := newDefaultRCLContext()
	require.NoError(t, err)
	defer rclctx.Close()
	_, err = rclctx.NewClock(rclgo.ClockType(99))
	require.Error(t, err, "an invalid clock type must fail")
	node, err := rclctx.NewNode("after_clock_failure", "/misc_test")
	require.NoError(t, err, "the context was closed by the failed NewClock")
	require.NoError(t, node.Close())
}

// InitLogging(nil) is treated as an empty argument list, as Init(nil) is.
func TestInitLoggingNilArgs(t *testing.T) {
	require.NotPanics(t, func() { _ = rclgo.InitLogging(nil) })
}
