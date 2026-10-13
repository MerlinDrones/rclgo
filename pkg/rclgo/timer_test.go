package rclgo_test

import (
	"sync"
	"testing"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/stretchr/testify/require"
)

// A periodic timer must keep its period: the next expiry is one period after
// the previous expiry, not one period after Spin serviced it. Restarting the
// period on each tick adds the wake-up latency to every period (measured at
// 0.6-1.2% for a 20 ms timer), which accumulates without bound. Keeping the
// schedule makes the average interval match the period to within
// microseconds, even though the callback works for a quarter of the period.
func TestTimerKeepsPeriodUnderCallbackLoad(t *testing.T) {
	const (
		period = 20 * time.Millisecond
		work   = 5 * time.Millisecond
		run    = 2 * time.Second
	)
	rclctx := newSpinTestContext(t)
	var (
		mu    sync.Mutex
		ticks []time.Time
	)
	_, err := rclctx.NewTimer(period, func(*rclgo.Timer) {
		mu.Lock()
		ticks = append(ticks, time.Now())
		mu.Unlock()
		time.Sleep(work)
	})
	require.NoError(t, err)
	stop := startSpin(t, rclctx.Spin)
	time.Sleep(run)
	require.NoError(t, stop())

	mu.Lock()
	defer mu.Unlock()
	require.Greater(t, len(ticks), 10, "timer barely fired")
	// The least-squares slope of tick time against tick number: accumulated
	// drift shows in it, while the scheduling jitter of single ticks (large
	// under -race or load) averages out, unlike in last-minus-first.
	var sumI, sumT, sumII, sumIT float64
	for i, tick := range ticks {
		x, y := float64(i), float64(tick.Sub(ticks[0]))
		sumI, sumT, sumII, sumIT = sumI+x, sumT+y, sumII+x*x, sumIT+x*y
	}
	n := float64(len(ticks))
	avg := time.Duration((n*sumIT - sumI*sumT) / (n*sumII - sumI*sumI))
	t.Logf("%d ticks, average interval %v (period %v)", len(ticks), avg, period)
	require.InDelta(t, float64(period), float64(avg), float64(period)/400,
		"average interval %v drifts from the period %v", avg, period)
}
