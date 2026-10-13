package rostime_test

import (
	"context"
	"fmt"
	"go.uber.org/goleak"
	"sync/atomic"
	"testing"
	"time"

	builtin_interfaces_msg "github.com/merlindrones/rclgo/pkg/msgs/builtin_interfaces/msg"
	rosgraph_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/rosgraph_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/rostime"
	"github.com/stretchr/testify/require"
)

// Simulated time end to end, after rclcpp's test_time_source and rclpy's
// test_time_source: a simulator publishes /clock and a node with
// use_sim_time follows it.

func TestMain(m *testing.M) {
	rclgotest.UseIsolatedDomain()
	goleak.VerifyTestMain(m)
}

// simulator publishes /clock from its own context, as Gazebo or a bag
// replay would.
type simulator struct {
	t   *testing.T
	pub *rclgo.Publisher
	now atomic.Int64 // nanoseconds
}

func newSimulator(t *testing.T) *simulator {
	t.Helper()
	node := rclgotest.Node(t, "sim", "")
	pub, err := node.NewPublisher("/clock", rosgraph_msgs_msg.ClockTypeSupport, &rclgo.PublisherOptions{Qos: qos.NewClockProfile()})
	require.NoError(t, err)
	return &simulator{t: t, pub: pub}
}

// set publishes time d after the epoch.
func (s *simulator) set(d time.Duration) {
	s.t.Helper()
	s.now.Store(int64(d))
	msg := rosgraph_msgs_msg.NewClock()
	msg.Clock = builtin_interfaces_msg.Time{Sec: int32(d / time.Second), Nanosec: uint32(d % time.Second)}
	require.NoError(s.t, s.pub.Publish(msg))
}

// run keeps publishing the current time every 10 ms until the test ends or
// stop is called, so that a best-effort /clock subscriber that joins late
// still receives it.
func (s *simulator) run() (stop func()) {
	ctx, cancel := context.WithCancel(context.Background())
	s.t.Cleanup(cancel)
	go func() {
		tick := time.NewTicker(10 * time.Millisecond)
		defer tick.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-tick.C:
				msg := rosgraph_msgs_msg.NewClock()
				d := time.Duration(s.now.Load())
				msg.Clock = builtin_interfaces_msg.Time{Sec: int32(d / time.Second), Nanosec: uint32(d % time.Second)}
				s.pub.Publish(msg) //nolint:errcheck
			}
		}
	}()
	return cancel
}

// newSimNode returns a spinning node with a parameter manager and a rostime
// clock; useSim sets use_sim_time before the clock is created.
func newSimNode(t *testing.T, name string, useSim bool) (*params.Manager, *rostime.Clock) {
	t.Helper()
	node := rclgotest.Node(t, name, "/sim_test")
	m, err := params.NewManager(node)
	require.NoError(t, err)
	if useSim {
		_, err = m.SetValue("use_sim_time", params.Value{Kind: params.KindBool, Bool: true})
		require.NoError(t, err)
	}
	clock, err := rostime.NewClock(context.Background(), node, m)
	require.NoError(t, err)
	t.Cleanup(clock.Shutdown)
	rclgotest.Spin(t, node.Context().Spin)
	return m, clock
}

func requireSimNow(t *testing.T, clock *rostime.Clock, want time.Duration) {
	t.Helper()
	rclgotest.Eventually(t, "sim time "+want.String(), func() error {
		if got := clock.Now(); !got.Equal(time.Unix(0, int64(want))) {
			return fmt.Errorf("Now() = %v", got)
		}
		return nil
	})
}

func TestSimTimeFollowsClock(t *testing.T) {
	sim := newSimulator(t)
	sim.set(100 * time.Second)
	sim.run()
	_, clock := newSimNode(t, "follows", true)
	require.True(t, clock.IsSimTime())
	requireSimNow(t, clock, 100*time.Second)
	sim.set(105*time.Second + 500*time.Millisecond)
	requireSimNow(t, clock, 105*time.Second+500*time.Millisecond)
}

// Before the first /clock message, sim time is zero (rclcpp, rclpy).
func TestSimTimeIsZeroBeforeFirstClock(t *testing.T) {
	_, clock := newSimNode(t, "zero", true)
	require.True(t, clock.IsSimTime())
	require.True(t, clock.Now().Equal(time.Unix(0, 0)), "Now() = %v before any /clock", clock.Now())
}

// Setting use_sim_time at runtime (as `ros2 param set` does) switches the
// clock both ways.
func TestSimTimeToggledAtRuntime(t *testing.T) {
	sim := newSimulator(t)
	sim.set(42 * time.Second)
	sim.run()
	m, clock := newSimNode(t, "toggle", false)
	require.False(t, clock.IsSimTime())
	require.WithinDuration(t, time.Now(), clock.Now(), time.Second)

	_, err := m.SetValue("use_sim_time", params.Value{Kind: params.KindBool, Bool: true})
	require.NoError(t, err)
	rclgotest.Eventually(t, "switch to sim time", func() error {
		if !clock.IsSimTime() {
			return fmt.Errorf("still wall time")
		}
		return nil
	})
	requireSimNow(t, clock, 42*time.Second)

	_, err = m.SetValue("use_sim_time", params.Value{Kind: params.KindBool, Bool: false})
	require.NoError(t, err)
	rclgotest.Eventually(t, "switch back to wall time", func() error {
		if clock.IsSimTime() {
			return fmt.Errorf("still sim time")
		}
		return nil
	})
	require.WithinDuration(t, time.Now(), clock.Now(), time.Second)
}

// A sim-time sleep returns when its context ends, even if the simulator is
// paused and no /clock message arrives.
func TestSimSleepCanceledWhilePaused(t *testing.T) {
	sim := newSimulator(t)
	sim.set(10 * time.Second)
	pause := sim.run()
	_, clock := newSimNode(t, "paused", true)
	requireSimNow(t, clock, 10*time.Second)
	pause() // a paused simulator may publish nothing
	time.Sleep(50 * time.Millisecond)

	ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
	defer cancel()
	done := make(chan error, 1)
	go func() { done <- clock.Sleep(ctx, time.Hour) }() // the simulator never gets there
	select {
	case err := <-done:
		require.ErrorIs(t, err, context.DeadlineExceeded)
	case <-time.After(2 * time.Second):
		t.Fatal("sim-time sleep ignored its context")
	}
}

// Switching back to wall time wakes sleepers waiting on sim time.
func TestSwitchToWallTimeWakesSimSleepers(t *testing.T) {
	sim := newSimulator(t)
	sim.set(10 * time.Second)
	sim.run()
	m, clock := newSimNode(t, "wake", true)
	requireSimNow(t, clock, 10*time.Second)

	done := make(chan error, 1)
	go func() { done <- clock.SleepUntil(context.Background(), time.Unix(20, 0)) }()
	time.Sleep(100 * time.Millisecond)
	_, err := m.SetValue("use_sim_time", params.Value{Kind: params.KindBool, Bool: false})
	require.NoError(t, err)
	select {
	case err := <-done:
		// 1970+20 s is long past in wall time.
		require.NoError(t, err)
	case <-time.After(2 * time.Second):
		t.Fatal("sleeper on sim time not woken by switching to wall time")
	}
}

// A Rate that overruns resets instead of firing a burst to catch up (rclcpp
// Rate::sleep).
func TestRateResetsAfterOverrun(t *testing.T) {
	_, clock := newSimNode(t, "overrun", false)
	r := clock.Rate(20) // 50 ms
	require.NoError(t, r.Sleep(context.Background()))
	time.Sleep(260 * time.Millisecond) // miss about five periods
	require.NoError(t, r.Sleep(context.Background()))
	start := time.Now()
	require.NoError(t, r.Sleep(context.Background()))
	require.NoError(t, r.Sleep(context.Background()))
	require.Greater(t, time.Since(start), 70*time.Millisecond, "the rate fired a burst after an overrun")
}

// When time jumps backwards (a simulation reset or a looping bag), a Rate
// keeps running instead of waiting for the old time to come round again.
func TestRateAfterTimeJumpsBackwards(t *testing.T) {
	sim := newSimulator(t)
	sim.set(100 * time.Second)
	sim.run()
	_, clock := newSimNode(t, "jump", true)
	requireSimNow(t, clock, 100*time.Second)
	r := clock.Rate(10)

	done := make(chan error, 1)
	go func() {
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		done <- r.Sleep(ctx)
	}()
	time.Sleep(50 * time.Millisecond)
	sim.set(10 * time.Second) // reset
	time.Sleep(50 * time.Millisecond)
	sim.set(10*time.Second + 200*time.Millisecond)
	select {
	case err := <-done:
		require.NoError(t, err)
	case <-time.After(3 * time.Second):
		t.Fatal("Rate.Sleep waits for the time before the jump")
	}
}

// With use_sim_time, rclgo timers follow /clock: a simulation running ten
// times faster than real time makes a 1 s timer fire every 100 ms, and a
// paused simulation stops it (rclcpp timers on a node clock behave the same).
func TestTimerFollowsSimTime(t *testing.T) {
	sim := newSimulator(t)
	sim.set(1000 * time.Second)
	pause := sim.run()
	node := rclgotest.Node(t, "sim_timer", "/sim_test")
	m, err := params.NewManager(node)
	require.NoError(t, err)
	_, err = m.SetValue("use_sim_time", params.Value{Kind: params.KindBool, Bool: true})
	require.NoError(t, err)
	clock, err := rostime.NewClock(context.Background(), node, m)
	require.NoError(t, err)
	t.Cleanup(clock.Shutdown)
	rclgotest.Spin(t, node.Context().Spin)
	requireSimNow(t, clock, 1000*time.Second)

	var fired atomic.Int64
	_, err = node.Context().NewTimer(time.Second, func(*rclgo.Timer) { fired.Add(1) })
	require.NoError(t, err)

	// Advance simulated time 100 ms every 10 ms of wall time, for 1 s.
	for i := 1; i <= 100; i++ {
		sim.set(1000*time.Second + time.Duration(i)*100*time.Millisecond)
		time.Sleep(10 * time.Millisecond)
	}
	got := fired.Load()
	require.GreaterOrEqual(t, got, int64(7), "a 1 s timer fired %d times in 10 s of simulated time", got)
	require.LessOrEqual(t, got, int64(11), "a 1 s timer fired %d times in 10 s of simulated time", got)

	pause()
	before := fired.Load()
	time.Sleep(1500 * time.Millisecond) // more than a period of wall time
	require.Equal(t, before, fired.Load(), "the timer fired while the simulation was paused")
}
