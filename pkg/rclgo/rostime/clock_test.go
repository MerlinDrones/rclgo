package rostime

import (
	"context"
	"sync"
	"testing"
	"time"
)

func newTestClock() *Clock {
	c := &Clock{
		now: time.Now(),
	}
	c.cond = sync.NewCond(&c.mu) // use the same mutex as production code
	return c
}

// advanceSim sets c.now and broadcasts (simulated /clock tick)
func advanceSim(c *Clock, t time.Time) {
	c.mu.Lock()
	c.now = t
	c.mu.Unlock()
	c.cond.Broadcast()
}

// wakeOnCancel wakes any Wait()ers when ctx is canceled, so they can observe ctx.Err().
func wakeOnCancel(c *Clock, ctx context.Context) {
	go func() {
		<-ctx.Done()
		c.mu.Lock()
		c.mu.Unlock()
		c.cond.Broadcast()
	}()
}

func TestNow_WallClock(t *testing.T) {
	c := newTestClock()
	before := time.Now()
	got := c.Now()
	after := time.Now()

	if got.Before(before) || got.After(after.Add(5*time.Millisecond)) {
		t.Fatalf("Now() not within expected wall-clock window: got=%v, before=%v, after=%v", got, before, after)
	}
}

func TestNowMsg_WallClock(t *testing.T) {
	c := newTestClock()
	got := c.NowMsg()
	now := time.Now()
	gotTime := time.Unix(int64(got.Sec), int64(got.Nanosec))
	if d := gotTime.Sub(now); d > 50*time.Millisecond || d < -50*time.Millisecond {
		t.Fatalf("NowMsg off from wall clock by %v (got=%v now=%v)", d, gotTime, now)
	}
}

func TestSleepUntil_WallClock(t *testing.T) {
	c := newTestClock()
	target := time.Now().Add(60 * time.Millisecond)
	ctx, cancel := context.WithTimeout(context.Background(), 300*time.Millisecond)
	defer cancel()

	start := time.Now()
	if err := c.SleepUntil(ctx, target); err != nil {
		t.Fatalf("SleepUntil (wall clock) returned error: %v", err)
	}
	elapsed := time.Since(start)
	if elapsed < 55*time.Millisecond {
		t.Fatalf("SleepUntil woke too early: %v", elapsed)
	}
	if elapsed > 250*time.Millisecond {
		t.Fatalf("SleepUntil took too long: %v", elapsed)
	}
}

func TestSleepUntil_SimTimeAdvance(t *testing.T) {
	c := newTestClock()
	// Enter simulated time mode.
	c.mu.Lock()
	c.useSim = true
	t0 := time.Unix(100, 0)
	c.now = t0
	c.mu.Unlock()

	// Ask to sleep until T1 in sim time
	t1 := t0.Add(150 * time.Millisecond)
	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()

	done := make(chan error, 1)
	go func() {
		done <- c.SleepUntil(ctx, t1)
	}()

	// Give the goroutine time to block
	time.Sleep(20 * time.Millisecond)

	// Advance simulated time in two steps
	advanceSim(c, t0.Add(50*time.Millisecond))
	time.Sleep(20 * time.Millisecond)
	advanceSim(c, t1)

	select {
	case err := <-done:
		if err != nil {
			t.Fatalf("SleepUntil (sim) returned error: %v", err)
		}
	case <-time.After(300 * time.Millisecond):
		t.Fatal("SleepUntil (sim) did not return after advancing to target time")
	}
}

func TestSleepUntil_SimTimeCanceled(t *testing.T) {
	c := newTestClock()
	c.mu.Lock()
	c.useSim = true
	c.now = time.Unix(200, 0)
	c.mu.Unlock()

	target := c.now.Add(10 * time.Second) // far future in sim
	ctx, cancel := context.WithTimeout(context.Background(), 50*time.Millisecond)
	defer cancel()

	// Ensure the waiter wakes on cancel to observe ctx.Err().
	wakeOnCancel(c, ctx)

	err := c.SleepUntil(ctx, target)
	if err == nil {
		t.Fatal("expected context deadline exceeded in sim sleep, got nil")
	}
}

func TestIsSimTimeAndSinceUntil(t *testing.T) {
	c := newTestClock()
	if c.IsSimTime() {
		t.Fatal("expected wall clock by default")
	}
	t0 := time.Now()
	time.Sleep(5 * time.Millisecond)
	if c.Since(t0) <= 0 {
		t.Fatal("expected Since to be > 0")
	}
	t1 := time.Now().Add(5 * time.Millisecond)
	if c.Until(t1) <= 0 {
		t.Fatal("expected Until to be > 0")
	}
}

func TestSleepAndAfter_WallClock(t *testing.T) {
	c := newTestClock()
	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()

	start := time.Now()
	if err := c.Sleep(ctx, 30*time.Millisecond); err != nil {
		t.Fatalf("Sleep failed: %v", err)
	}
	if time.Since(start) < 25*time.Millisecond {
		t.Fatal("Sleep returned too early")
	}

	ch := c.After(ctx, 20*time.Millisecond)
	select {
	case <-ch:
		// ok
	case <-time.After(200 * time.Millisecond):
		t.Fatal("After did not fire within expected time")
	}
}

func TestNow_UsesSimTimeWhenEnabled(t *testing.T) {
	c := newTestClock()
	c.mu.Lock()
	c.useSim = true
	want := time.Unix(1234, 567000000)
	c.now = want
	c.mu.Unlock()

	got := c.Now()
	if !got.Equal(want) {
		t.Fatalf("Now() in sim did not return sim time: got=%v want=%v", got, want)
	}
}
