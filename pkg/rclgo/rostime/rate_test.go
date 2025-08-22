package rostime

import (
	"context"
	"testing"
	"time"
)

func TestRate_WallClock(t *testing.T) {
	c := newTestClock()
	r := c.Rate(100) // 100 Hz -> 10ms period

	start := time.Now()
	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()

	if err := r.Sleep(ctx); err != nil {
		t.Fatalf("Rate.Sleep (1) failed: %v", err)
	}
	if err := r.Sleep(ctx); err != nil {
		t.Fatalf("Rate.Sleep (2) failed: %v", err)
	}

	elapsed := time.Since(start)
	// Two periods ~20ms; allow slack on busy CI
	if elapsed < 15*time.Millisecond {
		t.Fatalf("Rate.Sleep too short: %v", elapsed)
	}
	if elapsed > 300*time.Millisecond {
		t.Fatalf("Rate.Sleep too long: %v", elapsed)
	}
}

func TestRate_SimTime(t *testing.T) {
	c := newTestClock()
	// Enable sim time and pin a start
	c.mu.Lock()
	c.useSim = true
	base := time.Unix(300, 0)
	c.now = base
	c.mu.Unlock()

	r := c.Rate(20) // 20 Hz -> 50ms period

	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()

	// First sleep: should block until we advance by 50ms
	done1 := make(chan error, 1)
	go func() { done1 <- r.Sleep(ctx) }()
	time.Sleep(10 * time.Millisecond)
	advanceSim(c, base.Add(50*time.Millisecond))
	select {
	case err := <-done1:
		if err != nil {
			t.Fatalf("Rate.Sleep (sim 1) failed: %v", err)
		}
	case <-time.After(200 * time.Millisecond):
		t.Fatal("Rate.Sleep (sim 1) did not return after sim advance")
	}

	// Second sleep: advance another 50ms relative to last
	done2 := make(chan error, 1)
	go func() { done2 <- r.Sleep(ctx) }()
	time.Sleep(10 * time.Millisecond)
	advanceSim(c, base.Add(100*time.Millisecond))
	select {
	case err := <-done2:
		if err != nil {
			t.Fatalf("Rate.Sleep (sim 2) failed: %v", err)
		}
	case <-time.After(200 * time.Millisecond):
		t.Fatal("Rate.Sleep (sim 2) did not return after sim advance")
	}
}
