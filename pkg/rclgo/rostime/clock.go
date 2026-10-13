/*
This file is part of rclgo

Copyright © 2025 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rostime

import (
	"context"
	"errors"
	"sync"
	"time"

	"github.com/merlindrones/rclgo/pkg/msgs/builtin_interfaces/msg"
	"github.com/merlindrones/rclgo/pkg/msgs/rosgraph_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/utils"
)

// ErrTimeJumpedBack is returned by SleepUntil (and Sleep) when simulated time
// jumped backwards while sleeping, for example when a simulation is reset or a
// bag replay loops. The target time may now be far away; Rate resets itself on
// it, as rclcpp's Rate does.
var ErrTimeJumpedBack = errors.New("rostime: time jumped backwards")

// Clock is a node's ROS time: the system time, or simulated time from /clock
// while the node's use_sim_time parameter is true (as with rclcpp and rclpy).
//
// While simulated time is active, the clock also drives the ROS-time clock of
// the node's context (rclgo.Clock.SetROSTimeOverride), so that rclgo timers
// and action servers on that context follow simulated time. That clock is
// shared by every node of the context.
type Clock struct {
	mu     sync.RWMutex
	useSim bool
	now    time.Time // simulated time; the epoch until the first /clock
	jumps  uint64    // backwards jumps of simulated time
	cond   *sync.Cond

	node       *rclgo.Node
	pm         *params.Manager
	sub        *rclgo.Subscription
	removeHook func()
	// switchMu serializes switches between system and simulated time;
	// overriding and the /clock subscription change only under it.
	switchMu   sync.Mutex
	overriding bool // whether the context's rcl clock is in ROS-time override
}

// NewClock creates a node's clock. It follows the use_sim_time parameter of
// pm, if pm is not nil: its value now, and every later change (for example
// from `ros2 param set`). Call Shutdown when done with it.
func NewClock(_ context.Context, node *rclgo.Node, pm *params.Manager) (*Clock, error) {
	c := &Clock{node: node, pm: pm, now: time.Unix(0, 0)}
	c.cond = sync.NewCond(&c.mu)
	if pm == nil {
		return c, nil
	}
	// Parameter event stamps use ROS time.
	pm.SetNowProvider(c.NowMsg)
	// Register for changes first, then sync with the current value: a change
	// in between is then seen by one of the two, and both apply the value the
	// parameter has when they run.
	c.removeHook = pm.AddPostSetCallback(func(ps []params.Parameter) {
		for _, p := range ps {
			if p.Name == "use_sim_time" {
				c.syncUseSim()
				return
			}
		}
	})
	c.syncUseSim()
	return c, nil
}

// syncUseSim switches to simulated or system time to match the use_sim_time
// parameter as it is now.
func (c *Clock) syncUseSim() {
	c.switchMu.Lock()
	defer c.switchMu.Unlock()
	p, ok := c.pm.Get("use_sim_time")
	c.setUseSimLocked(ok && p.Value.Kind == params.KindBool && p.Value.Bool)
}

func (c *Clock) IsSimTime() bool {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.useSim
}

// Now returns the current time from the active source. Simulated time is the
// epoch until the first /clock message arrives.
func (c *Clock) Now() time.Time {
	c.mu.RLock()
	defer c.mu.RUnlock()
	if !c.useSim {
		return time.Now()
	}
	return c.now
}

// NowMsg returns builtin_interfaces/msg/Time.
func (c *Clock) NowMsg() builtin_interfaces_msg.Time {
	return utils.ROSTime(c.Now())
}

// Since returns c.Now().Sub(t).
func (c *Clock) Since(t time.Time) time.Duration {
	return c.Now().Sub(t)
}

// Until returns t.Sub(c.Now()).
func (c *Clock) Until(t time.Time) time.Duration {
	return t.Sub(c.Now())
}

// Sleep sleeps for duration d according to the active time source.
func (c *Clock) Sleep(ctx context.Context, d time.Duration) error {
	return c.SleepUntil(ctx, c.Now().Add(d))
}

// After returns a channel that closes when c.Now() has advanced by d.
// The channel does not close on context cancel; it only closes on time reach.
func (c *Clock) After(ctx context.Context, d time.Duration) <-chan struct{} {
	ch := make(chan struct{})
	target := c.Now().Add(d)
	go func() {
		if err := c.SleepUntil(ctx, target); err == nil {
			close(ch)
		}
	}()
	return ch
}

// Rate creates a ROS-aware rate helper (respects sim time).
func (c *Clock) Rate(hz float64) *Rate {
	if hz <= 0 {
		hz = 1
	}
	period := time.Duration(float64(time.Second) / hz)
	return &Rate{
		clock:  c,
		period: period,
		last:   c.Now(),
	}
}

// SleepUntil sleeps until t according to the active clock. It returns
// ctx.Err() when ctx ends, and ErrTimeJumpedBack when simulated time jumps
// backwards. When the clock switches between system and simulated time while
// sleeping, the sleep continues on the new source.
func (c *Clock) SleepUntil(ctx context.Context, t time.Time) error {
	// Wake the condition-variable wait below when ctx ends; /clock may not
	// arrive at all while a simulation is paused.
	stop := context.AfterFunc(ctx, func() {
		c.mu.Lock()
		c.cond.Broadcast()
		c.mu.Unlock()
	})
	defer stop()

	c.mu.Lock()
	jumps := c.jumps
	for {
		if err := ctx.Err(); err != nil {
			c.mu.Unlock()
			return err
		}
		if !c.useSim {
			c.mu.Unlock()
			return c.sleepWall(ctx, t)
		}
		if c.jumps != jumps {
			c.mu.Unlock()
			return ErrTimeJumpedBack
		}
		if !c.now.Before(t) {
			c.mu.Unlock()
			return nil
		}
		c.cond.Wait()
	}
}

// sleepWall sleeps on the system clock until t. A switch to simulated time
// during the sleep is not observed; the sleep ends at t in system time.
func (c *Clock) sleepWall(ctx context.Context, t time.Time) error {
	d := time.Until(t)
	if d <= 0 {
		return nil
	}
	timer := time.NewTimer(d)
	defer timer.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-timer.C:
		return nil
	}
}

// Shutdown stops following use_sim_time and /clock and returns the context's
// rcl clock to the system time.
func (c *Clock) Shutdown() {
	if c.removeHook != nil {
		c.removeHook()
		c.removeHook = nil
	}
	c.switchMu.Lock()
	defer c.switchMu.Unlock()
	c.setUseSimLocked(false)
}

// setUseSimLocked switches between system and simulated time. switchMu must
// be held.
func (c *Clock) setUseSimLocked(enable bool) {
	c.mu.Lock()
	if c.useSim == enable {
		c.mu.Unlock()
		return
	}
	c.useSim = enable
	if enable {
		c.now = time.Unix(0, 0)
	}
	old := c.sub
	c.sub = nil
	// Wake sleepers so they re-evaluate on the new source.
	c.cond.Broadcast()
	c.mu.Unlock()

	if old != nil {
		old.Close() //nolint:errcheck
	}
	rclClock := c.node.Context().Clock()
	if !enable {
		if c.overriding {
			rclClock.DisableROSTimeOverride() //nolint:errcheck
			c.overriding = false
		}
		return
	}
	// A context whose clock is not ROS time (e.g. system time) cannot be
	// overridden; its timers stay on that clock.
	if err := rclClock.SetROSTimeOverride(0); err == nil {
		c.overriding = rclClock.EnableROSTimeOverride() == nil
	}
	sub, err := c.node.NewSubscription("/clock", rosgraph_msgs_msg.ClockTypeSupport,
		&rclgo.SubscriptionOptions{Qos: qos.NewClockProfile()}, c.onClock)
	if err != nil {
		// Without /clock, simulated time cannot advance; fall back to system
		// time as before.
		c.node.Logger().Error("rostime: cannot subscribe to /clock, using system time: ", err) //nolint:errcheck
		c.mu.Lock()
		c.useSim = false
		c.cond.Broadcast()
		c.mu.Unlock()
		if c.overriding {
			rclClock.DisableROSTimeOverride() //nolint:errcheck
			c.overriding = false
		}
		return
	}
	c.mu.Lock()
	c.sub = sub
	c.mu.Unlock()
}

func (c *Clock) onClock(sub *rclgo.Subscription) {
	msg := rosgraph_msgs_msg.NewClock()
	if _, err := sub.TakeMessage(msg); err != nil {
		return
	}
	t := time.Unix(int64(msg.Clock.Sec), int64(msg.Clock.Nanosec))
	c.mu.Lock()
	if !c.useSim || c.sub != sub {
		c.mu.Unlock()
		return
	}
	if t.Before(c.now) {
		c.jumps++
	}
	c.now = t
	overriding := c.overriding
	c.cond.Broadcast()
	c.mu.Unlock()
	if overriding {
		// rcl makes due timers ready and resets them on a backwards jump.
		c.node.Context().Clock().SetROSTimeOverride(time.Duration(t.UnixNano())) //nolint:errcheck
	}
}
