package rostime

import (
	"context"
	"sync"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/utils"

	builtin_interfaces_msg "github.com/merlindrones/rclgo/internal/msgs/builtin_interfaces/msg"
	rosgraph_msgs_msg "github.com/merlindrones/rclgo/internal/msgs/rosgraph_msgs/msg"
)

type Clock struct {
	mu     sync.RWMutex
	useSim bool
	now    time.Time
	cond   *sync.Cond

	node   *rclgo.Node
	sub    *rclgo.Subscription
	cancel context.CancelFunc
}

// NewClock wires use_sim_time and (when enabled) subscribes to /clock.
func NewClock(_ context.Context, node *rclgo.Node, pm *params.Manager) (*Clock, error) {
	c := &Clock{
		node: node,
		now:  time.Now(),
	}
	c.cond = sync.NewCond(&c.mu)

	// Initialize from parameter (default false in ROS 2).
	// Note: params.Manager uses "use_sim_time" (no leading slash).
	useSim := false
	if pm != nil {
		// Let params/events use ROS time when sim time is active.
		pm.SetNowProvider(c.NowMsg)

		if p, ok := pm.Get("use_sim_time"); ok && p.Value.Kind == params.KindBool && p.Value.Bool {
			useSim = true
		}

		// (Optional) react to runtime changes — uncomment when exposing OnSet.
		// pm.OnSet(func(name string, p params.Parameter) error {
		// 	if name == "use_sim_time" && p.Value.Kind == params.KindBool {
		// 		c.setUseSim(p.Value.Bool)
		// 	}
		// 	return nil
		// })
	}

	c.setUseSim(useSim)
	return c, nil
}

// IsSimTime reports whether the clock is currently using simulated time.
func (c *Clock) IsSimTime() bool {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.useSim
}

// Now returns the current time from the active source (system or sim).
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

// SleepUntil sleeps until the given absolute time according to the active clock.
func (c *Clock) SleepUntil(ctx context.Context, t time.Time) error {
	c.mu.Lock()
	if !c.useSim {
		// Do NOT hold c.mu while sleeping on wall clock; avoid double-unlock.
		c.mu.Unlock()
		timer := time.NewTimer(time.Until(t))
		defer timer.Stop()
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-timer.C:
			return nil
		}
	}

	// Sim time: wait until c.now >= t or context cancels.
	for c.useSim && c.now.Before(t) {
		// Wait releases c.mu and re-acquires it on wake.
		c.cond.Wait()
		// Check context after each wake-up to allow cancellation while waiting.
		select {
		case <-ctx.Done():
			c.mu.Unlock()
			return ctx.Err()
		default:
		}
	}
	c.mu.Unlock()
	return nil
}

func (c *Clock) Shutdown() {
	c.setUseSim(false)
}

func (c *Clock) setUseSim(enable bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.useSim == enable {
		return
	}
	c.useSim = enable

	// Tear down any previous subscription/cancel
	if c.cancel != nil {
		c.cancel()
		c.cancel = nil
	}
	// If Subscription exposes Close/Fini, call it here. We just clear the ref.
	c.sub = nil

	if !enable {
		return
	}

	ctx, cancel := context.WithCancel(context.Background())
	c.cancel = cancel

	sub, err := nodeCreateClockSub(c.node, func(msg *rosgraph_msgs_msg.Clock) {
		c.mu.Lock()
		c.now = time.Unix(int64(msg.Clock.Sec), int64(msg.Clock.Nanosec))
		c.mu.Unlock()
		c.cond.Broadcast()
	})
	if err != nil {
		// Fall back to system time if subscription fails.
		c.useSim = false
		if c.cancel != nil {
			c.cancel()
			c.cancel = nil
		}
		return
	}
	c.sub = sub

	_ = ctx
}

// nodeCreateClockSub adapts to rclgo's NewSubscription signature.
func nodeCreateClockSub(node *rclgo.Node, cb func(*rosgraph_msgs_msg.Clock)) (*rclgo.Subscription, error) {
	opts := &rclgo.SubscriptionOptions{
		Qos: qos.NewClockProfile(),
	}
	callback := rclgo.SubscriptionCallback(func(sub *rclgo.Subscription) {
		// Take one rosgraph_msgs/Clock message and forward it.
		msg := rosgraph_msgs_msg.NewClock()
		if _, err := sub.TakeMessage(msg); err != nil {
			sub.Node().Logger().Debug(err)
			return
		}
		cb(msg)
	})
	return node.NewSubscription(
		"/clock",
		rosgraph_msgs_msg.ClockTypeSupport,
		opts,
		callback,
	)
}
