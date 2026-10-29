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
	"time"
)

// Rate runs a loop at a fixed frequency on a Clock, following simulated time
// when it is active.
type Rate struct {
	clock  *Clock
	period time.Duration
	last   time.Time
}

// Sleep sleeps until the next period, as rclcpp's Rate::sleep does: if the
// loop overran by more than a period it resets instead of returning
// immediately until it has caught up, and if time jumped backwards it starts
// counting periods again from the new time.
func (r *Rate) Sleep(ctx context.Context) error {
	now := r.clock.Now()
	next := r.last.Add(r.period)
	if now.Before(r.last) {
		// Time jumped backwards since the last sleep.
		next = now.Add(r.period)
	}
	if !next.After(now) {
		// Overran the period.
		if now.After(next.Add(r.period)) {
			r.last = now
		} else {
			r.last = next
		}
		return nil
	}
	err := r.clock.SleepUntil(ctx, next)
	if errors.Is(err, ErrTimeJumpedBack) {
		r.last = r.clock.Now()
		return nil
	}
	if err != nil {
		return err
	}
	r.last = next
	return nil
}
