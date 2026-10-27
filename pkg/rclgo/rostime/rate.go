/*
This file is part of rclgo

Copyright © 2025 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rostime

import (
	"context"
	"time"
)

type Rate struct {
	clock  *Clock
	period time.Duration
	last   time.Time
}

func (r *Rate) Sleep(ctx context.Context) error {
	next := r.last.Add(r.period)
	if err := r.clock.SleepUntil(ctx, next); err != nil {
		return err
	}
	r.last = next
	return nil
}
