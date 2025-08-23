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
