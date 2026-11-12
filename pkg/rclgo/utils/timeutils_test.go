/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package utils

import (
	"math"
	"testing"
	"time"

	builtin_interfaces_msg "github.com/merlindrones/rclgo/pkg/msgs/builtin_interfaces/msg"
)

func TestROSTime(t *testing.T) {
	for name, tc := range map[string]struct {
		in   time.Time
		want builtin_interfaces_msg.Time
	}{
		"epoch":              {time.Unix(0, 0), builtin_interfaces_msg.Time{}},
		"with nanoseconds":   {time.Unix(1700000000, 123456789), builtin_interfaces_msg.Time{Sec: 1700000000, Nanosec: 123456789}},
		"half second before": {time.Unix(0, -500000000), builtin_interfaces_msg.Time{Sec: -1, Nanosec: 500000000}},
		"last second of int32": {time.Unix(math.MaxInt32, 999999999),
			builtin_interfaces_msg.Time{Sec: math.MaxInt32, Nanosec: 999999999}},
		"after 2038":  {time.Unix(math.MaxInt32+1, 5), builtin_interfaces_msg.Time{Sec: math.MaxInt32, Nanosec: 999999999}},
		"far future":  {time.Date(3000, 1, 1, 0, 0, 0, 0, time.UTC), builtin_interfaces_msg.Time{Sec: math.MaxInt32, Nanosec: 999999999}},
		"before 1901": {time.Unix(math.MinInt32-1, 7), builtin_interfaces_msg.Time{Sec: math.MinInt32}},
	} {
		if got := ROSTime(tc.in); got != tc.want {
			t.Errorf("%s: ROSTime(%v) = %+v, want %+v", name, tc.in, got, tc.want)
		}
	}
}

func TestROSTimeNow(t *testing.T) {
	before := time.Now()
	got := ROSTimeNow()
	after := time.Now()
	ts := time.Unix(int64(got.Sec), int64(got.Nanosec))
	if ts.Before(before.Truncate(time.Nanosecond)) || ts.After(after) {
		t.Errorf("ROSTimeNow() = %v, not between %v and %v", ts, before, after)
	}
	if got.Nanosec >= 1e9 {
		t.Errorf("nanoseconds %d not below one second", got.Nanosec)
	}
}
