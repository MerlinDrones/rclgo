package utils

import (
	"math"
	"time"

	builtin_interfaces_msg "github.com/merlindrones/rclgo/pkg/msgs/builtin_interfaces/msg"
)

// ROSTime converts t to a builtin_interfaces/msg/Time: seconds since the epoch
// and the nanoseconds within that second, which are never negative (a time
// before the epoch has negative seconds). builtin_interfaces/msg/Time has
// 32-bit seconds, so times outside 1901-12-13 to 2038-01-19 (UTC) saturate at
// the nearest representable time instead of wrapping around (rclcpp throws).
func ROSTime(t time.Time) builtin_interfaces_msg.Time {
	sec := t.Unix()
	switch {
	case sec > math.MaxInt32:
		return builtin_interfaces_msg.Time{Sec: math.MaxInt32, Nanosec: 999999999}
	case sec < math.MinInt32:
		return builtin_interfaces_msg.Time{Sec: math.MinInt32}
	}
	return builtin_interfaces_msg.Time{Sec: int32(sec), Nanosec: uint32(t.Nanosecond())}
}

// ROSTimeNow is ROSTime(time.Now()): one reading of the clock, so that seconds
// and nanoseconds belong to the same instant.
func ROSTimeNow() builtin_interfaces_msg.Time {
	return ROSTime(time.Now())
}
