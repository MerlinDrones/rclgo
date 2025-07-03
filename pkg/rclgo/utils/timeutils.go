package utils

import (
	builtin_interfaces_msg "github.com/merlindrones/rclgo/internal/msgs/builtin_interfaces/msg"
	"time"
)

// GoTimeToROSTime converts a Go time.Time to ROS2 builtin_interfaces/msg/Time
func ROSTime(t time.Time) builtin_interfaces_msg.Time {
	return builtin_interfaces_msg.Time{
		Sec:     int32(t.Unix()),
		Nanosec: uint32(t.Nanosecond()),
	}
}

func ROSTimeNow() builtin_interfaces_msg.Time {
	return builtin_interfaces_msg.Time{
		Sec:     int32(time.Now().Unix()),
		Nanosec: uint32(time.Now().Nanosecond()),
	}
}
