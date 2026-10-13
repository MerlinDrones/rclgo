/*
This file is part of rclgo

Copyright © 2025 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package utils

import (
	"github.com/merlindrones/rclgo/pkg/msgs/builtin_interfaces/msg"

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
	// One reading, so that seconds and nanoseconds belong to the same instant.
	now := time.Now()
	return builtin_interfaces_msg.Time{
		Sec:     int32(now.Unix()),
		Nanosec: uint32(now.Nanosecond()),
	}
}
