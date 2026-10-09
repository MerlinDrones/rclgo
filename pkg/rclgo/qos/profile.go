/*
This file is part of rclgo

Copyright © 2025 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package qos

import (
	"math"
	"time"
)

// NOTE: enum values mirror your current rclgo values so C-casts still match.

type HistoryPolicy int

const (
	HistorySystemDefault HistoryPolicy = iota
	HistoryKeepLast
	HistoryKeepAll
	HistoryUnknown
)

type ReliabilityPolicy int

const (
	ReliabilitySystemDefault ReliabilityPolicy = iota
	ReliabilityReliable
	ReliabilityBestEffort
	ReliabilityUnknown
	ReliabilityBestAvailable
)

type DurabilityPolicy int

const (
	DurabilitySystemDefault DurabilityPolicy = iota
	DurabilityTransientLocal
	DurabilityVolatile
	DurabilityUnknown
	DurabilityBestAvailable
)

type LivelinessPolicy int

const (
	LivelinessSystemDefault LivelinessPolicy = iota
	LivelinessAutomatic
	_
	LivelinessManualByTopic
	LivelinessUnknown
	LivelinessBestAvailable
)

// Duration defaults match your current constants.
const (
	DurationUnspecified            = time.Duration(0)
	DeadlineDefault                = DurationUnspecified
	LifespanDefault                = DurationUnspecified
	LivelinessLeaseDurationDefault = DurationUnspecified

	// DurationInfinite is RMW_DURATION_INFINITE, the largest duration.
	DurationInfinite = time.Duration(math.MaxInt64)
)

// DurationToRMW splits d into the seconds and nanoseconds of an rmw_time_t
// (nanoseconds below one second). Negative durations are treated as zero.
func DurationToRMW(d time.Duration) (sec, nsec uint64) {
	if d <= 0 {
		return 0, 0
	}
	return uint64(d / time.Second), uint64(d % time.Second)
}

// DurationFromRMW joins the seconds and nanoseconds of an rmw_time_t.
// Durations beyond the range of time.Duration, such as
// RMW_DURATION_INFINITE, saturate to DurationInfinite.
func DurationFromRMW(sec, nsec uint64) time.Duration {
	const maxSec = uint64(math.MaxInt64 / int64(time.Second))
	if sec > maxSec {
		return DurationInfinite
	}
	total := sec * uint64(time.Second)
	if nsec > math.MaxInt64-total {
		return DurationInfinite
	}
	return time.Duration(total + nsec)
}

// Profile is the canonical QoS container used across rclgo.
type Profile struct {
	History                      HistoryPolicy     `yaml:"history"`
	Depth                        int               `yaml:"depth"`
	Reliability                  ReliabilityPolicy `yaml:"reliability"`
	Durability                   DurabilityPolicy  `yaml:"durability"`
	Deadline                     time.Duration     `yaml:"deadline"`
	Lifespan                     time.Duration     `yaml:"lifespan"`
	Liveliness                   LivelinessPolicy  `yaml:"liveliness"`
	LivelinessLeaseDuration      time.Duration     `yaml:"liveliness_lease_duration"`
	AvoidROSNamespaceConventions bool              `yaml:"avoid_ros_namespace_conventions"`
}

// Constructors (mirroring your existing helpers)
func NewSystemDefault() Profile {
	return Profile{
		History:                      HistorySystemDefault,
		Depth:                        0,
		Reliability:                  ReliabilitySystemDefault,
		Durability:                   DurabilitySystemDefault,
		Deadline:                     DeadlineDefault,
		Lifespan:                     LifespanDefault,
		Liveliness:                   LivelinessSystemDefault,
		LivelinessLeaseDuration:      LivelinessLeaseDurationDefault,
		AvoidROSNamespaceConventions: false,
	}
}

func NewDefault() Profile { // rclcpp::QoS(10): KeepLast(10), Reliable, Volatile
	return Profile{
		History:     HistoryKeepLast,
		Depth:       10,
		Reliability: ReliabilityReliable,
		Durability:  DurabilityVolatile,
		Deadline:    DeadlineDefault,
		Lifespan:    LifespanDefault,
		Liveliness:  LivelinessSystemDefault,
	}
}

func NewKeepLast(depth int) Profile {
	q := NewDefault()
	q.History = HistoryKeepLast
	q.Depth = depth
	return q
}

func NewKeepAll() Profile {
	q := NewDefault()
	q.History = HistoryKeepAll
	q.Depth = 0
	return q
}

func (p Profile) WithReliability(r ReliabilityPolicy) Profile { p.Reliability = r; return p }
func (p Profile) WithDurability(d DurabilityPolicy) Profile   { p.Durability = d; return p }
func (p Profile) WithHistory(h HistoryPolicy, depth int) Profile {
	p.History = h
	p.Depth = depth
	return p
}
