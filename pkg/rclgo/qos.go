/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo

// #include "rmw/rmw.h"
import "C"

import (
	"time"
)

const (
	DurationInfinite    = 9223372036*time.Second + 854775807*time.Nanosecond
	DurationUnspecified = time.Duration(0)
)

// -------------------------
// Policies  (mirrors rclcpp / RMW enums)
// -------------------------

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
)

type DurabilityPolicy int

const (
	DurabilitySystemDefault DurabilityPolicy = iota
	DurabilityTransientLocal
	DurabilityVolatile
	DurabilityUnknown
)

const DeadlineDefault = DurationUnspecified
const LifespanDefault = DurationUnspecified

type LivelinessPolicy int

const (
	LivelinessSystemDefault LivelinessPolicy = iota
	LivelinessAutomatic
	_
	LivelinessManualByTopic
	LivelinessUnknown
)

const LivelinessLeaseDurationDefault = DurationUnspecified

// -------------------------
// Profile
// -------------------------

// QosProfile mirrors rclcpp::QoS (high-level) mapped to rmw_qos_profile_t.
type QosProfile struct {
	History                      HistoryPolicy     `yaml:"history"`
	Depth                        int               `yaml:"depth"`
	Reliability                  ReliabilityPolicy `yaml:"reliability"`
	Durability                   DurabilityPolicy  `yaml:"durability"`
	Deadline                     time.Duration     `yaml:"deadline"`
	Lifespan                     time.Duration     `yaml:"lifespan"`
	Liveliness                   LivelinessPolicy  `yaml:"liveliness"`
	LivelinessLeaseDuration      time.Duration     `yaml:"liveliness_lease_duration"`
	AvoidRosNamespaceConventions bool              `yaml:"avoid_ros_namespace_conventions"`
}

// -------------------------
// Constructors (rclcpp-style, Go naming)
// -------------------------

// NewSystemDefaultQosProfile leaves all policies to the middleware.
func NewSystemDefaultQosProfile() QosProfile {
	return QosProfile{
		History:                      HistorySystemDefault,
		Depth:                        0,
		Reliability:                  ReliabilitySystemDefault,
		Durability:                   DurabilitySystemDefault,
		Deadline:                     DeadlineDefault,
		Lifespan:                     LifespanDefault,
		Liveliness:                   LivelinessSystemDefault,
		LivelinessLeaseDuration:      LivelinessLeaseDurationDefault,
		AvoidRosNamespaceConventions: false,
	}
}

// NewDefaultQosProfile matches rclcpp::QoS(10) default:
// KeepLast(10), Reliable, Volatile.
func NewDefaultQosProfile() QosProfile {
	return QosProfile{
		History:                      HistoryKeepLast,
		Depth:                        10,
		Reliability:                  ReliabilityReliable,
		Durability:                   DurabilityVolatile,
		Deadline:                     DeadlineDefault,
		Lifespan:                     LifespanDefault,
		Liveliness:                   LivelinessSystemDefault,
		LivelinessLeaseDuration:      LivelinessLeaseDurationDefault,
		AvoidRosNamespaceConventions: false,
	}
}

// NewDefaultServiceQosProfile is the common service endpoint profile in ROS 2:
// KeepLast(10), Reliable, Volatile.
func NewDefaultServiceQosProfile() QosProfile {
	return NewDefaultQosProfile()
}

// NewSensorDataQosProfile matches rclcpp::SensorDataQoS:
// KeepLast(5), BestEffort, Volatile.
func NewSensorDataQosProfile() QosProfile {
	return QosProfile{
		History:                      HistoryKeepLast,
		Depth:                        5,
		Reliability:                  ReliabilityBestEffort,
		Durability:                   DurabilityVolatile,
		Deadline:                     DeadlineDefault,
		Lifespan:                     LifespanDefault,
		Liveliness:                   LivelinessAutomatic,
		LivelinessLeaseDuration:      LivelinessLeaseDurationDefault,
		AvoidRosNamespaceConventions: false,
	}
}

// NewParametersQosProfile is a conservative default for parameter service calls:
// KeepLast(10), Reliable, Volatile.
func NewParametersQosProfile() QosProfile {
	return NewDefaultQosProfile()
}

// NewParameterEventsQosProfile matches rclcpp::ParameterEventsQoS:
// KeepAll, Reliable, TransientLocal (so late subscribers get the last events).
func NewParameterEventsQosProfile() QosProfile {
	return QosProfile{
		History:                      HistoryKeepAll,
		Depth:                        0, // ignored for KeepAll
		Reliability:                  ReliabilityReliable,
		Durability:                   DurabilityTransientLocal,
		Deadline:                     DeadlineDefault,
		Lifespan:                     LifespanDefault,
		Liveliness:                   LivelinessAutomatic,
		LivelinessLeaseDuration:      LivelinessLeaseDurationDefault,
		AvoidRosNamespaceConventions: false,
	}
}

// Convenience builders (optional)

// NewKeepLast builds a profile with KeepLast(depth), Reliable, Volatile.
func NewKeepLast(depth int) QosProfile {
	q := NewDefaultQosProfile()
	q.History = HistoryKeepLast
	q.Depth = depth
	return q
}

// NewKeepAll builds a profile with KeepAll, Reliable, Volatile.
func NewKeepAll() QosProfile {
	q := NewDefaultQosProfile()
	q.History = HistoryKeepAll
	q.Depth = 0
	return q
}

// WithReliability returns a modified copy with the given reliability.
func (p QosProfile) WithReliability(r ReliabilityPolicy) QosProfile {
	p.Reliability = r
	return p
}

// WithDurability returns a modified copy with the given durability.
func (p QosProfile) WithDurability(d DurabilityPolicy) QosProfile {
	p.Durability = d
	return p
}

// WithHistory returns a modified copy with the given history policy and depth.
func (p QosProfile) WithHistory(h HistoryPolicy, depth int) QosProfile {
	p.History = h
	p.Depth = depth
	return p
}

// -------------------------
// C interop
// -------------------------

func (p *QosProfile) asCStruct(dst *C.rmw_qos_profile_t) {
	dst.history = uint32(p.History)
	dst.depth = C.size_t(p.Depth)
	dst.reliability = uint32(p.Reliability)
	dst.durability = uint32(p.Durability)
	dst.deadline = C.rmw_time_t{nsec: C.uint64_t(p.Deadline)}
	dst.lifespan = C.rmw_time_t{nsec: C.uint64_t(p.Lifespan)}
	dst.liveliness = uint32(p.Liveliness)
	dst.liveliness_lease_duration = C.rmw_time_t{nsec: C.uint64_t(p.LivelinessLeaseDuration)}
	dst.avoid_ros_namespace_conventions = C.bool(p.AvoidRosNamespaceConventions)
}

func (p *QosProfile) fromCStruct(src *C.rmw_qos_profile_t) {
	p.History = HistoryPolicy(src.history)
	p.Depth = int(src.depth)
	p.Reliability = ReliabilityPolicy(src.reliability)
	p.Durability = DurabilityPolicy(src.durability)
	p.Deadline = time.Duration(src.deadline.sec)*time.Second + time.Duration(src.deadline.nsec)
	p.Lifespan = time.Duration(src.lifespan.sec)*time.Second + time.Duration(src.lifespan.nsec)
	p.Liveliness = LivelinessPolicy(src.liveliness)
	p.LivelinessLeaseDuration = time.Duration(src.liveliness_lease_duration.sec)*time.Second + time.Duration(src.liveliness_lease_duration.nsec)
	p.AvoidRosNamespaceConventions = bool(src.avoid_ros_namespace_conventions)
}
