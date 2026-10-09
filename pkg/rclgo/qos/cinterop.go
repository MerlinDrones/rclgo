/*
This file is part of rclgo

Copyright © 2025 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

// pkg/rclgo/qos/cinterop.go
package qos

/*
#include <rmw/rmw.h>
#include <stdbool.h>
#include <stdint.h>

// Cast enums on the C side to avoid cgo typedef mismatches across units.
static inline void rclgo_qos_set(
  rmw_qos_profile_t* dst,
  uint32_t history,
  size_t depth,
  uint32_t reliability,
  uint32_t durability,
  uint64_t deadline_sec, uint64_t deadline_nsec,
  uint64_t lifespan_sec, uint64_t lifespan_nsec,
  uint32_t liveliness,
  uint64_t lease_sec, uint64_t lease_nsec,
  _Bool avoid_ns
) {
  dst->history = (rmw_qos_history_policy_t)history;
  dst->depth = depth;
  dst->reliability = (rmw_qos_reliability_policy_t)reliability;
  dst->durability = (rmw_qos_durability_policy_t)durability;
  dst->deadline.sec = deadline_sec;
  dst->deadline.nsec = deadline_nsec;
  dst->lifespan.sec = lifespan_sec;
  dst->lifespan.nsec = lifespan_nsec;
  dst->liveliness = (rmw_qos_liveliness_policy_t)liveliness;
  dst->liveliness_lease_duration.sec = lease_sec;
  dst->liveliness_lease_duration.nsec = lease_nsec;
  dst->avoid_ros_namespace_conventions = avoid_ns;
}
*/
import "C"

// AsCStruct writes p into a C rmw_qos_profile_t.
func (p Profile) AsCStruct(dst *C.rmw_qos_profile_t) {
	if dst == nil {
		return
	}
	deadlineSec, deadlineNsec := DurationToRMW(p.Deadline)
	lifespanSec, lifespanNsec := DurationToRMW(p.Lifespan)
	leaseSec, leaseNsec := DurationToRMW(p.LivelinessLeaseDuration)
	C.rclgo_qos_set(
		dst,
		C.uint32_t(p.History),
		C.size_t(p.Depth),
		C.uint32_t(p.Reliability),
		C.uint32_t(p.Durability),
		C.uint64_t(deadlineSec), C.uint64_t(deadlineNsec),
		C.uint64_t(lifespanSec), C.uint64_t(lifespanNsec),
		C.uint32_t(p.Liveliness),
		C.uint64_t(leaseSec), C.uint64_t(leaseNsec),
		C.bool(p.AvoidROSNamespaceConventions),
	)
}

// FromCStruct reads a C rmw_qos_profile_t into p.
func (p *Profile) FromCStruct(src *C.rmw_qos_profile_t) {
	if p == nil || src == nil {
		return
	}
	*p = Profile{
		History:                      HistoryPolicy(src.history),
		Depth:                        int(src.depth),
		Reliability:                  ReliabilityPolicy(src.reliability),
		Durability:                   DurabilityPolicy(src.durability),
		Deadline:                     DurationFromRMW(uint64(src.deadline.sec), uint64(src.deadline.nsec)),
		Lifespan:                     DurationFromRMW(uint64(src.lifespan.sec), uint64(src.lifespan.nsec)),
		Liveliness:                   LivelinessPolicy(src.liveliness),
		LivelinessLeaseDuration:      DurationFromRMW(uint64(src.liveliness_lease_duration.sec), uint64(src.liveliness_lease_duration.nsec)),
		AvoidROSNamespaceConventions: bool(src.avoid_ros_namespace_conventions),
	}
}
