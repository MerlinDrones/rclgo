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
  uint64_t deadline,
  uint64_t lifespan,
  uint32_t liveliness,
  uint64_t liveliness_lease_duration,
  _Bool avoid_ns
) {
  dst->history = (rmw_qos_history_policy_t)history;
  dst->depth = depth;
  dst->reliability = (rmw_qos_reliability_policy_t)reliability;
  dst->durability = (rmw_qos_durability_policy_t)durability;
  dst->deadline.nsec = deadline;
  dst->lifespan.nsec = lifespan;
  dst->liveliness = (rmw_qos_liveliness_policy_t)liveliness;
  dst->liveliness_lease_duration.nsec = liveliness_lease_duration;
  dst->avoid_ros_namespace_conventions = avoid_ns;
}
*/
import "C"
import "time"

// AsCStruct writes p into a C rmw_qos_profile_t.
func (p Profile) AsCStruct(dst *C.rmw_qos_profile_t) {
	if dst == nil {
		return
	}
	C.rclgo_qos_set(
		dst,
		C.uint32_t(p.History),
		C.size_t(p.Depth),
		C.uint32_t(p.Reliability),
		C.uint32_t(p.Durability),
		C.uint64_t(p.Deadline),
		C.uint64_t(p.Lifespan),
		C.uint32_t(p.Liveliness),
		C.uint64_t(p.LivelinessLeaseDuration),
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
		Deadline:                     time.Duration(src.deadline.nsec),
		Lifespan:                     time.Duration(src.lifespan.nsec),
		Liveliness:                   LivelinessPolicy(src.liveliness),
		LivelinessLeaseDuration:      time.Duration(src.liveliness_lease_duration.nsec),
		AvoidROSNamespaceConventions: bool(src.avoid_ros_namespace_conventions),
	}
}
