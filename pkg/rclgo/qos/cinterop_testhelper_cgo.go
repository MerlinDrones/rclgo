//go:build cgo

package qos

/*
#include <rmw/rmw.h>
*/
import "C"

// roundTripProfileForTest uses the real cgo functions to round-trip a Profile.
func roundTripProfileForTest(p Profile) Profile {
	var c C.rmw_qos_profile_t
	p.AsCStruct(&c)
	var out Profile
	out.FromCStruct(&c)
	return out
}

// rmwDurationsForTest writes p into a C profile whose duration fields start
// out holding junk in their seconds parts, and returns the {sec, nsec} pairs
// of deadline, lifespan and liveliness lease duration.
func rmwDurationsForTest(p Profile) (sec, nsec [3]uint64) {
	var c C.rmw_qos_profile_t
	c.deadline.sec, c.lifespan.sec, c.liveliness_lease_duration.sec = 7, 7, 7
	p.AsCStruct(&c)
	for i, d := range []C.rmw_time_t{c.deadline, c.lifespan, c.liveliness_lease_duration} {
		sec[i], nsec[i] = uint64(d.sec), uint64(d.nsec)
	}
	return sec, nsec
}

// profileFromRMWDurationsForTest reads a C profile whose deadline, lifespan
// and liveliness lease duration are all {sec, nsec}.
func profileFromRMWDurationsForTest(sec, nsec uint64) Profile {
	var c C.rmw_qos_profile_t
	for _, d := range []*C.rmw_time_t{&c.deadline, &c.lifespan, &c.liveliness_lease_duration} {
		d.sec, d.nsec = C.uint64_t(sec), C.uint64_t(nsec)
	}
	var p Profile
	p.FromCStruct(&c)
	return p
}
