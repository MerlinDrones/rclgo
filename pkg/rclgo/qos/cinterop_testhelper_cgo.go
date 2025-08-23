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
