//go:build cgo

/*
This file is part of rclgo

Copyright © 2025 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

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
