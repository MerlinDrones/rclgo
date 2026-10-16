package qos

/*
#cgo LDFLAGS: -lrmw_implementation
#include <rmw/qos_profiles.h>
*/
import "C"

import (
	"fmt"
	"unsafe"
)

// Compatibility is rmw's verdict on whether a publisher and a subscription
// with given QoS profiles can communicate.
type Compatibility int

const (
	// CompatibilityOK: they communicate.
	CompatibilityOK Compatibility = iota
	// CompatibilityWarning: they may not, depending on what system default,
	// unknown or best-available policies resolve to.
	CompatibilityWarning
	// CompatibilityError: they do not communicate.
	CompatibilityError
)

func (c Compatibility) String() string {
	switch c {
	case CompatibilityOK:
		return "ok"
	case CompatibilityWarning:
		return "warning"
	case CompatibilityError:
		return "error"
	}
	return fmt.Sprintf("Compatibility(%d)", int(c))
}

// CheckCompatible reports whether a publisher with profile pub and a
// subscription with profile sub can communicate, with the rules of the rmw
// implementation in use (rmw_qos_profile_check_compatible; DDS
// implementations check reliability, durability, deadline and liveliness,
// rmw_zenoh_cpp reports everything compatible). reason explains a warning
// or an error.
func CheckCompatible(pub, sub Profile) (c Compatibility, reason string, err error) {
	var cpub, csub C.rmw_qos_profile_t
	pub.AsCStruct(&cpub)
	sub.AsCStruct(&csub)
	var compat C.rmw_qos_compatibility_type_t
	buf := make([]byte, 2048)
	rc := C.rmw_qos_profile_check_compatible(cpub, csub, &compat, (*C.char)(unsafe.Pointer(&buf[0])), C.size_t(len(buf)))
	if rc != C.RMW_RET_OK {
		return CompatibilityError, "", fmt.Errorf("rmw_qos_profile_check_compatible failed: %d", int(rc))
	}
	return Compatibility(compat), C.GoString((*C.char)(unsafe.Pointer(&buf[0]))), nil
}
