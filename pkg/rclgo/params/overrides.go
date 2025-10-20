package params

/*
#cgo LDFLAGS: -lrcl_yaml_param_parser
#include <rcl/arguments.h>
#include <rcl_yaml_param_parser/parser.h>

// Helper to free rcl_params_t
void rclgo_params_fini(rcl_params_t * params) {
	if (params) {
		rcl_yaml_node_struct_fini(params);
	}
}
*/
import "C"
import (
	"fmt"
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
)

// ApplyOverrides extracts parameter overrides from ROS args (e.g., -p param:=value)
// and applies them to the params Manager by declaring parameters if they don't exist
// or setting them if they do.
//
// This enables usage like:
//
//	ros2 run my_package my_node --ros-args -p my_param:=42
//
// Usage:
//
//	mgr, _ := params.NewManager(node)
//	_ = params.ApplyOverrides(mgr, node.Name(), rclArgs)
func ApplyOverrides(m *Manager, nodeName string, args *rclgo.Args) error {
	if args == nil {
		return nil
	}

	// Get parameter overrides from RCL arguments using the safe bridge method.
	// This avoids CGO pointer safety issues by calling rcl_arguments_get_param_overrides
	// directly in the rclgo package where Args is defined.
	cparamsPtr, err := args.GetParamOverrides()
	if err != nil {
		return fmt.Errorf("failed to get param overrides: %w", err)
	}
	if cparamsPtr == nil {
		// No overrides
		return nil
	}
	cparams := (*C.rcl_params_t)(cparamsPtr)
	defer C.rclgo_params_fini(cparams)

	// Find node-specific parameters
	var nodeParams *C.rcl_node_params_t
	for i := 0; i < int(cparams.num_nodes); i++ {
		// Access node name from array
		nodeNamePtr := *(**C.char)(unsafe.Pointer(uintptr(unsafe.Pointer(cparams.node_names)) + uintptr(i)*unsafe.Sizeof((*C.char)(nil))))
		nodeName_c := C.GoString(nodeNamePtr)
		// Match if it's the wildcard node "/**" OR matches our specific node name
		if nodeName_c == "/**" || nodeName_c == nodeName || nodeName_c == "/"+nodeName {
			// Access node params from array
			nodeParams = (*C.rcl_node_params_t)(unsafe.Pointer(uintptr(unsafe.Pointer(cparams.params)) + uintptr(i)*unsafe.Sizeof(C.rcl_node_params_t{})))
			break
		}
	}
	if nodeParams == nil {
		// No overrides for this node
		return nil
	}

	// Apply each parameter override
	for i := 0; i < int(nodeParams.num_params); i++ {
		// Access parameter name from array
		paramNamePtr := *(**C.char)(unsafe.Pointer(uintptr(unsafe.Pointer(nodeParams.parameter_names)) + uintptr(i)*unsafe.Sizeof((*C.char)(nil))))
		paramName := C.GoString(paramNamePtr)
		// Access parameter value from array
		variant := (*C.rcl_variant_t)(unsafe.Pointer(uintptr(unsafe.Pointer(nodeParams.parameter_values)) + uintptr(i)*unsafe.Sizeof(C.rcl_variant_t{})))

		val, ok := valueFromVariant(variant)
		if !ok {
			return fmt.Errorf("unsupported parameter type for %s", paramName)
		}

		// If parameter already exists, set it. Otherwise declare it.
		if _, exists := m.Get(paramName); exists {
			if _, err := m.SetValue(paramName, val); err != nil {
				return fmt.Errorf("failed to set override for %s: %w", paramName, err)
			}
		} else {
			if _, err := m.Declare(paramName, val, Descriptor{}); err != nil {
				return fmt.Errorf("failed to declare override for %s: %w", paramName, err)
			}
		}
	}

	return nil
}

// valueFromVariant converts a C rcl_variant_t to a Go params.Value
func valueFromVariant(v *C.rcl_variant_t) (Value, bool) {
	if v.bool_value != nil {
		return Value{Kind: KindBool, Bool: bool(*v.bool_value)}, true
	}
	if v.integer_value != nil {
		return Value{Kind: KindInt64, Int64: int64(*v.integer_value)}, true
	}
	if v.double_value != nil {
		return Value{Kind: KindDouble, Double: float64(*v.double_value)}, true
	}
	if v.string_value != nil {
		return Value{Kind: KindString, Str: C.GoString(v.string_value)}, true
	}
	if v.bool_array_value != nil {
		arr := v.bool_array_value
		out := make([]bool, arr.size)
		for i := 0; i < int(arr.size); i++ {
			valPtr := (*C.bool)(unsafe.Pointer(uintptr(unsafe.Pointer(arr.values)) + uintptr(i)*unsafe.Sizeof(C.bool(false))))
			out[i] = bool(*valPtr)
		}
		return Value{Kind: KindBoolArray, Bools: out}, true
	}
	if v.integer_array_value != nil {
		arr := v.integer_array_value
		out := make([]int64, arr.size)
		for i := 0; i < int(arr.size); i++ {
			valPtr := (*C.int64_t)(unsafe.Pointer(uintptr(unsafe.Pointer(arr.values)) + uintptr(i)*unsafe.Sizeof(C.int64_t(0))))
			out[i] = int64(*valPtr)
		}
		return Value{Kind: KindInt64Array, Int64s: out}, true
	}
	if v.double_array_value != nil {
		arr := v.double_array_value
		out := make([]float64, arr.size)
		for i := 0; i < int(arr.size); i++ {
			valPtr := (*C.double)(unsafe.Pointer(uintptr(unsafe.Pointer(arr.values)) + uintptr(i)*unsafe.Sizeof(C.double(0))))
			out[i] = float64(*valPtr)
		}
		return Value{Kind: KindDoubleArray, Doubles: out}, true
	}
	if v.string_array_value != nil {
		arr := v.string_array_value
		out := make([]string, arr.size)
		for i := 0; i < int(arr.size); i++ {
			strPtr := *(**C.char)(unsafe.Pointer(uintptr(unsafe.Pointer(arr.data)) + uintptr(i)*unsafe.Sizeof((*C.char)(nil))))
			out[i] = C.GoString(strPtr)
		}
		return Value{Kind: KindStringArray, Strs: out}, true
	}
	// Byte arrays not commonly used for ROS params, skip for now
	return Value{}, false
}
