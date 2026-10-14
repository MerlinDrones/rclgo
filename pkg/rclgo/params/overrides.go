package params

/*
#cgo CFLAGS: -I/opt/ros/jazzy/include/rcl -I/opt/ros/jazzy/include/rcl_yaml_param_parser -I/opt/ros/jazzy/include/rcutils -I/opt/ros/jazzy/include/rmw
#cgo CFLAGS: -I/opt/ros/jazzy/include/rosidl_runtime_c -I/opt/ros/jazzy/include/rosidl_typesupport_interface -I/opt/ros/jazzy/include/rosidl_dynamic_typesupport
#cgo CFLAGS: -I/opt/ros/jazzy/include/type_description_interfaces -I/opt/ros/jazzy/include/service_msgs -I/opt/ros/jazzy/include/builtin_interfaces
#cgo LDFLAGS: -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib -lrcl_yaml_param_parser
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
	"regexp"
	"strings"
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

	// Like rclcpp, apply every section whose node name matches this node, in
	// the order rcl stores them, so that a later section overrides an earlier
	// one: -p values are in the "/**" section, and each --params-file adds its
	// own sections.
	names := matchNames(m, nodeName)
	type override struct {
		name string
		val  Value
	}
	var merged []override
	index := map[string]int{}
	for i := 0; i < int(cparams.num_nodes); i++ {
		sectionPtr := *(**C.char)(unsafe.Pointer(uintptr(unsafe.Pointer(cparams.node_names)) + uintptr(i)*unsafe.Sizeof((*C.char)(nil))))
		if !sectionMatches(C.GoString(sectionPtr), names) {
			continue
		}
		nodeParams := (*C.rcl_node_params_t)(unsafe.Pointer(uintptr(unsafe.Pointer(cparams.params)) + uintptr(i)*unsafe.Sizeof(C.rcl_node_params_t{})))
		for j := 0; j < int(nodeParams.num_params); j++ {
			paramNamePtr := *(**C.char)(unsafe.Pointer(uintptr(unsafe.Pointer(nodeParams.parameter_names)) + uintptr(j)*unsafe.Sizeof((*C.char)(nil))))
			paramName := C.GoString(paramNamePtr)
			variant := (*C.rcl_variant_t)(unsafe.Pointer(uintptr(unsafe.Pointer(nodeParams.parameter_values)) + uintptr(j)*unsafe.Sizeof(C.rcl_variant_t{})))
			val, ok := valueFromVariant(variant)
			if !ok {
				return fmt.Errorf("unsupported parameter type for %s", paramName)
			}
			if k, seen := index[paramName]; seen {
				merged[k].val = val
			} else {
				index[paramName] = len(merged)
				merged = append(merged, override{paramName, val})
			}
		}
	}

	for _, o := range merged {
		// A later Declare in code attaches its descriptor to this value.
		if err := m.applyOverride(o.name, o.val, Descriptor{}); err != nil {
			return fmt.Errorf("failed to apply override for %s: %w", o.name, err)
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

// matchNames returns the fully qualified names a parameter section is matched
// against: the manager's node, and nodeName in the node's namespace (a
// nodeName starting with "/" is taken as fully qualified). As in rcl, a
// section names a node by its fully qualified name, so "/name" or "name"
// applies only to a node in the root namespace.
func matchNames(m *Manager, nodeName string) []string {
	var names []string
	ns := "/"
	if m.n != nil {
		names = append(names, m.n.FullyQualifiedName())
		ns = m.n.Namespace()
	}
	switch {
	case nodeName == "":
	case strings.HasPrefix(nodeName, "/"):
		names = append(names, nodeName)
	default:
		names = append(names, strings.TrimSuffix(ns, "/")+"/"+nodeName)
	}
	return names
}

// sectionMatches reports whether a parameter section applies to one of names,
// with rclcpp's rules: a missing leading slash is added, "/*" matches one name
// token and "/**" any number of them.
func sectionMatches(section string, names []string) bool {
	if !strings.HasPrefix(section, "/") {
		section = "/" + section
	}
	pattern := regexp.QuoteMeta(section)
	pattern = strings.ReplaceAll(pattern, `/\*\*`, `(/\w+)*`)
	pattern = strings.ReplaceAll(pattern, `/\*`, `(/\w+)`)
	re, err := regexp.Compile("^" + pattern + "$")
	if err != nil {
		return false
	}
	for _, n := range names {
		if re.MatchString(n) {
			return true
		}
	}
	return false
}
