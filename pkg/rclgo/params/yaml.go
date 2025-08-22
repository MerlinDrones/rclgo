package params

import (
	"fmt"
	"os"
	"path/filepath"

	"gopkg.in/yaml.v3"
)

// LoadYAML loads parameters from a ROS 2–style YAML file and declares them.
//
// Accepted shapes:
//
//	node_name:
//	  ros__parameters:
//	    camera.fps: 30
//	ros__parameters:
//	  camera.fps: 30
//
// It infers Value kinds from YAML scalar/sequence types.
//
// Typical use:
//
//	_ = params.LoadYAML(mgr, "param_demo", "/path/to/params.yaml")
func LoadYAML(m *Manager, nodeName string, path string) error {
	data, err := os.ReadFile(path)
	if err != nil {
		return fmt.Errorf("read %s: %w", path, err)
	}
	var root map[string]interface{}
	if err := yaml.Unmarshal(data, &root); err != nil {
		return fmt.Errorf("yaml unmarshal %s: %w", path, err)
	}

	// Find ros__parameters block
	var paramsMap map[string]interface{}

	// Prefer node-specific block
	if v, ok := root[nodeName]; ok {
		if m1, ok := v.(map[string]interface{}); ok {
			if rp, ok := m1["ros__parameters"]; ok {
				paramsMap, _ = rp.(map[string]interface{})
			}
		}
	}
	// Fallback to top-level ros__parameters
	if paramsMap == nil {
		if v, ok := root["ros__parameters"]; ok {
			paramsMap, _ = v.(map[string]interface{})
		}
	}
	if paramsMap == nil {
		return fmt.Errorf("no ros__parameters block found for node %q in %s", nodeName, filepath.Base(path))
	}

	for name, raw := range paramsMap {
		val, ok := valueFromYAML(raw)
		if !ok {
			return fmt.Errorf("unsupported YAML type for %s", name)
		}
		// Declare with empty descriptor (you can set constraints in code later)
		if _, err := m.Declare(name, val, Descriptor{}); err != nil {
			return fmt.Errorf("declare %s: %w", name, err)
		}
	}
	return nil
}

func valueFromYAML(v interface{}) (Value, bool) {
	switch x := v.(type) {
	case bool:
		return Value{Kind: KindBool, Bool: x}, true
	case int:
		return Value{Kind: KindInt64, Int64: int64(x)}, true
	case int64:
		return Value{Kind: KindInt64, Int64: x}, true
	case float32:
		return Value{Kind: KindDouble, Double: float64(x)}, true
	case float64:
		return Value{Kind: KindDouble, Double: x}, true
	case string:
		return Value{Kind: KindString, Str: x}, true
	case []interface{}:
		if len(x) == 0 {
			// ambiguous empty -> choose string array
			return Value{Kind: KindStringArray, Strs: []string{}}, true
		}
		switch x[0].(type) {
		case bool:
			out := make([]bool, 0, len(x))
			for _, e := range x {
				b, ok := e.(bool)
				if !ok {
					return Value{}, false
				}
				out = append(out, b)
			}
			return Value{Kind: KindBoolArray, Bools: out}, true
		case int, int64:
			out := make([]int64, 0, len(x))
			for _, e := range x {
				switch vv := e.(type) {
				case int:
					out = append(out, int64(vv))
				case int64:
					out = append(out, vv)
				default:
					return Value{}, false
				}
			}
			return Value{Kind: KindInt64Array, Int64s: out}, true
		case float32, float64:
			out := make([]float64, 0, len(x))
			for _, e := range x {
				switch vv := e.(type) {
				case float32:
					out = append(out, float64(vv))
				case float64:
					out = append(out, vv)
				case int:
					out = append(out, float64(vv))
				case int64:
					out = append(out, float64(vv))
				default:
					return Value{}, false
				}
			}
			return Value{Kind: KindDoubleArray, Doubles: out}, true
		case string:
			out := make([]string, 0, len(x))
			for _, e := range x {
				s, ok := e.(string)
				if !ok {
					return Value{}, false
				}
				out = append(out, s)
			}
			return Value{Kind: KindStringArray, Strs: out}, true
		default:
			return Value{}, false
		}
	default:
		return Value{}, false
	}
}
