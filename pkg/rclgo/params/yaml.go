package params

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"gopkg.in/yaml.v3"
)

// ErrNoMatchingSection is returned, wrapped, by LoadYAML when the file has no
// parameter section for the node. (rcl ignores such a file; LoadYAML reports
// it because the caller named the file for this node.)
var ErrNoMatchingSection = errors.New("no ros__parameters block found")

// LoadYAML loads parameters for a node from a ROS 2 parameter file, as rcl
// reads --params-file for rclcpp and rclpy, and applies them as overrides: a
// later Declare in code gives them its type and constraints.
//
// Every section that applies to the node is used, in file order, a later one
// overriding an earlier one. A section's name is its key, or the path of nested
// namespace keys above ros__parameters, and it is matched against the node's
// fully qualified name (the manager's node, or "/"+nodeName) with rcl's rules:
// "/*" matches one name token and "/**" any number, as in
//
//	/**:
//	  ros__parameters:
//	    use_sim_time: true
//	/robot/camera:          # or robot: {camera: {ros__parameters: ...}}
//	  ros__parameters:
//	    camera.fps: 30
//
// As an rclgo extension, a top-level ros__parameters block applies to any node.
// Nested maps under ros__parameters become dotted names (camera: {fps: 30} is
// camera.fps). Value kinds are inferred from the YAML scalars and sequences.
//
// Typical use:
//
//	_ = params.LoadYAML(mgr, "param_demo", "/path/to/params.yaml")
func LoadYAML(m *Manager, nodeName string, path string) error {
	data, err := os.ReadFile(path)
	if err != nil {
		return fmt.Errorf("read %s: %w", path, err)
	}
	var doc yaml.Node
	if err := yaml.Unmarshal(data, &doc); err != nil {
		return fmt.Errorf("yaml unmarshal %s: %w", path, err)
	}
	var root *yaml.Node
	if len(doc.Content) == 1 && doc.Content[0].Kind == yaml.MappingNode {
		root = doc.Content[0]
	}

	names := matchNames(m, nodeName)
	var (
		order  []string
		values = map[string]interface{}{}
		found  bool
	)
	apply := func(params *yaml.Node) error {
		found = true
		return flattenYAMLNode("", params, func(name string, v interface{}) {
			if _, seen := values[name]; !seen {
				order = append(order, name)
			}
			values[name] = v
		})
	}
	var walk func(section string, n *yaml.Node) error
	walk = func(section string, n *yaml.Node) error {
		if n.Kind != yaml.MappingNode {
			return nil
		}
		for i := 0; i+1 < len(n.Content); i += 2 {
			key, val := n.Content[i].Value, n.Content[i+1]
			if key == "ros__parameters" {
				if section == "" || sectionMatches(section, names) {
					if err := apply(val); err != nil {
						return err
					}
				}
				continue
			}
			if err := walk(strings.TrimSuffix(section, "/")+"/"+strings.TrimPrefix(key, "/"), val); err != nil {
				return err
			}
		}
		return nil
	}
	if root != nil {
		if err := walk("", root); err != nil {
			return fmt.Errorf("%s: %w", filepath.Base(path), err)
		}
	}
	if !found {
		return fmt.Errorf("%w for node %q in %s", ErrNoMatchingSection, nodeName, filepath.Base(path))
	}

	for _, name := range order {
		val, ok := valueFromYAML(values[name])
		if !ok {
			return fmt.Errorf("unsupported YAML type for %s", name)
		}
		// A later Declare in code attaches its descriptor to this value.
		if err := m.applyOverride(name, val, Descriptor{}); err != nil {
			return fmt.Errorf("set %s: %w", name, err)
		}
	}
	return nil
}

// flattenYAMLNode calls set for every parameter under n in document order,
// turning nested maps into dotted names. A ros__parameters value that isn't a
// map yields no parameters.
func flattenYAMLNode(prefix string, n *yaml.Node, set func(name string, v interface{})) error {
	if n.Kind == yaml.AliasNode && n.Alias != nil {
		n = n.Alias
	}
	if n.Kind != yaml.MappingNode {
		return nil
	}
	for i := 0; i+1 < len(n.Content); i += 2 {
		name := n.Content[i].Value
		if prefix != "" {
			name = prefix + "." + name
		}
		v := n.Content[i+1]
		if v.Kind == yaml.AliasNode && v.Alias != nil {
			v = v.Alias
		}
		if v.Kind == yaml.MappingNode {
			if err := flattenYAMLNode(name, v, set); err != nil {
				return err
			}
			continue
		}
		raw, err := decodeRCLValue(v)
		if err != nil {
			return fmt.Errorf("parameter %s: %w", name, err)
		}
		set(name, raw)
	}
	return nil
}

// decodeRCLValue decodes a scalar or sequence as rcl's parameter parser reads
// it: like YAML, except that unquoted y/yes/on and n/no/off (in any of their
// usual capitalizations) are booleans, as in YAML 1.1.
func decodeRCLValue(n *yaml.Node) (interface{}, error) {
	switch n.Kind {
	case yaml.ScalarNode:
		if n.Style == 0 && n.Tag == "!!str" {
			if b, ok := rclBool(n.Value); ok {
				return b, nil
			}
		}
	case yaml.SequenceNode:
		items := make([]interface{}, 0, len(n.Content))
		for _, c := range n.Content {
			v, err := decodeRCLValue(c)
			if err != nil {
				return nil, err
			}
			items = append(items, v)
		}
		return items, nil
	}
	var raw interface{}
	err := n.Decode(&raw)
	return raw, err
}

// rclBool reads the boolean spellings rcl's parameter parser accepts.
func rclBool(s string) (value, ok bool) {
	switch s {
	case "true", "True", "TRUE", "y", "Y", "yes", "Yes", "YES", "on", "On", "ON":
		return true, true
	case "false", "False", "FALSE", "n", "N", "no", "No", "NO", "off", "Off", "OFF":
		return false, true
	}
	return false, false
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
		case int, int64, float32, float64:
			// A list of integers is an integer array; a list with a float
			// anywhere is a double array (whatever the order of its elements).
			if !hasFloat(x) {
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
			}
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

func hasFloat(xs []interface{}) bool {
	for _, x := range xs {
		switch x.(type) {
		case float32, float64:
			return true
		}
	}
	return false
}
