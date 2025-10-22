package params

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/merlindrones/rclgo/pkg/rclgo"
)

// buildTestArgs constructs rclgo.Args from a slice of argument strings.
// This simulates command-line parsing for testing purposes.
func buildTestArgs(args []string) (*rclgo.Args, error) {
	rclArgs, _, err := rclgo.ParseArgs(args)
	return rclArgs, err
}

func TestApplyOverrides_ScalarTypes(t *testing.T) {
	// Init RCL context for this test
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("test_node", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Declare parameters with defaults
	_, _ = mgr.Declare("bool_param", Value{Kind: KindBool, Bool: false}, Descriptor{})
	_, _ = mgr.Declare("int_param", Value{Kind: KindInt64, Int64: 0}, Descriptor{})
	_, _ = mgr.Declare("double_param", Value{Kind: KindDouble, Double: 0.0}, Descriptor{})
	_, _ = mgr.Declare("string_param", Value{Kind: KindString, Str: ""}, Descriptor{})

	// Build test args with CLI overrides
	args, err := buildTestArgs([]string{
		"--ros-args",
		"-p", "bool_param:=true",
		"-p", "int_param:=42",
		"-p", "double_param:=3.14",
		"-p", "string_param:=hello",
		"--",
	})
	if err != nil {
		t.Fatalf("buildTestArgs: %v", err)
	}

	// Apply overrides
	if err := ApplyOverrides(mgr, "test_node", args); err != nil {
		t.Fatalf("ApplyOverrides: %v", err)
	}

	// Verify bool parameter
	if p, ok := mgr.Get("bool_param"); !ok {
		t.Error("bool_param not found")
	} else if p.Value.Kind != KindBool || !p.Value.Bool {
		t.Errorf("bool_param: got %+v, want true", p.Value)
	}

	// Verify int parameter
	if p, ok := mgr.Get("int_param"); !ok {
		t.Error("int_param not found")
	} else if p.Value.Kind != KindInt64 || p.Value.Int64 != 42 {
		t.Errorf("int_param: got %+v, want 42", p.Value)
	}

	// Verify double parameter
	if p, ok := mgr.Get("double_param"); !ok {
		t.Error("double_param not found")
	} else if p.Value.Kind != KindDouble || p.Value.Double != 3.14 {
		t.Errorf("double_param: got %+v, want 3.14", p.Value)
	}

	// Verify string parameter
	if p, ok := mgr.Get("string_param"); !ok {
		t.Error("string_param not found")
	} else if p.Value.Kind != KindString || p.Value.Str != "hello" {
		t.Errorf("string_param: got %+v, want 'hello'", p.Value)
	}
}

func TestApplyOverrides_ArrayTypes(t *testing.T) {
	// Init RCL context for this test
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("test_node_arrays", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Declare array parameters with empty defaults
	_, _ = mgr.Declare("bool_array", Value{Kind: KindBoolArray, Bools: []bool{}}, Descriptor{})
	_, _ = mgr.Declare("int_array", Value{Kind: KindInt64Array, Int64s: []int64{}}, Descriptor{})
	_, _ = mgr.Declare("double_array", Value{Kind: KindDoubleArray, Doubles: []float64{}}, Descriptor{})
	_, _ = mgr.Declare("string_array", Value{Kind: KindStringArray, Strs: []string{}}, Descriptor{})

	// Build test args with array overrides
	args, err := buildTestArgs([]string{
		"--ros-args",
		"-p", "bool_array:=[true,false,true]",
		"-p", "int_array:=[1,2,3]",
		"-p", "double_array:=[1.1,2.2,3.3]",
		"-p", "string_array:=[a,b,c]",
		"--",
	})
	if err != nil {
		t.Fatalf("buildTestArgs: %v", err)
	}

	// Apply overrides
	if err := ApplyOverrides(mgr, "test_node_arrays", args); err != nil {
		t.Fatalf("ApplyOverrides: %v", err)
	}

	// Verify bool array
	if p, ok := mgr.Get("bool_array"); !ok {
		t.Error("bool_array not found")
	} else if p.Value.Kind != KindBoolArray {
		t.Errorf("bool_array: wrong type %v", p.Value.Kind)
	} else if len(p.Value.Bools) != 3 || !p.Value.Bools[0] || p.Value.Bools[1] || !p.Value.Bools[2] {
		t.Errorf("bool_array: got %v, want [true,false,true]", p.Value.Bools)
	}

	// Verify int array
	if p, ok := mgr.Get("int_array"); !ok {
		t.Error("int_array not found")
	} else if p.Value.Kind != KindInt64Array {
		t.Errorf("int_array: wrong type %v", p.Value.Kind)
	} else if len(p.Value.Int64s) != 3 || p.Value.Int64s[0] != 1 || p.Value.Int64s[1] != 2 || p.Value.Int64s[2] != 3 {
		t.Errorf("int_array: got %v, want [1,2,3]", p.Value.Int64s)
	}

	// Verify double array
	if p, ok := mgr.Get("double_array"); !ok {
		t.Error("double_array not found")
	} else if p.Value.Kind != KindDoubleArray {
		t.Errorf("double_array: wrong type %v", p.Value.Kind)
	} else if len(p.Value.Doubles) != 3 || p.Value.Doubles[0] != 1.1 || p.Value.Doubles[1] != 2.2 || p.Value.Doubles[2] != 3.3 {
		t.Errorf("double_array: got %v, want [1.1,2.2,3.3]", p.Value.Doubles)
	}

	// Verify string array
	if p, ok := mgr.Get("string_array"); !ok {
		t.Error("string_array not found")
	} else if p.Value.Kind != KindStringArray {
		t.Errorf("string_array: wrong type %v", p.Value.Kind)
	} else if len(p.Value.Strs) != 3 || p.Value.Strs[0] != "a" || p.Value.Strs[1] != "b" || p.Value.Strs[2] != "c" {
		t.Errorf("string_array: got %v, want [a,b,c]", p.Value.Strs)
	}
}

func TestApplyOverrides_WildcardNode(t *testing.T) {
	// Init RCL context for this test
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("wildcard_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Declare a parameter
	_, _ = mgr.Declare("wildcard_param", Value{Kind: KindInt64, Int64: 0}, Descriptor{})

	// Build test args with wildcard node (/**) - should apply to any node
	args, err := buildTestArgs([]string{
		"--ros-args",
		"-p", "wildcard_param:=999",
		"--",
	})
	if err != nil {
		t.Fatalf("buildTestArgs: %v", err)
	}

	// Apply overrides - the wildcard logic is in ApplyOverrides
	// It matches "/**" OR the specific node name
	if err := ApplyOverrides(mgr, "wildcard_test", args); err != nil {
		t.Fatalf("ApplyOverrides: %v", err)
	}

	// Verify parameter was set
	if p, ok := mgr.Get("wildcard_param"); !ok {
		t.Error("wildcard_param not found")
	} else if p.Value.Kind != KindInt64 || p.Value.Int64 != 999 {
		t.Errorf("wildcard_param: got %+v, want 999", p.Value)
	}
}

func TestApplyOverrides_WithYAML(t *testing.T) {
	// Init RCL context for this test
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("yaml_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Create temporary YAML file
	tmpDir := t.TempDir()
	yamlPath := filepath.Join(tmpDir, "test_params.yaml")
	yamlContent := `yaml_test:
  ros__parameters:
    fps: 15
    resolution: 720
    name: "from_yaml"
`
	if err := os.WriteFile(yamlPath, []byte(yamlContent), 0644); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}

	// Load parameters from YAML
	if err := LoadYAML(mgr, "yaml_test", yamlPath); err != nil {
		t.Fatalf("LoadYAML: %v", err)
	}

	// Verify YAML values loaded
	if p, ok := mgr.Get("fps"); !ok || p.Value.Int64 != 15 {
		t.Errorf("fps from YAML: got %+v, want 15", p)
	}
	if p, ok := mgr.Get("resolution"); !ok || p.Value.Int64 != 720 {
		t.Errorf("resolution from YAML: got %+v, want 720", p)
	}
	if p, ok := mgr.Get("name"); !ok || p.Value.Str != "from_yaml" {
		t.Errorf("name from YAML: got %+v, want 'from_yaml'", p)
	}

	// Now apply CLI overrides - should override YAML values
	args, err := buildTestArgs([]string{
		"--ros-args",
		"-p", "fps:=30", // Override YAML value
		"-p", "name:=from_cli", // Override YAML value
		// resolution NOT overridden - should keep YAML value
		"--",
	})
	if err != nil {
		t.Fatalf("buildTestArgs: %v", err)
	}

	if err := ApplyOverrides(mgr, "yaml_test", args); err != nil {
		t.Fatalf("ApplyOverrides: %v", err)
	}

	// Verify CLI overrides took precedence
	if p, ok := mgr.Get("fps"); !ok || p.Value.Int64 != 30 {
		t.Errorf("fps after CLI override: got %+v, want 30 (CLI should override YAML)", p)
	}
	if p, ok := mgr.Get("name"); !ok || p.Value.Str != "from_cli" {
		t.Errorf("name after CLI override: got %+v, want 'from_cli' (CLI should override YAML)", p)
	}
	// Resolution should still be from YAML
	if p, ok := mgr.Get("resolution"); !ok || p.Value.Int64 != 720 {
		t.Errorf("resolution after CLI override: got %+v, want 720 (YAML value should remain)", p)
	}
}

func TestApplyOverrides_NoArgs(t *testing.T) {
	// Init RCL context for this test
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("no_args_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Declare a parameter
	_, _ = mgr.Declare("test_param", Value{Kind: KindInt64, Int64: 100}, Descriptor{})

	// Apply overrides with nil args - should be no-op
	if err := ApplyOverrides(mgr, "no_args_test", nil); err != nil {
		t.Fatalf("ApplyOverrides with nil args: %v", err)
	}

	// Verify parameter unchanged
	if p, ok := mgr.Get("test_param"); !ok || p.Value.Int64 != 100 {
		t.Errorf("test_param: got %+v, want 100 (should be unchanged)", p)
	}

	// Apply overrides with empty args - should also be no-op
	emptyArgs, err := buildTestArgs([]string{})
	if err != nil {
		t.Fatalf("buildTestArgs: %v", err)
	}

	if err := ApplyOverrides(mgr, "no_args_test", emptyArgs); err != nil {
		t.Fatalf("ApplyOverrides with empty args: %v", err)
	}

	// Verify parameter still unchanged
	if p, ok := mgr.Get("test_param"); !ok || p.Value.Int64 != 100 {
		t.Errorf("test_param: got %+v, want 100 (should still be unchanged)", p)
	}
}

func TestApplyOverrides_UndeclaredParam(t *testing.T) {
	// Init RCL context for this test
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("undeclared_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// DO NOT declare the parameter first

	// Build args to override a parameter that doesn't exist yet
	args, err := buildTestArgs([]string{
		"--ros-args",
		"-p", "new_param:=123",
		"--",
	})
	if err != nil {
		t.Fatalf("buildTestArgs: %v", err)
	}

	// Apply overrides - should declare the parameter if it doesn't exist
	if err := ApplyOverrides(mgr, "undeclared_test", args); err != nil {
		t.Fatalf("ApplyOverrides: %v", err)
	}

	// Verify parameter was declared and set
	if p, ok := mgr.Get("new_param"); !ok {
		t.Error("new_param should have been declared")
	} else if p.Value.Kind != KindInt64 || p.Value.Int64 != 123 {
		t.Errorf("new_param: got %+v, want 123", p.Value)
	}
}
