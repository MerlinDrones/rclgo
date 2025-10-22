package params

import (
	"os"
	"testing"

	"github.com/merlindrones/rclgo/pkg/rclgo"
)

func TestApplyEnvVars_ScalarTypes(t *testing.T) {
	// Set environment variables
	os.Setenv("RCL_PARAM_bool_var", "true")
	os.Setenv("RCL_PARAM_int_var", "42")
	os.Setenv("RCL_PARAM_double_var", "3.14")
	os.Setenv("RCL_PARAM_string_var", "hello")
	defer func() {
		os.Unsetenv("RCL_PARAM_bool_var")
		os.Unsetenv("RCL_PARAM_int_var")
		os.Unsetenv("RCL_PARAM_double_var")
		os.Unsetenv("RCL_PARAM_string_var")
	}()

	// Init RCL context
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("env_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Apply env vars
	if err := ApplyEnvVars(mgr); err != nil {
		t.Fatalf("ApplyEnvVars: %v", err)
	}

	// Verify bool (inferred from "true")
	if p, ok := mgr.Get("bool_var"); !ok {
		t.Error("bool_var not found")
	} else if p.Value.Kind != KindBool || !p.Value.Bool {
		t.Errorf("bool_var: got %+v, want true", p.Value)
	}

	// Verify int (inferred from "42")
	if p, ok := mgr.Get("int_var"); !ok {
		t.Error("int_var not found")
	} else if p.Value.Kind != KindInt64 || p.Value.Int64 != 42 {
		t.Errorf("int_var: got %+v, want 42", p.Value)
	}

	// Verify double (inferred from "3.14")
	if p, ok := mgr.Get("double_var"); !ok {
		t.Error("double_var not found")
	} else if p.Value.Kind != KindDouble || p.Value.Double != 3.14 {
		t.Errorf("double_var: got %+v, want 3.14", p.Value)
	}

	// Verify string (inferred as string)
	if p, ok := mgr.Get("string_var"); !ok {
		t.Error("string_var not found")
	} else if p.Value.Kind != KindString || p.Value.Str != "hello" {
		t.Errorf("string_var: got %+v, want 'hello'", p.Value)
	}
}

func TestApplyEnvVars_ArrayTypes(t *testing.T) {
	// Set environment variables with array syntax
	os.Setenv("RCL_PARAM_bool_array", "[true,false,true]")
	os.Setenv("RCL_PARAM_int_array", "[1,2,3]")
	os.Setenv("RCL_PARAM_double_array", "[1.1,2.2,3.3]")
	os.Setenv("RCL_PARAM_string_array", "[a,b,c]")
	defer func() {
		os.Unsetenv("RCL_PARAM_bool_array")
		os.Unsetenv("RCL_PARAM_int_array")
		os.Unsetenv("RCL_PARAM_double_array")
		os.Unsetenv("RCL_PARAM_string_array")
	}()

	// Init RCL context
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("env_array_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Apply env vars
	if err := ApplyEnvVars(mgr); err != nil {
		t.Fatalf("ApplyEnvVars: %v", err)
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

func TestApplyEnvVars_TypeFromExisting(t *testing.T) {
	// Set env var that will be interpreted based on existing param type
	os.Setenv("RCL_PARAM_existing_param", "123")
	defer os.Unsetenv("RCL_PARAM_existing_param")

	// Init RCL context
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("env_existing_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Declare parameter as string type with default value
	_, _ = mgr.Declare("existing_param", Value{Kind: KindString, Str: "default"}, Descriptor{})

	// Apply env vars - should use declared type (string), not infer as int
	if err := ApplyEnvVars(mgr); err != nil {
		t.Fatalf("ApplyEnvVars: %v", err)
	}

	// Verify parameter is still string type (not int)
	if p, ok := mgr.Get("existing_param"); !ok {
		t.Error("existing_param not found")
	} else if p.Value.Kind != KindString {
		t.Errorf("existing_param: got type %v, want KindString", p.Value.Kind)
	} else if p.Value.Str != "123" {
		t.Errorf("existing_param: got %v, want '123' (as string)", p.Value.Str)
	}
}

func TestApplyEnvVars_EmptyEnvironment(t *testing.T) {
	// No RCL_PARAM_* env vars set

	// Init RCL context
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("env_empty_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Declare a parameter with default
	_, _ = mgr.Declare("test_param", Value{Kind: KindInt64, Int64: 100}, Descriptor{})

	// Apply env vars - should be no-op
	if err := ApplyEnvVars(mgr); err != nil {
		t.Fatalf("ApplyEnvVars: %v", err)
	}

	// Verify parameter unchanged
	if p, ok := mgr.Get("test_param"); !ok {
		t.Error("test_param not found")
	} else if p.Value.Int64 != 100 {
		t.Errorf("test_param: got %v, want 100 (unchanged)", p.Value.Int64)
	}
}

func TestApplyEnvVars_DotNotation(t *testing.T) {
	// Test parameter names with dots (common in ROS)
	os.Setenv("RCL_PARAM_camera.fps", "30")
	os.Setenv("RCL_PARAM_robot.wheel.diameter", "0.5")
	defer func() {
		os.Unsetenv("RCL_PARAM_camera.fps")
		os.Unsetenv("RCL_PARAM_robot.wheel.diameter")
	}()

	// Init RCL context
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("env_dot_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Apply env vars
	if err := ApplyEnvVars(mgr); err != nil {
		t.Fatalf("ApplyEnvVars: %v", err)
	}

	// Verify camera.fps
	if p, ok := mgr.Get("camera.fps"); !ok {
		t.Error("camera.fps not found")
	} else if p.Value.Int64 != 30 {
		t.Errorf("camera.fps: got %v, want 30", p.Value.Int64)
	}

	// Verify robot.wheel.diameter
	if p, ok := mgr.Get("robot.wheel.diameter"); !ok {
		t.Error("robot.wheel.diameter not found")
	} else if p.Value.Double != 0.5 {
		t.Errorf("robot.wheel.diameter: got %v, want 0.5", p.Value.Double)
	}
}

func TestParseEnvVarValue_TypeInference(t *testing.T) {
	tests := []struct {
		name     string
		input    string
		wantKind uint8
		wantVal  interface{}
	}{
		{"bool_true", "true", KindBool, true},
		{"bool_false", "false", KindBool, false},
		{"int", "42", KindInt64, int64(42)},
		{"int_negative", "-123", KindInt64, int64(-123)},
		{"double", "3.14", KindDouble, 3.14},
		{"double_negative", "-2.5", KindDouble, -2.5},
		{"string", "hello", KindString, "hello"},
		{"string_numeric_like", "123abc", KindString, "123abc"},
		{"bool_array", "[true,false]", KindBoolArray, []bool{true, false}},
		{"int_array", "[1,2,3]", KindInt64Array, []int64{1, 2, 3}},
		{"double_array", "[1.1,2.2]", KindDoubleArray, []float64{1.1, 2.2}},
		{"string_array", "[a,b,c]", KindStringArray, []string{"a", "b", "c"}},
		{"empty_array", "[]", KindStringArray, []string{}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			val, err := inferValueType(tt.input)
			if err != nil {
				t.Fatalf("inferValueType(%q): %v", tt.input, err)
			}
			if val.Kind != tt.wantKind {
				t.Errorf("Kind: got %v, want %v", val.Kind, tt.wantKind)
			}

			// Check actual value based on type
			switch tt.wantKind {
			case KindBool:
				if val.Bool != tt.wantVal.(bool) {
					t.Errorf("Bool: got %v, want %v", val.Bool, tt.wantVal)
				}
			case KindInt64:
				if val.Int64 != tt.wantVal.(int64) {
					t.Errorf("Int64: got %v, want %v", val.Int64, tt.wantVal)
				}
			case KindDouble:
				if val.Double != tt.wantVal.(float64) {
					t.Errorf("Double: got %v, want %v", val.Double, tt.wantVal)
				}
			case KindString:
				if val.Str != tt.wantVal.(string) {
					t.Errorf("Str: got %v, want %v", val.Str, tt.wantVal)
				}
			case KindBoolArray:
				want := tt.wantVal.([]bool)
				if len(val.Bools) != len(want) {
					t.Errorf("BoolArray length: got %d, want %d", len(val.Bools), len(want))
				} else {
					for i := range want {
						if val.Bools[i] != want[i] {
							t.Errorf("BoolArray[%d]: got %v, want %v", i, val.Bools[i], want[i])
						}
					}
				}
			case KindInt64Array:
				want := tt.wantVal.([]int64)
				if len(val.Int64s) != len(want) {
					t.Errorf("Int64Array length: got %d, want %d", len(val.Int64s), len(want))
				} else {
					for i := range want {
						if val.Int64s[i] != want[i] {
							t.Errorf("Int64Array[%d]: got %v, want %v", i, val.Int64s[i], want[i])
						}
					}
				}
			case KindDoubleArray:
				want := tt.wantVal.([]float64)
				if len(val.Doubles) != len(want) {
					t.Errorf("DoubleArray length: got %d, want %d", len(val.Doubles), len(want))
				} else {
					for i := range want {
						if val.Doubles[i] != want[i] {
							t.Errorf("DoubleArray[%d]: got %v, want %v", i, val.Doubles[i], want[i])
						}
					}
				}
			case KindStringArray:
				want := tt.wantVal.([]string)
				if len(val.Strs) != len(want) {
					t.Errorf("StringArray length: got %d, want %d", len(val.Strs), len(want))
				} else {
					for i := range want {
						if val.Strs[i] != want[i] {
							t.Errorf("StringArray[%d]: got %v, want %v", i, val.Strs[i], want[i])
						}
					}
				}
			}
		})
	}
}

func TestGetParamEnvVars(t *testing.T) {
	// Set various env vars
	os.Setenv("RCL_PARAM_test1", "value1")
	os.Setenv("RCL_PARAM_test2", "value2")
	os.Setenv("OTHER_VAR", "should_be_ignored")
	os.Setenv("RCL_PARAM_", "should_be_ignored_empty_name")
	defer func() {
		os.Unsetenv("RCL_PARAM_test1")
		os.Unsetenv("RCL_PARAM_test2")
		os.Unsetenv("OTHER_VAR")
		os.Unsetenv("RCL_PARAM_")
	}()

	result := getParamEnvVars()

	// Should find test1 and test2
	if val, ok := result["test1"]; !ok || val != "value1" {
		t.Errorf("test1: got %v, want 'value1'", val)
	}
	if val, ok := result["test2"]; !ok || val != "value2" {
		t.Errorf("test2: got %v, want 'value2'", val)
	}

	// Should not find OTHER_VAR or empty name
	if _, ok := result["OTHER_VAR"]; ok {
		t.Error("OTHER_VAR should not be in result")
	}
	if _, ok := result[""]; ok {
		t.Error("empty name should not be in result")
	}

	// Should have exactly 2 entries
	if len(result) != 2 {
		t.Errorf("result length: got %d, want 2", len(result))
	}
}
