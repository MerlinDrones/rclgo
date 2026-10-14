package params

import (
	"strings"
	"testing"

	rcl_interfaces_msg "github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
)

// managerWithYAML returns a manager after loading the given ros__parameters
// body, the way nodes load their parameter file before declaring.
func managerWithYAML(t *testing.T, body string) *Manager {
	t.Helper()
	path := writeTemp(t, t.TempDir(), "params.yaml", "ros__parameters:\n"+body)
	m := newBareManager()
	if err := LoadYAML(m, "node", path); err != nil {
		t.Fatalf("LoadYAML: %v", err)
	}
	return m
}

// A parameter given in a parameter file and then declared in code keeps the
// file's value but gets the code's descriptor, so its constraints apply.
func TestDeclareAfterOverrideAppliesDescriptor(t *testing.T) {
	for _, declare := range []struct {
		name string
		f    func(m *Manager, v Value, d Descriptor) error
	}{
		{"Declare", func(m *Manager, v Value, d Descriptor) error { _, err := m.Declare("rate", v, d); return err }},
		{"DeclareIfMissing", func(m *Manager, v Value, d Descriptor) error { return m.DeclareIfMissing("rate", v, d) }},
	} {
		t.Run(declare.name, func(t *testing.T) {
			m := managerWithYAML(t, "  rate: 50\n")
			err := declare.f(m, Value{Kind: KindInt64, Int64: 10}, Descriptor{MinInt: i64(1), MaxInt: i64(100)})
			if err != nil {
				t.Fatalf("declare: %v", err)
			}
			if p, _ := m.Get("rate"); p.Value.Int64 != 50 {
				t.Fatalf("rate = %v, want the file's 50", p.Value)
			}
			if _, err := m.SetValue("rate", Value{Kind: KindInt64, Int64: 500}); err == nil {
				t.Fatal("set above the declared max succeeded: constraints were not applied")
			}
		})
	}
}

func TestDeclareRejectsOverrideOutsideConstraints(t *testing.T) {
	m := managerWithYAML(t, "  rate: 500\n")
	_, err := m.Declare("rate", Value{Kind: KindInt64, Int64: 10}, Descriptor{MaxInt: i64(100)})
	if err == nil {
		t.Fatal("Declare accepted an override above the declared max")
	}
	if !strings.Contains(err.Error(), "rate") || !strings.Contains(err.Error(), "above max") {
		t.Errorf("error does not name the parameter and the violated constraint: %v", err)
	}	// As in rclcpp, the parameter stays undeclared: not readable, and not
	// settable to an arbitrary value with no descriptor.
	if p, ok := m.Get("rate"); ok {
		t.Errorf("the rejected parameter is still present: %+v", p)
	}
	if _, err := m.SetValue("rate", Value{Kind: KindString, Str: "anything"}); err == nil {
		t.Error("the rejected parameter can still be set")
	}
	if _, err := m.Declare("rate", Value{Kind: KindInt64, Int64: 10}, Descriptor{MaxInt: i64(100)}); err != nil {
		t.Errorf("a later declaration failed: %v", err)
	}
}

// An override reaches a read-only parameter whichever comes first, the
// override or the declaration; only later sets are refused.
func TestOverrideOfReadOnlyParameter(t *testing.T) {
	t.Run("override first", func(t *testing.T) {
		m := managerWithYAML(t, "  frame: map\n")
		if _, err := m.Declare("frame", Value{Kind: KindString, Str: "odom"}, Descriptor{ReadOnly: true}); err != nil {
			t.Fatalf("Declare: %v", err)
		}
		if p, _ := m.Get("frame"); p.Value.Str != "map" {
			t.Fatalf("frame = %q, want the file's map", p.Value.Str)
		}
		if _, err := m.SetValue("frame", Value{Kind: KindString, Str: "x"}); err == nil {
			t.Fatal("set of a read-only parameter succeeded")
		}
	})
	t.Run("declaration first", func(t *testing.T) {
		m := newBareManager()
		if _, err := m.Declare("frame", Value{Kind: KindString, Str: "odom"}, Descriptor{ReadOnly: true}); err != nil {
			t.Fatalf("Declare: %v", err)
		}
		path := writeTemp(t, t.TempDir(), "params.yaml", "ros__parameters:\n  frame: map\n")
		if err := LoadYAML(m, "node", path); err != nil {
			t.Fatalf("LoadYAML: %v", err)
		}
		if p, _ := m.Get("frame"); p.Value.Str != "map" {
			t.Fatalf("frame = %q, want the file's map", p.Value.Str)
		}
	})
}

// An integer override for a double parameter is widened; other type
// mismatches are refused.
func TestOverrideTypeIsCheckedAgainstDeclaration(t *testing.T) {
	m := managerWithYAML(t, "  gain: 5\n  gains: [1, 2]\n  label: 5\n")
	if _, err := m.Declare("gain", Value{Kind: KindDouble, Double: 0.5}, Descriptor{}); err != nil {
		t.Fatalf("Declare gain: %v", err)
	}
	if p, _ := m.Get("gain"); p.Value.Kind != KindDouble || p.Value.Double != 5 {
		t.Fatalf("gain = %+v, want double 5", p.Value)
	}
	if _, err := m.Declare("gains", Value{Kind: KindDoubleArray, Doubles: []float64{0}}, Descriptor{}); err != nil {
		t.Fatalf("Declare gains: %v", err)
	}
	if p, _ := m.Get("gains"); p.Value.Kind != KindDoubleArray || len(p.Value.Doubles) != 2 || p.Value.Doubles[1] != 2 {
		t.Fatalf("gains = %+v, want doubles [1 2]", p.Value)
	}
	if _, err := m.Declare("label", Value{Kind: KindString, Str: "x"}, Descriptor{}); err == nil || !strings.Contains(err.Error(), "type") {
		t.Fatalf("Declare of an int override for a string parameter: got %v, want a type error", err)
	}
}

// A parameter declared in code keeps its type on set.
func TestSetKeepsDeclaredType(t *testing.T) {
	m := newBareManager()
	if _, err := m.Declare("gain", Value{Kind: KindDouble, Double: 0.5}, Descriptor{}); err != nil {
		t.Fatalf("Declare: %v", err)
	}
	if _, err := m.SetValue("gain", Value{Kind: KindInt64, Int64: 3}); err != nil {
		t.Fatalf("set int for double: %v", err)
	}
	if p, _ := m.Get("gain"); p.Value.Kind != KindDouble || p.Value.Double != 3 {
		t.Fatalf("gain = %+v, want double 3", p.Value)
	}
	if _, err := m.SetValue("gain", Value{Kind: KindString, Str: "fast"}); err == nil {
		t.Fatal("set string for double succeeded")
	}
	if p, _ := m.Get("gain"); p.Value.Kind != KindDouble {
		t.Fatalf("failed set changed the value: %+v", p.Value)
	}
}

// Overrides from environment variables and the command line follow the same
// rules as the parameter file.
func TestDeclareAfterEnvAndCLIOverrides(t *testing.T) {
	t.Run("environment", func(t *testing.T) {
		t.Setenv("RCL_PARAM_rate", "500")
		m := newBareManager()
		if err := ApplyEnvVars(m); err != nil {
			t.Fatalf("ApplyEnvVars: %v", err)
		}
		_, err := m.Declare("rate", Value{Kind: KindInt64, Int64: 10}, Descriptor{MaxInt: i64(100)})
		if err == nil || !strings.Contains(err.Error(), "above max") {
			t.Fatalf("Declare of an environment override above the declared max: got %v, want an above-max error", err)
		}
	})
	t.Run("command line", func(t *testing.T) {
		if err := rclgo.Init(nil); err != nil {
			t.Fatalf("rclgo.Init: %v", err)
		}
		defer rclgo.Uninit() //nolint:errcheck
		n, err := rclgo.NewNode("cli_node", "")
		if err != nil {
			t.Fatalf("NewNode: %v", err)
		}
		args, err := buildTestArgs([]string{"--ros-args", "-p", "rate:=50"})
		if err != nil {
			t.Fatalf("ParseArgs: %v", err)
		}
		m := newBareManagerFor(n)
		if err := ApplyOverrides(m, "cli_node", args); err != nil {
			t.Fatalf("ApplyOverrides: %v", err)
		}
		if _, err := m.Declare("rate", Value{Kind: KindInt64, Int64: 10}, Descriptor{MaxInt: i64(100)}); err != nil {
			t.Fatalf("Declare: %v", err)
		}
		if _, err := m.SetValue("rate", Value{Kind: KindInt64, Int64: 500}); err == nil {
			t.Fatal("set above the declared max succeeded: constraints were not applied")
		}
	})
}

// On set, a ParameterEvent carries the new values in changed_parameters;
// new_parameters is for declarations only.
func TestParameterEventFields(t *testing.T) {
	var events []*rcl_interfaces_msg.ParameterEvent
	m := newBareManager()
	m.eventSink = func(e *rcl_interfaces_msg.ParameterEvent) { events = append(events, e) }

	if _, err := m.Declare("gain", Value{Kind: KindDouble, Double: 0.5}, Descriptor{}); err != nil {
		t.Fatalf("Declare: %v", err)
	}
	if _, err := m.SetValue("gain", Value{Kind: KindDouble, Double: 0.9}); err != nil {
		t.Fatalf("SetValue: %v", err)
	}
	if len(events) != 2 {
		t.Fatalf("got %d events, want 2", len(events))
	}
	if d := events[0]; len(d.NewParameters) != 1 || len(d.ChangedParameters) != 0 {
		t.Fatalf("declare event: new=%d changed=%d, want 1 and 0", len(d.NewParameters), len(d.ChangedParameters))
	}
	set := events[1]
	if len(set.NewParameters) != 0 || len(set.ChangedParameters) != 1 {
		t.Fatalf("set event: new=%d changed=%d, want 0 and 1", len(set.NewParameters), len(set.ChangedParameters))
	}
	if got := set.ChangedParameters[0].Value.DoubleValue; got != 0.9 {
		t.Fatalf("changed_parameters carries %v, want the new value 0.9", got)
	}
}
