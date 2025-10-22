package params

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// writeTemp creates a temp file with the given contents and returns its path.
func writeTemp(t *testing.T, dir, name, contents string) string {
	t.Helper()
	p := filepath.Join(dir, name)
	if err := os.WriteFile(p, []byte(contents), 0o644); err != nil {
		t.Fatalf("write %s: %v", p, err)
	}
	return p
}

func TestLoadYAML_TopLevel(t *testing.T) {
	td := t.TempDir()
	yaml := `
ros__parameters:
  flag: true
  count: 42
  ratio: 0.25
  name: "front"
  bools: [true, false, true]
  ints: [1, 2, 3]
  doubles: [1.5, 2.5, 3.5]
  strings: ["a", "b"]
  empty: []
`
	path := writeTemp(t, td, "params.yaml", yaml)

	m := &Manager{st: newStore()}

	if err := LoadYAML(m, "ignored_node", path); err != nil {
		t.Fatalf("LoadYAML: %v", err)
	}

	// Scalars
	if p, ok := m.Get("flag"); !ok || p.Value.Kind != KindBool || !p.Value.Bool {
		t.Fatalf("flag wrong: ok=%v val=%+v", ok, p.Value)
	}
	if p, ok := m.Get("count"); !ok || p.Value.Kind != KindInt64 || p.Value.Int64 != 42 {
		t.Fatalf("count wrong: ok=%v val=%+v", ok, p.Value)
	}
	if p, ok := m.Get("ratio"); !ok || p.Value.Kind != KindDouble || p.Value.Double != 0.25 {
		t.Fatalf("ratio wrong: ok=%v val=%+v", ok, p.Value)
	}
	if p, ok := m.Get("name"); !ok || p.Value.Kind != KindString || p.Value.Str != "front" {
		t.Fatalf("name wrong: ok=%v val=%+v", ok, p.Value)
	}

	// Arrays
	if p, ok := m.Get("bools"); !ok || p.Value.Kind != KindBoolArray || len(p.Value.Bools) != 3 || !p.Value.Bools[0] || p.Value.Bools[1] {
		t.Fatalf("bools wrong: ok=%v val=%+v", ok, p.Value)
	}
	if p, ok := m.Get("ints"); !ok || p.Value.Kind != KindInt64Array || len(p.Value.Int64s) != 3 || p.Value.Int64s[2] != 3 {
		t.Fatalf("ints wrong: ok=%v val=%+v", ok, p.Value)
	}
	if p, ok := m.Get("doubles"); !ok || p.Value.Kind != KindDoubleArray || len(p.Value.Doubles) != 3 || p.Value.Doubles[1] != 2.5 {
		t.Fatalf("doubles wrong: ok=%v val=%+v", ok, p.Value)
	}
	if p, ok := m.Get("strings"); !ok || p.Value.Kind != KindStringArray || len(p.Value.Strs) != 2 || p.Value.Strs[0] != "a" {
		t.Fatalf("strings wrong: ok=%v val=%+v", ok, p.Value)
	}

	// Empty array defaults to string array (by design)
	if p, ok := m.Get("empty"); !ok || p.Value.Kind != KindStringArray || len(p.Value.Strs) != 0 {
		t.Fatalf("empty wrong: ok=%v val=%+v", ok, p.Value)
	}
}

func TestLoadYAML_NodeScoped(t *testing.T) {
	td := t.TempDir()
	yaml := `
param_demo:
  ros__parameters:
    camera.fps: 25
    camera.frame_id: "front_cam"
`
	path := writeTemp(t, td, "node_params.yaml", yaml)

	m := &Manager{st: newStore()}

	if err := LoadYAML(m, "param_demo", path); err != nil {
		t.Fatalf("LoadYAML: %v", err)
	}

	if p, ok := m.Get("camera.fps"); !ok || p.Value.Kind != KindInt64 || p.Value.Int64 != 25 {
		t.Fatalf("camera.fps wrong: ok=%v val=%+v", ok, p.Value)
	}
	if p, ok := m.Get("camera.frame_id"); !ok || p.Value.Kind != KindString || p.Value.Str != "front_cam" {
		t.Fatalf("camera.frame_id wrong: ok=%v val=%+v", ok, p.Value)
	}
}

func TestLoadYAML_NoBlockError(t *testing.T) {
	td := t.TempDir()
	yaml := `
some_other_key:
  not_params: true
`
	path := writeTemp(t, td, "bad.yaml", yaml)

	m := &Manager{st: newStore()}
	err := LoadYAML(m, "param_demo", path)
	if err == nil || !strings.Contains(err.Error(), "no ros__parameters block") {
		t.Fatalf("expected missing block error, got: %v", err)
	}
}

func TestLoadYAML_UnsupportedType(t *testing.T) {
	td := t.TempDir()
	// map value under ros__parameters should be rejected by valueFromYAML
	yaml := `
ros__parameters:
  complex: { a: 1, b: 2 }
`
	path := writeTemp(t, td, "unsupported.yaml", yaml)

	m := &Manager{st: newStore()}
	err := LoadYAML(m, "node", path)
	if err == nil || !strings.Contains(err.Error(), "unsupported YAML type for complex") {
		t.Fatalf("expected unsupported type error, got: %v", err)
	}
}

func TestLoadYAML_OverrideExisting(t *testing.T) {
	td := t.TempDir()
	yaml := `
ros__parameters:
  k: 10
`
	path := writeTemp(t, td, "override.yaml", yaml)

	m := &Manager{st: newStore()}
	// Pre-declare with different value
	_, _ = m.Declare("k", Value{Kind: KindInt64, Int64: 0}, Descriptor{})

	// LoadYAML should now set the existing parameter instead of erroring
	err := LoadYAML(m, "node", path)
	if err != nil {
		t.Fatalf("LoadYAML should set existing param, got error: %v", err)
	}

	// Verify the YAML value was set
	if p, ok := m.Get("k"); !ok {
		t.Fatal("parameter k not found")
	} else if p.Value.Int64 != 10 {
		t.Errorf("k: got %d, want 10 (YAML should override declared value)", p.Value.Int64)
	}
}
