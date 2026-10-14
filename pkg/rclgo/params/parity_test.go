/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package params

import (
	"errors"
	"testing"

	"github.com/merlindrones/rclgo/pkg/rclgo"
)

// Parameter-file and override details that follow rcl/rclcpp.

// An empty list (`[]`) in a parameter file has no element type; it fits any
// array declaration.
func TestYAMLEmptyListFitsAnyArray(t *testing.T) {
	path := writeTemp(t, t.TempDir(), "params.yaml", "node:\n  ros__parameters:\n    ids: []\n    names: []\n")
	m := newBareManager()
	if err := LoadYAML(m, "node", path); err != nil {
		t.Fatal(err)
	}
	p, err := m.Declare("ids", Value{Kind: KindInt64Array, Int64s: []int64{1}}, Descriptor{})
	if err != nil {
		t.Fatalf("Declare(int64 array) of an empty YAML list: %v", err)
	}
	if p.Value.Kind != KindInt64Array || len(p.Value.Int64s) != 0 {
		t.Errorf("ids = %+v, want an empty integer array", p.Value)
	}
	if _, err := m.Declare("names", Value{Kind: KindStringArray}, Descriptor{}); err != nil {
		t.Fatalf("Declare(string array): %v", err)
	}
}

// A list of numbers with a float anywhere is a double array, whatever the
// order (it was rejected when an integer came first).
func TestYAMLMixedNumberListIsDoubleArray(t *testing.T) {
	path := writeTemp(t, t.TempDir(), "params.yaml", "node:\n  ros__parameters:\n    a: [1, 2.5]\n    b: [2.5, 1]\n    c: [1, 2]\n")
	m := newBareManager()
	if err := LoadYAML(m, "node", path); err != nil {
		t.Fatal(err)
	}
	for name, want := range map[string][]float64{"a": {1, 2.5}, "b": {2.5, 1}} {
		p, _ := m.Get(name)
		if p.Value.Kind != KindDoubleArray || len(p.Value.Doubles) != 2 || p.Value.Doubles[0] != want[0] || p.Value.Doubles[1] != want[1] {
			t.Errorf("%s = %+v, want double array %v", name, p.Value, want)
		}
	}
	if p, _ := m.Get("c"); p.Value.Kind != KindInt64Array {
		t.Errorf("c = %+v, want an integer array", p.Value)
	}
}

// A file without a section for the node is reported with ErrNoMatchingSection.
func TestYAMLNoMatchingSection(t *testing.T) {
	path := writeTemp(t, t.TempDir(), "params.yaml", "other:\n  ros__parameters:\n    x: 1\n")
	err := LoadYAML(newBareManager(), "node", path)
	if !errors.Is(err, ErrNoMatchingSection) {
		t.Fatalf("LoadYAML: %v, want ErrNoMatchingSection", err)
	}
}

// A remapped node (__node:=, __ns:=) takes the sections for its remapped
// fully qualified name only, not those for the name the code passes.
func TestOverridesForRemappedNode(t *testing.T) {
	args, _, err := rclgo.ParseArgs([]string{"--ros-args", "-r", "__node:=listener", "-r", "__ns:=/remapped", "--"})
	if err != nil {
		t.Fatal(err)
	}
	ctx, err := rclgo.NewContext(0, args)
	if err != nil {
		t.Fatal(err)
	}
	defer ctx.Close()
	n, err := ctx.NewNode("talker", "")
	if err != nil {
		t.Fatal(err)
	}
	if got := n.FullyQualifiedName(); got != "/remapped/listener" {
		t.Fatalf("remapped name %q", got)
	}
	path := writeTemp(t, t.TempDir(), "params.yaml", `
talker:
  ros__parameters:
    from_original_name: 1
/remapped/listener:
  ros__parameters:
    from_remapped_name: 2
`)
	m := newBareManagerFor(n)
	if err := LoadYAML(m, "talker", path); err != nil {
		t.Fatal(err)
	}
	if _, ok := m.Get("from_remapped_name"); !ok {
		t.Error("the section for the remapped name was not applied")
	}
	if _, ok := m.Get("from_original_name"); ok {
		t.Error("the section for the name in the code was applied to the remapped node")
	}
}

// Several --params-file arguments apply in order: a later file wins.
func TestOverridesFromSeveralParamsFiles(t *testing.T) {
	dir := t.TempDir()
	first := writeTemp(t, dir, "first.yaml", "/**:\n  ros__parameters:\n    x: 1\n    only_first: true\n")
	second := writeTemp(t, dir, "second.yaml", "/**:\n  ros__parameters:\n    x: 2\n")
	args, _, err := rclgo.ParseArgs([]string{"--ros-args", "--params-file", first, "--params-file", second, "--"})
	if err != nil {
		t.Fatal(err)
	}
	m := newBareManager()
	if err := ApplyOverrides(m, "node", args); err != nil {
		t.Fatal(err)
	}
	if p, _ := m.Get("x"); p.Value.Int64 != 2 {
		t.Errorf("x = %+v, want 2 from the later file", p.Value)
	}
	if p, ok := m.Get("only_first"); !ok || !p.Value.Bool {
		t.Errorf("only_first = %+v (present %v), want true from the first file", p.Value, ok)
	}
}

// An RCL_PARAM_ environment variable for a parameter declared without a type
// (not set) takes the type its value has, instead of failing.
func TestEnvVarForNotSetParameter(t *testing.T) {
	m := newBareManager()
	if _, err := m.Declare("anything", Value{Kind: KindNotSet}, Descriptor{}); err != nil {
		t.Fatal(err)
	}
	t.Setenv("RCL_PARAM_anything", "5")
	if err := ApplyEnvVars(m); err != nil {
		t.Fatalf("ApplyEnvVars: %v", err)
	}
	if p, _ := m.Get("anything"); p.Value.Kind != KindInt64 || p.Value.Int64 != 5 {
		t.Errorf("anything = %+v, want integer 5", p.Value)
	}
}
