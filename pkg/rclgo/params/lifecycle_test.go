/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package params

import (
	"fmt"
	"strings"
	"sync"
	"testing"

	rcl_interfaces_msg "github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/msg"
)

// After an override-only parameter is undeclared, declaring it again in code
// makes later overrides subject to its type and descriptor again.
func TestUndeclaredOverrideRedeclaredIsTypeChecked(t *testing.T) {
	m := newBareManager()
	if err := m.applyOverride("x", Value{Kind: KindString, Str: "s"}, Descriptor{}); err != nil {
		t.Fatal(err)
	}
	if err := m.Undeclare("x"); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Declare("x", Value{Kind: KindInt64, Int64: 1}, Descriptor{}); err != nil {
		t.Fatal(err)
	}
	if err := m.applyOverride("x", Value{Kind: KindString, Str: "not an int"}, Descriptor{}); err == nil {
		t.Fatal("a string override was accepted for a declared integer parameter")
	}
	if p, _ := m.Get("x"); p.Value.Kind != KindInt64 {
		t.Fatalf("x changed type to %s", kindName(p.Value.Kind))
	}
}

// As rclcpp's undeclare_parameter, Undeclare refuses read-only and unknown
// parameters, and use_sim_time, which the node's clock follows.
func TestUndeclareRefusals(t *testing.T) {
	m := newBareManager()
	if _, err := m.Declare("fixed", Value{Kind: KindInt64, Int64: 1}, Descriptor{ReadOnly: true}); err != nil {
		t.Fatal(err)
	}
	if err := m.Undeclare("fixed"); err == nil || !strings.Contains(err.Error(), "read-only") {
		t.Errorf("undeclaring a read-only parameter: %v", err)
	}
	if _, ok := m.Get("fixed"); !ok {
		t.Error("the read-only parameter was removed")
	}
	if err := m.Undeclare("missing"); err == nil {
		t.Error("undeclaring an unknown parameter succeeded")
	}
	if _, err := m.Declare("use_sim_time", Value{Kind: KindBool}, Descriptor{}); err != nil {
		t.Fatal(err)
	}
	if err := m.Undeclare("use_sim_time"); err == nil {
		t.Error("use_sim_time was undeclared")
	}
}

// As rclcpp's declare_parameter, Declare checks the default against the
// descriptor and the on-set callback.
func TestDeclareValidatesDefault(t *testing.T) {
	m := newBareManager()
	if _, err := m.Declare("rate", Value{Kind: KindInt64, Int64: 500}, Descriptor{MaxInt: i64(100)}); err == nil {
		t.Error("a default above the declared maximum was accepted")
	}
	if _, ok := m.Get("rate"); ok {
		t.Error("the refused parameter was declared")
	}
	m.OnSet(func(ps []Parameter) SetResult {
		return SetResult{Successful: ps[0].Value.Int64 >= 0, Reason: "negative"}
	})
	if _, err := m.Declare("count", Value{Kind: KindInt64, Int64: -1}, Descriptor{}); err == nil {
		t.Error("a default refused by the on-set callback was accepted")
	}
	if _, err := m.Declare("count", Value{Kind: KindInt64, Int64: 3}, Descriptor{}); err != nil {
		t.Errorf("a valid default was refused: %v", err)
	}
}

// Declaring a parameter that an override created publishes its value, now
// typed by the declaration.
func TestClaimingOverridePublishesEvent(t *testing.T) {
	m := newBareManager()
	var events []*rcl_interfaces_msg.ParameterEvent
	m.eventSink = func(e *rcl_interfaces_msg.ParameterEvent) { events = append(events, e) }
	if err := m.applyOverride("gain", Value{Kind: KindInt64, Int64: 2}, Descriptor{}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Declare("gain", Value{Kind: KindDouble, Double: 0.5}, Descriptor{}); err != nil {
		t.Fatal(err)
	}
	if len(events) != 2 || len(events[1].ChangedParameters) != 1 {
		t.Fatalf("got %d events (%+v), want the override's new_parameters and the declaration's changed_parameters", len(events), events)
	}
	if v := events[1].ChangedParameters[0].Value; v.Type != KindDouble || v.DoubleValue != 2 {
		t.Errorf("claimed value %+v, want the double 2", v)
	}
}

// Post-set callbacks see changes in the order they were made, so the last
// notification matches the final value even under concurrent sets.
func TestPostSetCallbacksInChangeOrder(t *testing.T) {
	for round := 0; round < 20; round++ {
		m := newBareManager()
		if _, err := m.Declare("flag", Value{Kind: KindBool}, Descriptor{}); err != nil {
			t.Fatal(err)
		}
		var (
			mu   sync.Mutex
			last bool
		)
		m.AddPostSetCallback(func(ps []Parameter) {
			mu.Lock()
			last = ps[0].Value.Bool
			mu.Unlock()
		})
		var wg sync.WaitGroup
		for g := 0; g < 8; g++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				for i := 0; i < 100; i++ {
					_, _ = m.SetValue("flag", Value{Kind: KindBool, Bool: (i+g)%2 == 0})
				}
			}()
		}
		wg.Wait()
		p, _ := m.Get("flag")
		if last != p.Value.Bool {
			t.Fatalf("round %d: last post-set notification %v, final value %v", round, last, p.Value.Bool)
		}
	}
}

// Post-set callbacks run in the order they were registered, and removing one
// keeps the order of the others (rclcpp keeps a list).
func TestPostSetCallbacksInRegistrationOrder(t *testing.T) {
	m := newBareManager()
	if _, err := m.Declare("x", Value{Kind: KindInt64}, Descriptor{}); err != nil {
		t.Fatal(err)
	}
	var calls []int
	var removes []func()
	for i := 0; i < 20; i++ {
		removes = append(removes, m.AddPostSetCallback(func([]Parameter) { calls = append(calls, i) }))
	}
	if _, err := m.SetValue("x", Value{Kind: KindInt64, Int64: 1}); err != nil {
		t.Fatal(err)
	}
	for i, c := range calls {
		if c != i {
			t.Fatalf("callbacks ran in order %v", calls)
		}
	}
	removes[3]()
	removes[10]()
	calls = nil
	if _, err := m.SetValue("x", Value{Kind: KindInt64, Int64: 2}); err != nil {
		t.Fatal(err)
	}
	want := []int{0, 1, 2, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18, 19}
	if fmt.Sprint(calls) != fmt.Sprint(want) {
		t.Fatalf("after removals callbacks ran in order %v, want %v", calls, want)
	}
}
