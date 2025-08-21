package params

import (
	"reflect"
	"sync"
	"testing"
)

// Shared helpers for tests
func i64(v int64) *int64     { return &v }
func f64(v float64) *float64 { return &v }

// ---------- Value.String() ----------

func TestValue_String_Scalars(t *testing.T) {
	cases := []struct {
		name string
		v    Value
		want string
	}{
		{"notset", Value{Kind: KindNotSet}, "<not set>"},
		{"bool", Value{Kind: KindBool, Bool: true}, "true"},
		{"int64", Value{Kind: KindInt64, Int64: 42}, "42"},
		{"double", Value{Kind: KindDouble, Double: 3.14}, "3.14"},
		{"string", Value{Kind: KindString, Str: "hello"}, "hello"},
		{"bytes", Value{Kind: KindBytes, Bytes: []byte{1, 2, 3}}, "[3 bytes]"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			if got := tc.v.String(); got != tc.want {
				t.Fatalf("String() = %q, want %q", got, tc.want)
			}
		})
	}
}

func TestValue_String_Arrays(t *testing.T) {
	cases := []Value{
		{Kind: KindBoolArray, Bools: []bool{true, false}},
		{Kind: KindInt64Array, Int64s: []int64{1, 2, 3}},
		{Kind: KindDoubleArray, Doubles: []float64{1.5, 2.5}},
		{Kind: KindStringArray, Strs: []string{"a", "b"}},
	}
	for _, v := range cases {
		if got := v.String(); got == "" {
			t.Fatalf("String() empty for kind %d", v.Kind)
		}
	}
}

// ---------- store.declare / get / list ----------

func TestStore_DeclareGetList(t *testing.T) {
	s := newStore()

	p1, err := s.declare("camera.fps", Value{Kind: KindInt64, Int64: 15}, Descriptor{})
	if err != nil {
		t.Fatalf("declare p1: %v", err)
	}
	p2, err := s.declare("camera.frame_id", Value{Kind: KindString, Str: "camera"}, Descriptor{})
	if err != nil {
		t.Fatalf("declare p2: %v", err)
	}

	// duplicate should fail
	if _, err := s.declare("camera.fps", Value{Kind: KindInt64, Int64: 30}, Descriptor{}); err == nil {
		t.Fatalf("expected duplicate declare error")
	}

	// get
	if got, ok := s.get("camera.fps"); !ok || !reflect.DeepEqual(got, p1) {
		t.Fatalf("get p1 = (%v,%v), want (%v,true)", got, ok, p1)
	}
	if got, ok := s.get("camera.frame_id"); !ok || !reflect.DeepEqual(got, p2) {
		t.Fatalf("get p2 = (%v,%v), want (%v,true)", got, ok, p2)
	}

	// list contains both
	lst := s.list()
	if len(lst) != 2 {
		t.Fatalf("list len=%d, want 2", len(lst))
	}
}

// ---------- store.set (success, read-only, undeclared, validation) ----------

func TestStore_Set_SuccessAndEventOldNew(t *testing.T) {
	s := newStore()
	_, _ = s.declare("x", Value{Kind: KindInt64, Int64: 1}, Descriptor{MinInt: i64(0), MaxInt: i64(10)})

	res, old, newp := s.set([]Parameter{{Name: "x", Value: Value{Kind: KindInt64, Int64: 2}}})
	if !res.Successful || res.Reason != "" {
		t.Fatalf("set result = %+v, want success", res)
	}
	if len(old) != 1 || old[0].Value.Int64 != 1 {
		t.Fatalf("old = %+v, want previous value 1", old)
	}
	if len(newp) != 1 || newp[0].Value.Int64 != 2 {
		t.Fatalf("new = %+v, want new value 2", newp)
	}
}

func TestStore_Set_ReadOnlyRejected(t *testing.T) {
	s := newStore()
	_, _ = s.declare("ro", Value{Kind: KindString, Str: "fixed"}, Descriptor{ReadOnly: true})

	res, old, newp := s.set([]Parameter{{Name: "ro", Value: Value{Kind: KindString, Str: "change"}}})
	if res.Successful {
		t.Fatalf("expected read-only rejection, got success")
	}
	if old != nil || newp != nil {
		t.Fatalf("expected no old/new on rejection, got old=%v new=%v", old, newp)
	}
}

func TestStore_Set_UndeclaredRejected(t *testing.T) {
	s := newStore()
	res, old, newp := s.set([]Parameter{{Name: "nope", Value: Value{Kind: KindBool, Bool: true}}})
	if res.Successful {
		t.Fatalf("expected undeclared rejection")
	}
	if old != nil || newp != nil {
		t.Fatalf("expected nil old/new on rejection")
	}
}

func TestStore_Set_ValidationBounds(t *testing.T) {
	s := newStore()
	_, _ = s.declare("i", Value{Kind: KindInt64, Int64: 5}, Descriptor{MinInt: i64(1), MaxInt: i64(10)})
	_, _ = s.declare("d", Value{Kind: KindDouble, Double: 0.5}, Descriptor{MinDouble: f64(0.1), MaxDouble: f64(0.9)})
	_, _ = s.declare("s", Value{Kind: KindString, Str: "b"}, Descriptor{AllowedStrings: []string{"a", "b"}})

	// below min
	if res, _, _ := s.set([]Parameter{{Name: "i", Value: Value{Kind: KindInt64, Int64: 0}}}); res.Successful {
		t.Fatalf("expected int below-min rejection")
	}
	// above max
	if res, _, _ := s.set([]Parameter{{Name: "d", Value: Value{Kind: KindDouble, Double: 1.0}}}); res.Successful {
		t.Fatalf("expected double above-max rejection")
	}
	// not allowed string
	if res, _, _ := s.set([]Parameter{{Name: "s", Value: Value{Kind: KindString, Str: "z"}}}); res.Successful {
		t.Fatalf("expected string not-in-allowed-set rejection")
	}
	// valid changes pass
	if res, _, _ := s.set([]Parameter{
		{Name: "i", Value: Value{Kind: KindInt64, Int64: 6}},
		{Name: "d", Value: Value{Kind: KindDouble, Double: 0.7}},
		{Name: "s", Value: Value{Kind: KindString, Str: "a"}},
	}); !res.Successful {
		t.Fatalf("expected success for valid changes")
	}
}

// ---------- OnSet callback ----------

func TestStore_OnSetCallbackRejects(t *testing.T) {
	s := newStore()
	_, _ = s.declare("k", Value{Kind: KindInt64, Int64: 1}, Descriptor{})

	// Reject if value set to 13
	s.cb = func(ps []Parameter) SetResult {
		for _, p := range ps {
			if p.Name == "k" && p.Value.Kind == KindInt64 && p.Value.Int64 == 13 {
				return SetResult{Successful: false, Reason: "bad luck"}
			}
		}
		return SetResult{Successful: true}
	}

	// attempt rejected
	if res, old, newp := s.set([]Parameter{{Name: "k", Value: Value{Kind: KindInt64, Int64: 13}}}); res.Successful {
		t.Fatalf("expected rejection by callback")
	} else if old != nil || newp != nil {
		t.Fatalf("expected nil old/new on rejection")
	}

	// acceptable value applies
	if res, _, _ := s.set([]Parameter{{Name: "k", Value: Value{Kind: KindInt64, Int64: 7}}}); !res.Successful {
		t.Fatalf("expected success")
	}
	if got, _ := s.get("k"); got.Value.Int64 != 7 {
		t.Fatalf("value not applied, got %d", got.Value.Int64)
	}
}

// ---------- Concurrency sanity ----------

func TestStore_ConcurrentGetSet(t *testing.T) {
	s := newStore()
	_, _ = s.declare("n", Value{Kind: KindInt64, Int64: 0}, Descriptor{MinInt: i64(0), MaxInt: i64(1 << 30)})

	var wg sync.WaitGroup
	// Writer
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := int64(1); i <= 1000; i++ {
			res, _, _ := s.set([]Parameter{{Name: "n", Value: Value{Kind: KindInt64, Int64: i}}})
			if !res.Successful {
				t.Errorf("set failed at %d: %+v", i, res)
				return
			}
		}
	}()

	// Readers
	for r := 0; r < 4; r++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			var last int64 = -1
			for i := 0; i < 1000; i++ {
				if p, ok := s.get("n"); ok {
					if p.Value.Int64 < last {
						t.Errorf("monotonicity violated: %d < %d", p.Value.Int64, last)
						return
					}
					last = p.Value.Int64
				}
			}
		}()
	}

	wg.Wait()
}
