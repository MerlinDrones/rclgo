package params

import (
	"fmt"
	"sync"
	"time"
)

// Kind mirrors rcl_interfaces/msg/ParameterType (0..9 in Humble)
const (
	KindNotSet      = 0
	KindBool        = 1
	KindInt64       = 2
	KindDouble      = 3
	KindString      = 4
	KindBytes       = 5
	KindBoolArray   = 6
	KindInt64Array  = 7
	KindDoubleArray = 8
	KindStringArray = 9
)

type Value struct {
	Kind   uint8
	Bool   bool
	Int64  int64
	Double float64
	Str    string
	Bytes  []byte

	Bools   []bool
	Int64s  []int64
	Doubles []float64
	Strs    []string
}

func (v Value) String() string {
	switch v.Kind {
	case KindNotSet:
		return "<not set>"
	case KindBool:
		return fmt.Sprintf("%v", v.Bool)
	case KindInt64:
		return fmt.Sprintf("%d", v.Int64)
	case KindDouble:
		return fmt.Sprintf("%g", v.Double)
	case KindString:
		return v.Str
	case KindBytes:
		return fmt.Sprintf("[%d bytes]", len(v.Bytes))
	case KindBoolArray:
		return fmt.Sprintf("%v", v.Bools)
	case KindInt64Array:
		return fmt.Sprintf("%v", v.Int64s)
	case KindDoubleArray:
		return fmt.Sprintf("%v", v.Doubles)
	case KindStringArray:
		return fmt.Sprintf("%v", v.Strs)
	default:
		return "<??>"
	}
}

type Descriptor struct {
	Name                 string
	Description          string
	ReadOnly             bool
	MinInt, MaxInt       *int64
	MinDouble, MaxDouble *float64
	AllowedStrings       []string
}

type Parameter struct {
	Name  string
	Value Value
}

type SetResult struct {
	Successful bool
	Reason     string
}

type OnSetCallback func([]Parameter) SetResult

type Event struct {
	Stamp time.Time
	New   []Parameter
	Old   []Parameter
	// Deleted are published from Manager; not stored here.
}

// thread-safe in-memory store
type store struct {
	mu   sync.RWMutex
	vals map[string]Parameter
	desc map[string]Descriptor
	cb   OnSetCallback

	// overridden holds the parameters that exist only because an override
	// (parameter file, command line or environment) set them, and that code
	// has not declared yet. Declaring one attaches the code's descriptor and
	// type to the override's value.
	overridden map[string]bool
}

func newStore() *store {
	return &store{vals: map[string]Parameter{}, desc: map[string]Descriptor{}, overridden: map[string]bool{}}
}

// declare declares a parameter; see declareOrClaim.
func (s *store) declare(name string, v Value, d Descriptor) (Parameter, error) {
	p, _, err := s.declareOrClaim(name, v, d)
	return p, err
}

// declareOrClaim declares a parameter. If an override already created it, the
// override's value is kept, converted to the type of v and checked against d,
// and claimed reports true.
func (s *store) declareOrClaim(name string, v Value, d Descriptor) (p Parameter, claimed bool, err error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if _, exists := s.vals[name]; exists {
		if !s.overridden[name] {
			return Parameter{}, false, fmt.Errorf("parameter %q already declared", name)
		}
		override := s.vals[name].Value
		val, ok := coerce(override, v.Kind)
		if !ok {
			return Parameter{}, false, fmt.Errorf("parameter %q: override of type %s cannot be used for a %s parameter",
				name, kindName(override.Kind), kindName(v.Kind))
		}
		p = Parameter{Name: name, Value: val}
		if res := validate(p, d); !res.Successful {
			return Parameter{}, false, fmt.Errorf("parameter %q: override %v rejected: %s", name, override, res.Reason)
		}
		s.desc[name] = d
		s.vals[name] = p
		delete(s.overridden, name)
		return p, true, nil
	}
	s.desc[name] = d
	p = Parameter{Name: name, Value: v}
	s.vals[name] = p
	return p, false, nil
}

// applyOverride sets a parameter from an override. An undeclared parameter is
// created with descriptor d and stays open to a later declaration (created
// reports true). A declared parameter keeps its descriptor and type; read-only
// does not stop an override, but the other constraints and the OnSet callback
// apply.
func (s *store) applyOverride(name string, v Value, d Descriptor) (p Parameter, created bool, err error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	cur, exists := s.vals[name]
	if !exists {
		s.desc[name] = d
		p = Parameter{Name: name, Value: v}
		s.vals[name] = p
		s.overridden[name] = true
		return p, true, nil
	}
	if s.overridden[name] {
		p = Parameter{Name: name, Value: v}
		s.vals[name] = p
		return p, false, nil
	}
	val, ok := coerce(v, cur.Value.Kind)
	if !ok {
		return Parameter{}, false, fmt.Errorf("parameter %q: override of type %s cannot be used for a %s parameter",
			name, kindName(v.Kind), kindName(cur.Value.Kind))
	}
	p = Parameter{Name: name, Value: val}
	if res := validate(p, s.desc[name]); !res.Successful {
		return Parameter{}, false, fmt.Errorf("parameter %q: override %v rejected: %s", name, v, res.Reason)
	}
	if s.cb != nil {
		if res := s.cb([]Parameter{p}); !res.Successful {
			return Parameter{}, false, fmt.Errorf("parameter %q: override %v rejected: %s", name, v, res.Reason)
		}
	}
	s.vals[name] = p
	return p, false, nil
}

func (s *store) isOverrideOnly(name string) bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.overridden[name]
}

// coerce converts v to kind. Values of the same kind, and any value for
// KindNotSet, are returned as they are; integers widen to doubles. Other
// conversions fail.
func coerce(v Value, kind uint8) (Value, bool) {
	switch {
	case kind == KindNotSet || v.Kind == kind:
		return v, true
	case kind == KindDouble && v.Kind == KindInt64:
		return Value{Kind: KindDouble, Double: float64(v.Int64)}, true
	case kind == KindDoubleArray && v.Kind == KindInt64Array:
		ds := make([]float64, len(v.Int64s))
		for i, x := range v.Int64s {
			ds[i] = float64(x)
		}
		return Value{Kind: KindDoubleArray, Doubles: ds}, true
	}
	return v, false
}

func kindName(kind uint8) string {
	switch kind {
	case KindNotSet:
		return "not set"
	case KindBool:
		return "bool"
	case KindInt64:
		return "integer"
	case KindDouble:
		return "double"
	case KindString:
		return "string"
	case KindBytes:
		return "byte array"
	case KindBoolArray:
		return "bool array"
	case KindInt64Array:
		return "integer array"
	case KindDoubleArray:
		return "double array"
	case KindStringArray:
		return "string array"
	}
	return fmt.Sprintf("kind %d", kind)
}

func (s *store) get(name string) (Parameter, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	p, ok := s.vals[name]
	return p, ok
}

func (s *store) list() []Parameter {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]Parameter, 0, len(s.vals))
	for _, p := range s.vals {
		out = append(out, p)
	}
	return out
}

func (s *store) set(params []Parameter) (SetResult, []Parameter, []Parameter) {
	s.mu.Lock()
	defer s.mu.Unlock()

	old := make([]Parameter, 0, len(params))
	newp := make([]Parameter, 0, len(params))

	// descriptor / read-only validation
	// A parameter declared in code keeps its type; integers widen to
	// doubles. Parameters that exist only through an override take any type.
	coerced := make([]Parameter, len(params))
	for i, np := range params {
		d, ok := s.desc[np.Name]
		if !ok {
			return SetResult{false, "undeclared parameter: " + np.Name}, nil, nil
		}
		if d.ReadOnly {
			return SetResult{false, "parameter is read-only: " + np.Name}, nil, nil
		}
		if !s.overridden[np.Name] {
			cur := s.vals[np.Name].Value
			v, ok := coerce(np.Value, cur.Kind)
			if !ok {
				return SetResult{false, fmt.Sprintf("wrong type for %s: got %s, want %s",
					np.Name, kindName(np.Value.Kind), kindName(cur.Kind))}, nil, nil
			}
			np.Value = v
		}
		if res := validate(np, d); !res.Successful {
			return res, nil, nil
		}
		coerced[i] = np
	}
	params = coerced

	// preview callback
	if s.cb != nil {
		if res := s.cb(params); !res.Successful {
			return res, nil, nil
		}
	}

	// apply
	for _, np := range params {
		op := s.vals[np.Name]
		old = append(old, op)
		s.vals[np.Name] = np
		newp = append(newp, np)
	}
	return SetResult{true, ""}, old, newp
}

func validate(p Parameter, d Descriptor) SetResult {
	switch p.Value.Kind {
	case KindInt64:
		if d.MinInt != nil && p.Value.Int64 < *d.MinInt {
			return SetResult{false, "below min"}
		}
		if d.MaxInt != nil && p.Value.Int64 > *d.MaxInt {
			return SetResult{false, "above max"}
		}
	case KindDouble:
		if d.MinDouble != nil && p.Value.Double < *d.MinDouble {
			return SetResult{false, "below min"}
		}
		if d.MaxDouble != nil && p.Value.Double > *d.MaxDouble {
			return SetResult{false, "above max"}
		}
	case KindString:
		if len(d.AllowedStrings) > 0 {
			ok := false
			for _, s := range d.AllowedStrings {
				if s == p.Value.Str {
					ok = true
					break
				}
			}
			if !ok {
				return SetResult{false, "not in allowed set"}
			}
		}
	}
	return SetResult{true, ""}
}

// --- Undeclare ---
// removes a parameter definition and its descriptor if present.
// returns the removed Parameter and true if it existed.
func (s *store) undeclare(name string) (Parameter, bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	p, ok := s.vals[name]
	if !ok {
		return Parameter{}, false
	}
	delete(s.vals, name)
	delete(s.desc, name)
	return p, true
}
