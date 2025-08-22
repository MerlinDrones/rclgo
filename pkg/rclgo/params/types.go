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
}

func newStore() *store {
	return &store{vals: map[string]Parameter{}, desc: map[string]Descriptor{}}
}

func (s *store) declare(name string, v Value, d Descriptor) (Parameter, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if _, exists := s.vals[name]; exists {
		return Parameter{}, fmt.Errorf("parameter %q already declared", name)
	}
	s.desc[name] = d
	p := Parameter{Name: name, Value: v}
	s.vals[name] = p
	return p, nil
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
	for _, np := range params {
		d, ok := s.desc[np.Name]
		if !ok {
			return SetResult{false, "undeclared parameter: " + np.Name}, nil, nil
		}
		if d.ReadOnly {
			return SetResult{false, "parameter is read-only: " + np.Name}, nil, nil
		}
		if res := validate(np, d); !res.Successful {
			return res, nil, nil
		}
	}

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
