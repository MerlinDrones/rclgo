package params

import (
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
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
//
// mu guards the maps and cb. Changes (declare, set, override, undeclare) are
// also serialized by modMu, which is held while the on-set callback runs but
// mu is not: the callback can read parameters, and a change it attempts fails
// with errInCallback instead of deadlocking, as rclcpp rejects it.
type store struct {
	mu   sync.RWMutex
	vals map[string]Parameter
	desc map[string]Descriptor
	cb   OnSetCallback

	modMu   sync.Mutex
	cbOwner atomic.Int64 // goroutine running the on-set callback, or 0

	// overridden holds the parameters that exist only because an override
	// (parameter file, command line or environment) set them, and that code
	// has not declared yet. Declaring one attaches the code's descriptor and
	// type to the override's value.
	overridden map[string]bool

	// notify, if set, is told about every change after it is made, while
	// modMu is still held, so that notifications (parameter events and
	// post-set callbacks) are delivered in the order of the changes, as in
	// rclcpp, where they run under the parameters mutex.
	notify func(declared, changed, deleted []Parameter)
}

// errInCallback is returned for a change attempted from an on-set callback.
var errInCallback = errors.New("parameters cannot be declared, set or undeclared from an on-set callback")

// lockChanges takes modMu, or fails if the caller is the on-set callback,
// which runs with modMu held.
func (s *store) lockChanges() error {
	if s.modMu.TryLock() {
		return nil
	}
	if id := s.cbOwner.Load(); id != 0 && id == goroutineID() {
		return errInCallback
	}
	s.modMu.Lock()
	return nil
}

// runCallback runs the on-set callback, if any, for params. modMu must be
// held and mu not.
func (s *store) runCallback(params []Parameter) SetResult {
	s.mu.RLock()
	cb := s.cb
	s.mu.RUnlock()
	if cb == nil {
		return SetResult{Successful: true}
	}
	s.cbOwner.Store(goroutineID())
	defer s.cbOwner.Store(0)
	return cb(params)
}

// notifyChange calls notify for a change; modMu must be held and mu not. A
// change attempted from the notification fails with errInCallback, as rclcpp
// rejects changes from post-set callbacks.
func (s *store) notifyChange(declared, changed, deleted []Parameter) {
	if s.notify == nil || len(declared)+len(changed)+len(deleted) == 0 {
		return
	}
	prev := s.cbOwner.Load()
	s.cbOwner.Store(goroutineID())
	defer s.cbOwner.Store(prev)
	s.notify(declared, changed, deleted)
}

func (s *store) setCallback(cb OnSetCallback) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.cb = cb
}

func newStore() *store {
	return &store{vals: map[string]Parameter{}, desc: map[string]Descriptor{}, overridden: map[string]bool{}}
}

// declare declares a parameter; see declareOrClaim.
func (s *store) declare(name string, v Value, d Descriptor) (Parameter, error) {
	p, _, err := s.declareOrClaim(name, v, d)
	return p, err
}

// declareOrClaim declares a parameter, as rclcpp's declare_parameter: the
// value (v, or an override's value if an override already created the
// parameter, converted to v's type; claimed then reports true) must satisfy d
// and the on-set callback, and is then reported as a new parameter (or, for a
// claimed override, a changed one).
func (s *store) declareOrClaim(name string, v Value, d Descriptor) (p Parameter, claimed bool, err error) {
	if err := s.lockChanges(); err != nil {
		return Parameter{}, false, err
	}
	defer s.modMu.Unlock()
	s.mu.RLock()
	cur, exists := s.vals[name]
	claimed = exists && s.overridden[name]
	s.mu.RUnlock()
	if exists && !claimed {
		return Parameter{}, false, fmt.Errorf("parameter %q already declared", name)
	}
	if claimed {
		// A declaration that fails leaves the parameter undeclared, as in
		// rclcpp: the override is dropped instead of staying settable with
		// no descriptor.
		defer func() {
			if err != nil {
				s.dropOverride(name, cur)
			}
		}()
	}
	what := fmt.Sprintf("default %v", v)
	p = Parameter{Name: name, Value: v}
	if claimed {
		what = fmt.Sprintf("override %v", cur.Value)
		val, ok := coerce(cur.Value, v.Kind)
		if !ok {
			return Parameter{}, false, fmt.Errorf("parameter %q: override of type %s cannot be used for a %s parameter",
				name, kindName(cur.Value.Kind), kindName(v.Kind))
		}
		p.Value = val
	}
	if res := validate(p, d); !res.Successful {
		return Parameter{}, false, fmt.Errorf("parameter %q: %s rejected: %s", name, what, res.Reason)
	}
	if res := s.runCallback([]Parameter{p}); !res.Successful {
		return Parameter{}, false, fmt.Errorf("parameter %q: %s rejected by the on-set callback: %s", name, what, res.Reason)
	}
	s.mu.Lock()
	s.desc[name] = d
	s.vals[name] = p
	delete(s.overridden, name)
	s.mu.Unlock()
	if claimed {
		s.notifyChange(nil, []Parameter{p}, nil)
	} else {
		s.notifyChange([]Parameter{p}, nil, nil)
	}
	return p, claimed, nil
}

// applyOverride sets a parameter from an override. An undeclared parameter is
// created with descriptor d and stays open to a later declaration (created
// reports true). A declared parameter keeps its descriptor and type; read-only
// does not stop an override, but the other constraints and the OnSet callback
// apply.
func (s *store) applyOverride(name string, v Value, d Descriptor) (p Parameter, created bool, err error) {
	if err := s.lockChanges(); err != nil {
		return Parameter{}, false, err
	}
	defer s.modMu.Unlock()
	s.mu.Lock()
	cur, exists := s.vals[name]
	if !exists {
		s.desc[name] = d
		p = Parameter{Name: name, Value: v}
		s.vals[name] = p
		s.overridden[name] = true
		s.mu.Unlock()
		s.notifyChange([]Parameter{p}, nil, nil)
		return p, true, nil
	}
	if s.overridden[name] {
		p = Parameter{Name: name, Value: v}
		s.vals[name] = p
		s.mu.Unlock()
		s.notifyChange(nil, []Parameter{p}, nil)
		return p, false, nil
	}
	desc := s.desc[name]
	s.mu.Unlock()
	val, ok := coerce(v, cur.Value.Kind)
	if !ok {
		return Parameter{}, false, fmt.Errorf("parameter %q: override of type %s cannot be used for a %s parameter",
			name, kindName(v.Kind), kindName(cur.Value.Kind))
	}
	p = Parameter{Name: name, Value: val}
	if res := validate(p, desc); !res.Successful {
		return Parameter{}, false, fmt.Errorf("parameter %q: override %v rejected: %s", name, v, res.Reason)
	}
	if res := s.runCallback([]Parameter{p}); !res.Successful {
		return Parameter{}, false, fmt.Errorf("parameter %q: override %v rejected: %s", name, v, res.Reason)
	}
	s.mu.Lock()
	s.vals[name] = p
	s.mu.Unlock()
	s.notifyChange(nil, []Parameter{p}, nil)
	return p, false, nil
}

func (s *store) descriptor(name string) (Descriptor, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	d, ok := s.desc[name]
	return d, ok
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
	if err := s.lockChanges(); err != nil {
		return SetResult{false, err.Error()}, nil, nil
	}
	defer s.modMu.Unlock()
	// Nothing else changes the store while modMu is held, so what is
	// validated under the read lock is still current when it is applied.
	s.mu.RLock()
	res, coerced := s.validateLocked(params)
	s.mu.RUnlock()
	if !res.Successful {
		return res, nil, nil
	}
	if res := s.runCallback(coerced); !res.Successful {
		return res, nil, nil
	}
	s.mu.Lock()
	old := make([]Parameter, 0, len(coerced))
	newp := make([]Parameter, 0, len(coerced))
	for _, np := range coerced {
		old = append(old, s.vals[np.Name])
		s.vals[np.Name] = np
		newp = append(newp, np)
	}
	s.mu.Unlock()
	s.notifyChange(nil, newp, nil)
	return SetResult{true, ""}, old, newp
}

// validateLocked checks params against their descriptors and converts their
// values to the declared types. mu must be held.
func (s *store) validateLocked(params []Parameter) (SetResult, []Parameter) {

	// descriptor / read-only validation
	// A parameter declared in code keeps its type; integers widen to
	// doubles. Parameters that exist only through an override take any type.
	coerced := make([]Parameter, len(params))
	for i, np := range params {
		d, ok := s.desc[np.Name]
		if !ok {
			return SetResult{false, "undeclared parameter: " + np.Name}, nil
		}
		if d.ReadOnly {
			return SetResult{false, "parameter is read-only: " + np.Name}, nil
		}
		if !s.overridden[np.Name] {
			cur := s.vals[np.Name].Value
			v, ok := coerce(np.Value, cur.Kind)
			if !ok {
				return SetResult{false, fmt.Sprintf("wrong type for %s: got %s, want %s",
					np.Name, kindName(np.Value.Kind), kindName(cur.Kind))}, nil
			}
			np.Value = v
		}
		if res := validate(np, d); !res.Successful {
			return res, nil
		}
		coerced[i] = np
	}
	return SetResult{true, ""}, coerced
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

// undeclare removes a parameter, as rclcpp's undeclare_parameter: a parameter
// that isn't declared, or is read-only, is refused.
// dropOverride removes the override-only parameter name, whose value was cur,
// and reports it as deleted. s.modMu must be held.
func (s *store) dropOverride(name string, cur Parameter) {
	s.mu.Lock()
	if !s.overridden[name] {
		s.mu.Unlock()
		return
	}
	delete(s.vals, name)
	delete(s.desc, name)
	delete(s.overridden, name)
	s.mu.Unlock()
	s.notifyChange(nil, nil, []Parameter{cur})
}

func (s *store) undeclare(name string) error {
	if err := s.lockChanges(); err != nil {
		return err
	}
	defer s.modMu.Unlock()
	s.mu.Lock()
	p, ok := s.vals[name]
	if !ok {
		s.mu.Unlock()
		return fmt.Errorf("parameter %q is not declared", name)
	}
	if s.desc[name].ReadOnly {
		s.mu.Unlock()
		return fmt.Errorf("parameter %q is read-only", name)
	}
	delete(s.vals, name)
	delete(s.desc, name)
	delete(s.overridden, name)
	s.mu.Unlock()
	s.notifyChange(nil, nil, []Parameter{p})
	return nil
}
