package params

import (
	"fmt"
	"math"
	"slices"
	"strings"
	"sync"
	"time"

	"github.com/merlindrones/rclgo/pkg/msgs/builtin_interfaces/msg"
	rcl_interfaces_msg2 "github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/msg"
	rcl_interfaces_srv2 "github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

type Manager struct {
	st  *store
	pub *rclgo.Publisher // /parameter_events
	n   *rclgo.Node

	// Optional provider for simulated time. If nil, wall clock is used.
	// When 'use_sim_time' is true and nowProvider != nil, event stamps use nowProvider().
	nowProvider func() builtin_interfaces_msg.Time

	// eventSink, if set, receives every parameter event (for tests).
	eventSink func(*rcl_interfaces_msg2.ParameterEvent)

	postSetMu   sync.Mutex
	postSet     map[uint64]func([]Parameter)
	postSetNext uint64
}

// AddPostSetCallback registers f to be called after parameters were changed,
// with their new values: by Set or SetValue, through the set_parameters and
// set_parameters_atomically services, or by an override of a declared
// parameter. Unlike OnSet, which validates and can refuse a change, f only
// observes it, and any number of post-set callbacks can be registered (as
// rclcpp's add_post_set_parameters_callback). It returns a function that
// removes f.
func (m *Manager) AddPostSetCallback(f func([]Parameter)) (remove func()) {
	m.postSetMu.Lock()
	defer m.postSetMu.Unlock()
	if m.postSet == nil {
		m.postSet = make(map[uint64]func([]Parameter))
	}
	id := m.postSetNext
	m.postSetNext++
	m.postSet[id] = f
	return func() {
		m.postSetMu.Lock()
		defer m.postSetMu.Unlock()
		delete(m.postSet, id)
	}
}

func (m *Manager) runPostSet(changed []Parameter) {
	m.postSetMu.Lock()
	callbacks := make([]func([]Parameter), 0, len(m.postSet))
	for _, f := range m.postSet {
		callbacks = append(callbacks, f)
	}
	m.postSetMu.Unlock()
	for _, f := range callbacks {
		f(changed)
	}
}

func NewManager(n *rclgo.Node) (*Manager, error) {
	m := &Manager{st: newStore(), n: n}

	// /parameter_events publisher, with the same QoS as rclcpp and rclpy
	opts := &rclgo.PublisherOptions{Qos: qos.NewParameterEventsProfile()}
	pub, err := n.NewPublisher("/parameter_events", rcl_interfaces_msg2.ParameterEventTypeSupport, opts)
	if err != nil {
		return nil, err
	}
	m.pub = pub

	// Ensure 'use_sim_time' exists by default (ROS 2 convention; default=false).
	// This makes the parameter visible to tools (ros2 param) immediately.
	// Descriptor is not read-only; users/simulators can set it at runtime.
	_, _ = m.st.declare("use_sim_time", Value{Kind: KindBool, Bool: false}, Descriptor{
		Description: "Use simulated time from /clock when true.",
	})

	// Node-scoped parameter services, the same six as rclcpp and rclpy. rclpy's
	// parameter client (and so the ros2 param tool) waits for all of them.
	base := n.FullyQualifiedName()
	for _, svc := range []struct {
		name    string
		ts      types.ServiceTypeSupport
		handler rclgo.ServiceRequestHandler
	}{
		{"get_parameters", rcl_interfaces_srv2.GetParametersTypeSupport, m.onGetParameters},
		{"get_parameter_types", rcl_interfaces_srv2.GetParameterTypesTypeSupport, m.onGetParameterTypes},
		{"set_parameters", rcl_interfaces_srv2.SetParametersTypeSupport, m.onSetParameters},
		{"set_parameters_atomically", rcl_interfaces_srv2.SetParametersAtomicallyTypeSupport, m.onSetParametersAtomically},
		{"list_parameters", rcl_interfaces_srv2.ListParametersTypeSupport, m.onListParameters},
		{"describe_parameters", rcl_interfaces_srv2.DescribeParametersTypeSupport, m.onDescribeParameters},
	} {
		name := strings.TrimSuffix(base, "/") + "/" + svc.name
		if _, err := n.NewService(name, svc.ts, nil, svc.handler); err != nil {
			return nil, fmt.Errorf("parameter service %s: %w", name, err)
		}
		n.Logger().Debugf("parameter service registered: %s", name) //nolint:errcheck
	}

	return m, nil
}

// Public API

// Declare declares a parameter with default value v and descriptor d.
//
// If an override (parameter file, command line or environment variable)
// already set the parameter, Declare keeps the override's value instead of v,
// converted to v's type (an integer widens to a double), and attaches d. It
// returns an error if the override has an incompatible type or violates d.
func (m *Manager) Declare(name string, v Value, d Descriptor) (Parameter, error) {
	p, claimed, err := m.st.declareOrClaim(name, v, d)
	if err == nil && !claimed {
		m.publishEvent([]Parameter{p}, nil)
	}
	return p, err
}

// DeclareIfMissing is like Declare but does nothing if code already declared
// the parameter.
func (m *Manager) DeclareIfMissing(name string, v Value, d Descriptor) error {
	if _, ok := m.Get(name); ok && !m.st.isOverrideOnly(name) {
		return nil
	}
	_, err := m.Declare(name, v, d)
	return err
}

// applyOverride sets a parameter from an override source. See store.applyOverride.
func (m *Manager) applyOverride(name string, v Value, d Descriptor) error {
	p, created, err := m.st.applyOverride(name, v, d)
	if err != nil {
		return err
	}
	if created {
		m.publishEvent([]Parameter{p}, nil)
	} else {
		m.publishEvent(nil, []Parameter{p})
	}
	return nil
}

func (m *Manager) Get(name string) (Parameter, bool) { return m.st.get(name) }
func (m *Manager) List() []Parameter                 { return m.st.list() }
func (m *Manager) OnSet(cb OnSetCallback)            { m.st.cb = cb }

// Set updates one or more existing parameters. Returns SetResult indicating success/failure.
// All parameters must already be declared, and constraints will be validated.
func (m *Manager) Set(params ...Parameter) (SetResult, error) {
	result, _, newp := m.st.set(params)
	if result.Successful {
		m.publishEvent(nil, newp)
	}
	if !result.Successful {
		return result, fmt.Errorf("set failed: %s", result.Reason)
	}
	return result, nil
}

// SetValue is a convenience wrapper for Set that updates a single parameter by name.
func (m *Manager) SetValue(name string, v Value) (SetResult, error) {
	return m.Set(Parameter{Name: name, Value: v})
}

// SimTimeEnabled reports whether 'use_sim_time' is currently true.
func (m *Manager) SimTimeEnabled() bool {
	if p, ok := m.st.get("use_sim_time"); ok && p.Value.Kind == KindBool {
		return p.Value.Bool
	}
	return false
}

// Undeclare removes a parameter and publishes a delete event.
func (m *Manager) Undeclare(name string) bool {
	if old, ok := m.st.undeclare(name); ok {
		m.publishDelete([]Parameter{old})
		return true
	}
	return false
}

// SetNowProvider lets you provide a sim-time source. When 'use_sim_time' is true,
// publishEvent()/publishDelete() will use this instead of wall-clock.
func (m *Manager) SetNowProvider(f func() builtin_interfaces_msg.Time) {
	m.nowProvider = f
}

// --- Service handlers ---

func (m *Manager) onGetParameters(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv2.GetParameters_Request)
	resp := rcl_interfaces_srv2.NewGetParameters_Response()

	values := make([]rcl_interfaces_msg2.ParameterValue, 0, len(req.Names))
	for _, name := range req.Names {
		if p, ok := m.st.get(name); ok {
			values = append(values, toRclValue(p.Value))
		} else {
			values = append(values, rcl_interfaces_msg2.ParameterValue{Type: KindNotSet})
		}
	}
	resp.Values = values
	_ = send.SendResponse(resp)
}

func (m *Manager) onSetParameters(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv2.SetParameters_Request)
	resp := rcl_interfaces_srv2.NewSetParameters_Response()

	incoming := make([]Parameter, 0, len(req.Parameters))
	for _, p := range req.Parameters {
		incoming = append(incoming, Parameter{Name: p.Name, Value: fromRclValue(p.Value)})
	}
	// Each parameter is set on its own, as in rclcpp: one that is refused
	// does not stop the others, and each gets its own result.
	results := make([]rcl_interfaces_msg2.SetParametersResult, len(incoming))
	var changed []Parameter
	for i, p := range incoming {
		result, _, newp := m.st.set([]Parameter{p})
		results[i] = rcl_interfaces_msg2.SetParametersResult{Successful: result.Successful, Reason: result.Reason}
		if result.Successful {
			changed = append(changed, newp...)
		}
	}
	resp.Results = results
	if len(changed) > 0 {
		m.publishEvent(nil, changed)
	}
	_ = send.SendResponse(resp)
}

// onSetParametersAtomically sets all parameters or none.
func (m *Manager) onSetParametersAtomically(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv2.SetParametersAtomically_Request)
	resp := rcl_interfaces_srv2.NewSetParametersAtomically_Response()
	incoming := make([]Parameter, 0, len(req.Parameters))
	for _, p := range req.Parameters {
		incoming = append(incoming, Parameter{Name: p.Name, Value: fromRclValue(p.Value)})
	}
	result, _, newp := m.st.set(incoming)
	resp.Result = rcl_interfaces_msg2.SetParametersResult{Successful: result.Successful, Reason: result.Reason}
	if result.Successful {
		m.publishEvent(nil, newp)
	}
	_ = send.SendResponse(resp)
}

func (m *Manager) onGetParameterTypes(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv2.GetParameterTypes_Request)
	resp := rcl_interfaces_srv2.NewGetParameterTypes_Response()
	resp.Types = make([]uint8, len(req.Names))
	for i, name := range req.Names {
		if p, ok := m.st.get(name); ok {
			resp.Types[i] = p.Value.Kind
		}
	}
	_ = send.SendResponse(resp)
}

func (m *Manager) onListParameters(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv2.ListParameters_Request)
	resp := rcl_interfaces_srv2.NewListParameters_Response()
	resp.Result.Names, resp.Result.Prefixes = listParameters(m.st.list(), req.Prefixes, req.Depth)
	_ = send.SendResponse(resp)
}

// listParameters selects the parameter names under prefixes, as rclcpp does: a
// prefix matches a name equal to it or followed by '.', depth (0 = unlimited)
// limits how many '.'-separated segments below the prefix (or below the root
// without prefixes) a name may have, and prefixes reports the distinct
// parents of the selected names.
func listParameters(all []Parameter, wantPrefixes []string, depth uint64) (names, prefixes []string) {
	within := func(rest string) bool {
		return depth == 0 || uint64(strings.Count(rest, "."))+1 <= depth
	}
	for _, p := range all {
		match := false
		if len(wantPrefixes) == 0 {
			match = within(p.Name)
		}
		for _, pref := range wantPrefixes {
			if p.Name == pref || (strings.HasPrefix(p.Name, pref+".") && within(p.Name[len(pref)+1:])) {
				match = true
				break
			}
		}
		if !match {
			continue
		}
		names = append(names, p.Name)
		if i := strings.LastIndex(p.Name, "."); i >= 0 && !slices.Contains(prefixes, p.Name[:i]) {
			prefixes = append(prefixes, p.Name[:i])
		}
	}
	slices.Sort(names)
	slices.Sort(prefixes)
	return names, prefixes
}

func (m *Manager) onDescribeParameters(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv2.DescribeParameters_Request)
	resp := rcl_interfaces_srv2.NewDescribeParameters_Response()

	descs := make([]rcl_interfaces_msg2.ParameterDescriptor, 0, len(req.Names))
	for _, name := range req.Names {
		if p, ok := m.st.get(name); ok {
			d, _ := m.st.descriptor(name)
			pd := rcl_interfaces_msg2.ParameterDescriptor{
				Name:        name,
				Type:        uint8(p.Value.Kind),
				Description: d.Description,
				ReadOnly:    d.ReadOnly,
			}
			// Ranges
			// A missing bound is the type's limit; the message cannot
			// express a one-sided range.
			if d.MinInt != nil || d.MaxInt != nil {
				from, to := int64(math.MinInt64), int64(math.MaxInt64)
				if d.MinInt != nil {
					from = *d.MinInt
				}
				if d.MaxInt != nil {
					to = *d.MaxInt
				}
				pd.IntegerRange = []rcl_interfaces_msg2.IntegerRange{{
					FromValue: from,
					ToValue:   to,
					Step:      1,
				}}
			}
			if d.MinDouble != nil || d.MaxDouble != nil {
				from, to := -math.MaxFloat64, math.MaxFloat64
				if d.MinDouble != nil {
					from = *d.MinDouble
				}
				if d.MaxDouble != nil {
					to = *d.MaxDouble
				}
				pd.FloatingPointRange = []rcl_interfaces_msg2.FloatingPointRange{{
					FromValue: from,
					ToValue:   to,
					Step:      0.0,
				}}
			}
			// Allowed strings -> human-readable constraints
			if len(d.AllowedStrings) > 0 {
				pd.AdditionalConstraints = "allowed: " + joinStrings(d.AllowedStrings, ",")
			}
			descs = append(descs, pd)
			continue
		}
		// Placeholder for undeclared names
		descs = append(descs, rcl_interfaces_msg2.ParameterDescriptor{
			Name: name,
			Type: KindNotSet,
		})
	}
	resp.Descriptors = descs
	_ = send.SendResponse(resp)
}

// --- Event publishers ---

func (m *Manager) stampNow() builtin_interfaces_msg.Time {
	// If use_sim_time is true and a nowProvider is set, use it.
	if p, ok := m.st.get("use_sim_time"); ok && p.Value.Kind == KindBool && p.Value.Bool && m.nowProvider != nil {
		return m.nowProvider()
	}
	// Fallback to wall clock
	now := time.Now()
	return builtin_interfaces_msg.Time{
		Sec:     int32(now.Unix()),
		Nanosec: uint32(now.Nanosecond()),
	}
}

// publishEvent publishes a ParameterEvent with declared parameters in
// new_parameters and the new values of changed parameters in
// changed_parameters.
func (m *Manager) publishEvent(declared, changed []Parameter) {
	if len(changed) > 0 {
		m.runPostSet(changed)
	}
	if m.pub == nil && m.eventSink == nil {
		return
	}
	evt := m.newEvent()
	for _, p := range declared {
		evt.NewParameters = append(evt.NewParameters, rcl_interfaces_msg2.Parameter{
			Name:  p.Name,
			Value: toRclValue(p.Value),
		})
	}
	for _, p := range changed {
		evt.ChangedParameters = append(evt.ChangedParameters, rcl_interfaces_msg2.Parameter{
			Name:  p.Name,
			Value: toRclValue(p.Value),
		})
	}
	m.emit(evt)
}

func (m *Manager) publishDelete(del []Parameter) {
	if m.pub == nil && m.eventSink == nil {
		return
	}
	evt := m.newEvent()
	for _, p := range del {
		evt.DeletedParameters = append(evt.DeletedParameters, rcl_interfaces_msg2.Parameter{
			Name:  p.Name,
			Value: toRclValue(p.Value),
		})
	}
	m.emit(evt)
}

func (m *Manager) newEvent() *rcl_interfaces_msg2.ParameterEvent {
	evt := &rcl_interfaces_msg2.ParameterEvent{Stamp: m.stampNow()}
	if m.n != nil {
		evt.Node = m.n.FullyQualifiedName()
	}
	return evt
}

func (m *Manager) emit(evt *rcl_interfaces_msg2.ParameterEvent) {
	if m.eventSink != nil {
		m.eventSink(evt)
	}
	if m.pub != nil {
		_ = m.pub.Publish(evt)
	}
}

// --- Converters between Value and rcl_interfaces/ParameterValue ---

func toRclValue(v Value) rcl_interfaces_msg2.ParameterValue {
	out := rcl_interfaces_msg2.ParameterValue{Type: uint8(v.Kind)}
	switch v.Kind {
	case KindBool:
		out.BoolValue = v.Bool
	case KindInt64:
		out.IntegerValue = v.Int64
	case KindDouble:
		out.DoubleValue = v.Double
	case KindString:
		out.StringValue = v.Str
	case KindBytes:
		out.ByteArrayValue = v.Bytes
	case KindBoolArray:
		out.BoolArrayValue = v.Bools
	case KindInt64Array:
		out.IntegerArrayValue = v.Int64s
	case KindDoubleArray:
		out.DoubleArrayValue = v.Doubles
	case KindStringArray:
		out.StringArrayValue = v.Strs
	}
	return out
}

func fromRclValue(v rcl_interfaces_msg2.ParameterValue) Value {
	out := Value{Kind: v.Type}
	switch v.Type {
	case KindBool:
		out.Bool = v.BoolValue
	case KindInt64:
		out.Int64 = v.IntegerValue
	case KindDouble:
		out.Double = v.DoubleValue
	case KindString:
		out.Str = v.StringValue
	case KindBytes:
		out.Bytes = v.ByteArrayValue
	case KindBoolArray:
		out.Bools = v.BoolArrayValue
	case KindInt64Array:
		out.Int64s = v.IntegerArrayValue
	case KindDoubleArray:
		out.Doubles = v.DoubleArrayValue
	case KindStringArray:
		out.Strs = v.StringArrayValue
	}
	return out
}

// tiny local helper, avoids importing strings.Builder just for join
func joinStrings(ss []string, sep string) string {
	if len(ss) == 0 {
		return ""
	}
	n := 0
	for _, s := range ss {
		n += len(s)
	}
	n += (len(ss) - 1) * len(sep)
	b := make([]byte, 0, n)
	for i, s := range ss {
		if i > 0 {
			b = append(b, sep...)
		}
		b = append(b, s...)
	}
	return string(b)
}
