/*
This file is part of rclgo

Copyright © 2025 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package params

import (
	"fmt"
	"math"
	"slices"
	"strings"
	"sync"
	"sync/atomic"
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
	// When 'use_sim_time' is true and nowProvider is set, event stamps use it.
	nowProvider atomic.Pointer[func() builtin_interfaces_msg.Time]

	// eventSink, if set, receives every parameter event (for tests).
	eventSink func(*rcl_interfaces_msg2.ParameterEvent)

	postSetMu   sync.Mutex
	postSet     []postSetCallback // in registration order
	postSetNext uint64
}

// AddPostSetCallback registers f to be called after parameters were declared
// or changed, with their new values: by Declare, Set or SetValue, through the
// set_parameters and set_parameters_atomically services, or by an override.
// Unlike OnSet, which validates and can refuse a change, f only observes it,
// and any number of post-set callbacks can be registered (as rclcpp's
// add_post_set_parameters_callback), in the order they were registered.
// Callbacks run in the order of the changes,
// while further changes wait; a change attempted from f fails, as in rclcpp.
// It returns a function that removes f.
func (m *Manager) AddPostSetCallback(f func([]Parameter)) (remove func()) {
	m.postSetMu.Lock()
	defer m.postSetMu.Unlock()
	id := m.postSetNext
	m.postSetNext++
	m.postSet = append(m.postSet, postSetCallback{id, f})
	return func() {
		m.postSetMu.Lock()
		defer m.postSetMu.Unlock()
		m.postSet = slices.DeleteFunc(m.postSet, func(c postSetCallback) bool { return c.id == id })
	}
}

type postSetCallback struct {
	id uint64
	f  func([]Parameter)
}

func (m *Manager) runPostSet(changed []Parameter) {
	m.postSetMu.Lock()
	callbacks := slices.Clone(m.postSet)
	m.postSetMu.Unlock()
	for _, c := range callbacks {
		c.f(changed)
	}
}

func NewManager(n *rclgo.Node) (*Manager, error) {
	m := &Manager{st: newStore(), n: n}
	m.st.notify = m.onChange

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
	p, _, err := m.st.declareOrClaim(name, v, d)
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
	_, _, err := m.st.applyOverride(name, v, d)
	return err
}

func (m *Manager) Get(name string) (Parameter, bool) { return m.st.get(name) }
func (m *Manager) List() []Parameter                 { return m.st.list() }
func (m *Manager) OnSet(cb OnSetCallback)            { m.st.setCallback(cb) }

// Set updates one or more existing parameters. Returns SetResult indicating success/failure.
// All parameters must already be declared, and constraints will be validated.
func (m *Manager) Set(params ...Parameter) (SetResult, error) {
	result, _, _ := m.st.set(params)
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

// Undeclare removes a parameter and publishes a delete event, as rclcpp's
// undeclare_parameter. It fails for a parameter that isn't declared, is
// read-only, or is use_sim_time, which the node's clock follows.
func (m *Manager) Undeclare(name string) error {
	if name == "use_sim_time" {
		return fmt.Errorf("parameter %q cannot be undeclared: the node's clock uses it", name)
	}
	return m.st.undeclare(name)
}

// SetNowProvider lets you provide a sim-time source. When 'use_sim_time' is true,
// publishEvent()/publishDelete() will use this instead of wall-clock.
func (m *Manager) SetNowProvider(f func() builtin_interfaces_msg.Time) {
	if f == nil {
		m.nowProvider.Store(nil)
		return
	}
	m.nowProvider.Store(&f)
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
	m.logErr("failed to send parameter service response: ", send.SendResponse(resp))
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
	for i, p := range incoming {
		result, _, _ := m.st.set([]Parameter{p})
		results[i] = rcl_interfaces_msg2.SetParametersResult{Successful: result.Successful, Reason: result.Reason}
	}
	resp.Results = results
	m.logErr("failed to send parameter service response: ", send.SendResponse(resp))
}

// onSetParametersAtomically sets all parameters or none.
func (m *Manager) onSetParametersAtomically(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv2.SetParametersAtomically_Request)
	resp := rcl_interfaces_srv2.NewSetParametersAtomically_Response()
	incoming := make([]Parameter, 0, len(req.Parameters))
	for _, p := range req.Parameters {
		incoming = append(incoming, Parameter{Name: p.Name, Value: fromRclValue(p.Value)})
	}
	result, _, _ := m.st.set(incoming)
	resp.Result = rcl_interfaces_msg2.SetParametersResult{Successful: result.Successful, Reason: result.Reason}
	m.logErr("failed to send parameter service response: ", send.SendResponse(resp))
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
	m.logErr("failed to send parameter service response: ", send.SendResponse(resp))
}

func (m *Manager) onListParameters(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv2.ListParameters_Request)
	resp := rcl_interfaces_srv2.NewListParameters_Response()
	resp.Result.Names, resp.Result.Prefixes = listParameters(m.st.list(), req.Prefixes, req.Depth)
	m.logErr("failed to send parameter service response: ", send.SendResponse(resp))
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
	m.logErr("failed to send parameter service response: ", send.SendResponse(resp))
}

// --- Event publishers ---

func (m *Manager) stampNow() builtin_interfaces_msg.Time {
	// If use_sim_time is true and a nowProvider is set, use it.
	if p, ok := m.st.get("use_sim_time"); ok && p.Value.Kind == KindBool && p.Value.Bool {
		if f := m.nowProvider.Load(); f != nil {
			return (*f)()
		}
	}
	// Fallback to wall clock
	now := time.Now()
	return builtin_interfaces_msg.Time{
		Sec:     int32(now.Unix()),
		Nanosec: uint32(now.Nanosecond()),
	}
}

// logErr logs err, if not nil, with the node's logger.
func (m *Manager) logErr(what string, err error) {
	if err != nil {
		m.n.Logger().Error(what, err) //nolint:errcheck
	}
}

// onChange is the store's change notification: it runs the post-set
// callbacks for declared and changed parameters (as rclcpp runs them after
// declare_parameter and set_parameters) and publishes the parameter event. The
// store calls it with its change lock held, so notifications arrive in the
// order of the changes.
func (m *Manager) onChange(declared, changed, deleted []Parameter) {
	if len(declared)+len(changed) > 0 {
		m.runPostSet(append(append([]Parameter(nil), declared...), changed...))
	}
	if len(declared)+len(changed) > 0 {
		m.publishEvent(declared, changed)
	}
	if len(deleted) > 0 {
		m.publishDelete(deleted)
	}
}

// publishEvent publishes a ParameterEvent with declared parameters in
// new_parameters and the new values of changed parameters in
// changed_parameters.
func (m *Manager) publishEvent(declared, changed []Parameter) {
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
		m.logErr("failed to publish parameter event: ", m.pub.Publish(evt))
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
