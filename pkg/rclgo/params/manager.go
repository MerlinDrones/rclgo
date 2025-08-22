package params

import (
	"log"
	"strings"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"

	builtin_interfaces_msg "github.com/merlindrones/rclgo/internal/msgs/builtin_interfaces/msg"
	rcl_interfaces_msg "github.com/merlindrones/rclgo/internal/msgs/rcl_interfaces/msg"
	rcl_interfaces_srv "github.com/merlindrones/rclgo/internal/msgs/rcl_interfaces/srv"
)

type Manager struct {
	st  *store
	pub *rclgo.Publisher // /parameter_events
	n   *rclgo.Node

	// Optional provider for simulated time. If nil, wall clock is used.
	// When 'use_sim_time' is true and nowProvider != nil, event stamps use nowProvider().
	nowProvider func() builtin_interfaces_msg.Time
}

func NewManager(n *rclgo.Node) (*Manager, error) {
	m := &Manager{st: newStore(), n: n}

	// /parameter_events publisher: Reliable + TransientLocal + KeepAll
	opts := &rclgo.PublisherOptions{
		Qos: rclgo.NewParameterEventsQosProfile(),
	}
	pub, err := n.NewPublisher("/parameter_events", rcl_interfaces_msg.ParameterEventTypeSupport, opts)
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

	// Node-scoped parameter services (match rclcpp/rclpy)
	base := n.FullyQualifiedName()
	if base == "" {
		base = "/" + n.Name()
	}
	log.Printf("[params] node base=%s", base)

	if _, err := n.NewService(base+"/get_parameters", rcl_interfaces_srv.GetParametersTypeSupport, nil, m.onGetParameters); err != nil {
		return nil, err
	}
	log.Printf("[params] service registered: %s/get_parameters", base)

	if _, err := n.NewService(base+"/set_parameters", rcl_interfaces_srv.SetParametersTypeSupport, nil, m.onSetParameters); err != nil {
		return nil, err
	}
	log.Printf("[params] service registered: %s/set_parameters", base)

	if _, err := n.NewService(base+"/list_parameters", rcl_interfaces_srv.ListParametersTypeSupport, nil, m.onListParameters); err != nil {
		return nil, err
	}
	log.Printf("[params] service registered: %s/list_parameters", base)

	if _, err := n.NewService(base+"/describe_parameters", rcl_interfaces_srv.DescribeParametersTypeSupport, nil, m.onDescribeParameters); err != nil {
		return nil, err
	}
	log.Printf("[params] service registered: %s/describe_parameters", base)

	return m, nil
}

// Public API

func (m *Manager) Declare(name string, v Value, d Descriptor) (Parameter, error) {
	p, err := m.st.declare(name, v, d)
	if err == nil {
		m.publishEvent([]Parameter{p}, nil)
	}
	return p, err
}

func (m *Manager) Get(name string) (Parameter, bool) { return m.st.get(name) }
func (m *Manager) List() []Parameter                 { return m.st.list() }
func (m *Manager) OnSet(cb OnSetCallback)            { m.st.cb = cb }

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
	req := reqMsg.(*rcl_interfaces_srv.GetParameters_Request)
	resp := rcl_interfaces_srv.NewGetParameters_Response()

	values := make([]rcl_interfaces_msg.ParameterValue, 0, len(req.Names))
	for _, name := range req.Names {
		if p, ok := m.st.get(name); ok {
			values = append(values, toRclValue(p.Value))
		} else {
			values = append(values, rcl_interfaces_msg.ParameterValue{Type: KindNotSet})
		}
	}
	resp.Values = values
	_ = send.SendResponse(resp)
}

func (m *Manager) onSetParameters(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv.SetParameters_Request)
	resp := rcl_interfaces_srv.NewSetParameters_Response()

	incoming := make([]Parameter, 0, len(req.Parameters))
	for _, p := range req.Parameters {
		incoming = append(incoming, Parameter{Name: p.Name, Value: fromRclValue(p.Value)})
	}
	result, old, newp := m.st.set(incoming)

	// Per-item results: mirror ROS 2 semantics (one result per requested parameter)
	results := make([]rcl_interfaces_msg.SetParametersResult, len(req.Parameters))
	for i := range results {
		results[i] = rcl_interfaces_msg.SetParametersResult{
			Successful: result.Successful,
			Reason:     result.Reason,
		}
	}
	resp.Results = results

	if result.Successful {
		m.publishEvent(newp, old)
	}
	_ = send.SendResponse(resp)
}

func (m *Manager) onListParameters(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv.ListParameters_Request)
	resp := rcl_interfaces_srv.NewListParameters_Response()

	var names []string
	all := m.st.list()

	// depth semantics: 0 == unlimited (ROS 2)
	withinDepth := func(name, prefix string, depth uint64) bool {
		if depth == 0 {
			return true
		}
		rest := strings.TrimPrefix(name, prefix)
		rest = strings.TrimPrefix(rest, ".")
		if rest == "" {
			return true
		}
		segments := uint64(strings.Count(rest, ".") + 1)
		return segments <= depth
	}

	for _, p := range all {
		if len(req.Prefixes) == 0 {
			names = append(names, p.Name)
			continue
		}
		okDepth := false
		for _, pref := range req.Prefixes {
			if strings.HasPrefix(p.Name, pref) && withinDepth(p.Name, pref, req.Depth) {
				okDepth = true
				break
			}
		}
		if okDepth {
			names = append(names, p.Name)
		}
	}

	resp.Result.Names = names
	_ = send.SendResponse(resp)
}

func (m *Manager) onDescribeParameters(_ *rclgo.ServiceInfo, reqMsg types.Message, send rclgo.ServiceResponseSender) {
	req := reqMsg.(*rcl_interfaces_srv.DescribeParameters_Request)
	resp := rcl_interfaces_srv.NewDescribeParameters_Response()

	descs := make([]rcl_interfaces_msg.ParameterDescriptor, 0, len(req.Names))
	for _, name := range req.Names {
		if p, ok := m.st.get(name); ok {
			d := m.st.desc[name]
			pd := rcl_interfaces_msg.ParameterDescriptor{
				Name:        name,
				Type:        uint8(p.Value.Kind),
				Description: d.Description,
				ReadOnly:    d.ReadOnly,
			}
			// Ranges
			if d.MinInt != nil || d.MaxInt != nil {
				var from, to int64
				if d.MinInt != nil {
					from = *d.MinInt
				}
				if d.MaxInt != nil {
					to = *d.MaxInt
				}
				pd.IntegerRange = []rcl_interfaces_msg.IntegerRange{{
					FromValue: from,
					ToValue:   to,
					Step:      1,
				}}
			}
			if d.MinDouble != nil || d.MaxDouble != nil {
				var from, to float64
				if d.MinDouble != nil {
					from = *d.MinDouble
				}
				if d.MaxDouble != nil {
					to = *d.MaxDouble
				}
				pd.FloatingPointRange = []rcl_interfaces_msg.FloatingPointRange{{
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
		descs = append(descs, rcl_interfaces_msg.ParameterDescriptor{
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

func (m *Manager) publishEvent(newp, old []Parameter) {
	if m.pub == nil {
		return
	}
	evt := &rcl_interfaces_msg.ParameterEvent{
		Stamp: m.stampNow(),
		Node:  m.n.FullyQualifiedName(),
	}
	for _, p := range newp {
		evt.NewParameters = append(evt.NewParameters, rcl_interfaces_msg.Parameter{
			Name:  p.Name,
			Value: toRclValue(p.Value),
		})
	}
	for _, p := range old {
		evt.ChangedParameters = append(evt.ChangedParameters, rcl_interfaces_msg.Parameter{
			Name:  p.Name,
			Value: toRclValue(p.Value),
		})
	}
	_ = m.pub.Publish(evt)
}

func (m *Manager) publishDelete(del []Parameter) {
	if m.pub == nil {
		return
	}
	evt := &rcl_interfaces_msg.ParameterEvent{
		Stamp: m.stampNow(),
		Node:  m.n.FullyQualifiedName(),
	}
	for _, p := range del {
		evt.DeletedParameters = append(evt.DeletedParameters, rcl_interfaces_msg.Parameter{
			Name:  p.Name,
			Value: toRclValue(p.Value),
		})
	}
	_ = m.pub.Publish(evt)
}

// --- Converters between Value and rcl_interfaces/ParameterValue ---

func toRclValue(v Value) rcl_interfaces_msg.ParameterValue {
	out := rcl_interfaces_msg.ParameterValue{Type: uint8(v.Kind)}
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

func fromRclValue(v rcl_interfaces_msg.ParameterValue) Value {
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
