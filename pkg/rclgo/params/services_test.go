package params_test

import (
	"fmt"
	"go.uber.org/goleak"
	"math"
	"slices"
	"strings"
	"testing"
	"time"

	rcl_interfaces_msg "github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/msg"
	rcl_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Parameter services end to end, after rclcpp's test_parameter_service and
// test_parameter_client: a node with a params.Manager is queried through the
// real rcl_interfaces services, from Go and with the ros2 command-line tools.

func TestMain(m *testing.M) {
	rclgotest.UseIsolatedDomain()
	goleak.VerifyTestMain(m)
}

func i64(v int64) *int64     { return &v }
func f64(v float64) *float64 { return &v }

// newParamNode returns a spinning node with a parameter manager and a few
// declared parameters, and a client node on another context.
func newParamNode(t *testing.T, name string) (*params.Manager, *rclgo.Node, *rclgo.Node) {
	t.Helper()
	server := rclgotest.Node(t, name, "/params_test")
	m, err := params.NewManager(server)
	require.NoError(t, err)
	for _, d := range []struct {
		name string
		v    params.Value
		d    params.Descriptor
	}{
		{"camera.fps", params.Value{Kind: params.KindInt64, Int64: 15}, params.Descriptor{MinInt: i64(1), MaxInt: i64(60)}},
		{"camera.name", params.Value{Kind: params.KindString, Str: "front"}, params.Descriptor{Description: "camera name"}},
		{"camera_fps", params.Value{Kind: params.KindInt64, Int64: 1}, params.Descriptor{}},
		{"gain", params.Value{Kind: params.KindDouble, Double: 0.5}, params.Descriptor{MinDouble: f64(0)}},
		{"frame", params.Value{Kind: params.KindString, Str: "map"}, params.Descriptor{ReadOnly: true}},
	} {
		_, err := m.Declare(d.name, d.v, d.d)
		require.NoError(t, err)
	}
	rclgotest.Spin(t, server.Context().Spin)
	client := rclgotest.Node(t, name+"_client", "/params_test")
	rclgotest.Spin(t, client.Context().Spin)
	return m, server, client
}

func paramClient(t *testing.T, node *rclgo.Node, server, service string, ts types.ServiceTypeSupport) *rclgo.Client {
	t.Helper()
	c, err := node.NewClient("/params_test/"+server+"/"+service, ts, nil)
	require.NoError(t, err)
	return c
}

// rclcpp and rclpy register six parameter services per node.
func TestParameterServicesRegistered(t *testing.T) {
	_, _, client := newParamNode(t, "registered")
	want := []string{
		"describe_parameters", "get_parameter_types", "get_parameters",
		"list_parameters", "set_parameters", "set_parameters_atomically",
	}
	rclgotest.Eventually(t, "parameter services", func() error {
		got, err := client.GetServiceNamesAndTypesByNode("registered", "/params_test")
		if err != nil {
			return err
		}
		var missing []string
		for _, s := range want {
			if _, ok := got["/params_test/registered/"+s]; !ok {
				missing = append(missing, s)
			}
		}
		if len(missing) > 0 {
			return fmt.Errorf("missing %v (have %v)", missing, got)
		}
		return nil
	})
}

func TestGetAndSetParametersOverServices(t *testing.T) {
	m, _, client := newParamNode(t, "getset")
	get := paramClient(t, client, "getset", "get_parameters", rcl_interfaces_srv.GetParametersTypeSupport)
	set := paramClient(t, client, "getset", "set_parameters", rcl_interfaces_srv.SetParametersTypeSupport)

	req := rcl_interfaces_srv.NewGetParameters_Request()
	req.Names = []string{"camera.fps", "camera.name", "missing"}
	resp := rclgotest.Call(t, get, req).(*rcl_interfaces_srv.GetParameters_Response)
	require.Len(t, resp.Values, 3)
	require.EqualValues(t, params.KindInt64, resp.Values[0].Type)
	require.EqualValues(t, 15, resp.Values[0].IntegerValue)
	require.Equal(t, "front", resp.Values[1].StringValue)
	require.EqualValues(t, params.KindNotSet, resp.Values[2].Type)

	setReq := rcl_interfaces_srv.NewSetParameters_Request()
	setReq.Parameters = []rcl_interfaces_msg.Parameter{
		{Name: "camera.fps", Value: rcl_interfaces_msg.ParameterValue{Type: params.KindInt64, IntegerValue: 30}},
	}
	setResp := rclgotest.Call(t, set, setReq).(*rcl_interfaces_srv.SetParameters_Response)
	require.Len(t, setResp.Results, 1)
	require.True(t, setResp.Results[0].Successful, setResp.Results[0].Reason)
	p, _ := m.Get("camera.fps")
	require.EqualValues(t, 30, p.Value.Int64)

	for _, bad := range []rcl_interfaces_msg.Parameter{
		{Name: "camera.fps", Value: rcl_interfaces_msg.ParameterValue{Type: params.KindInt64, IntegerValue: 500}}, // above max
		{Name: "frame", Value: rcl_interfaces_msg.ParameterValue{Type: params.KindString, StringValue: "odom"}},   // read-only
		{Name: "camera.fps", Value: rcl_interfaces_msg.ParameterValue{Type: params.KindString, StringValue: "x"}}, // wrong type
		{Name: "undeclared", Value: rcl_interfaces_msg.ParameterValue{Type: params.KindBool, BoolValue: true}},
	} {
		setReq.Parameters = []rcl_interfaces_msg.Parameter{bad}
		r := rclgotest.Call(t, set, setReq).(*rcl_interfaces_srv.SetParameters_Response)
		require.Len(t, r.Results, 1)
		require.False(t, r.Results[0].Successful, "set of %s succeeded", bad.Name)
		require.NotEmpty(t, r.Results[0].Reason)
	}
	p, _ = m.Get("camera.fps")
	require.EqualValues(t, 30, p.Value.Int64, "a refused set changed the value")
}

// set_parameters applies each parameter on its own: one bad parameter does
// not stop the others (rclcpp semantics); set_parameters_atomically is all or
// nothing.
func TestSetParametersPerParameterAndAtomically(t *testing.T) {
	m, _, client := newParamNode(t, "atomic")
	set := paramClient(t, client, "atomic", "set_parameters", rcl_interfaces_srv.SetParametersTypeSupport)
	good := rcl_interfaces_msg.Parameter{Name: "gain", Value: rcl_interfaces_msg.ParameterValue{Type: params.KindDouble, DoubleValue: 0.9}}
	bad := rcl_interfaces_msg.Parameter{Name: "camera.fps", Value: rcl_interfaces_msg.ParameterValue{Type: params.KindInt64, IntegerValue: 500}}

	req := rcl_interfaces_srv.NewSetParameters_Request()
	req.Parameters = []rcl_interfaces_msg.Parameter{good, bad}
	resp := rclgotest.Call(t, set, req).(*rcl_interfaces_srv.SetParameters_Response)
	require.Len(t, resp.Results, 2)
	require.True(t, resp.Results[0].Successful, "good parameter: %s", resp.Results[0].Reason)
	require.False(t, resp.Results[1].Successful, "bad parameter")
	p, _ := m.Get("gain")
	require.InDelta(t, 0.9, p.Value.Double, 1e-9)

	atomic := paramClient(t, client, "atomic", "set_parameters_atomically", rcl_interfaces_srv.SetParametersAtomicallyTypeSupport)
	areq := rcl_interfaces_srv.NewSetParametersAtomically_Request()
	good.Value.DoubleValue = 0.1
	areq.Parameters = []rcl_interfaces_msg.Parameter{good, bad}
	aresp := rclgotest.Call(t, atomic, areq).(*rcl_interfaces_srv.SetParametersAtomically_Response)
	require.False(t, aresp.Result.Successful)
	p, _ = m.Get("gain")
	require.InDelta(t, 0.9, p.Value.Double, 1e-9, "an atomic set with a bad parameter changed another one")
}

func TestListParametersOverService(t *testing.T) {
	_, _, client := newParamNode(t, "list")
	list := paramClient(t, client, "list", "list_parameters", rcl_interfaces_srv.ListParametersTypeSupport)
	req := rcl_interfaces_srv.NewListParameters_Request()
	all := rclgotest.Call(t, list, req).(*rcl_interfaces_srv.ListParameters_Response)
	for _, n := range []string{"camera.fps", "camera.name", "camera_fps", "gain", "frame", "use_sim_time"} {
		require.Contains(t, all.Result.Names, n)
	}

	// A prefix matches whole name segments: "camera" lists camera.fps and
	// camera.name but not camera_fps, and the result reports the prefix.
	req.Prefixes = []string{"camera"}
	r := rclgotest.Call(t, list, req).(*rcl_interfaces_srv.ListParameters_Response)
	slices.Sort(r.Result.Names)
	require.Equal(t, []string{"camera.fps", "camera.name"}, r.Result.Names)
	require.Contains(t, r.Result.Prefixes, "camera")
}

func TestDescribeParametersOverService(t *testing.T) {
	_, _, client := newParamNode(t, "describe")
	describe := paramClient(t, client, "describe", "describe_parameters", rcl_interfaces_srv.DescribeParametersTypeSupport)
	types := paramClient(t, client, "describe", "get_parameter_types", rcl_interfaces_srv.GetParameterTypesTypeSupport)

	req := rcl_interfaces_srv.NewDescribeParameters_Request()
	req.Names = []string{"camera.fps", "gain", "frame", "camera.name"}
	resp := rclgotest.Call(t, describe, req).(*rcl_interfaces_srv.DescribeParameters_Response)
	require.Len(t, resp.Descriptors, 4)
	fps, gain, frame, name := resp.Descriptors[0], resp.Descriptors[1], resp.Descriptors[2], resp.Descriptors[3]
	require.Len(t, fps.IntegerRange, 1)
	require.EqualValues(t, 1, fps.IntegerRange[0].FromValue)
	require.EqualValues(t, 60, fps.IntegerRange[0].ToValue)
	// Only a minimum: the range extends to the largest double, not to 0.
	require.Len(t, gain.FloatingPointRange, 1)
	require.EqualValues(t, 0, gain.FloatingPointRange[0].FromValue)
	require.Equal(t, math.MaxFloat64, gain.FloatingPointRange[0].ToValue)
	require.True(t, frame.ReadOnly)
	require.Equal(t, "camera name", name.Description)
	require.EqualValues(t, params.KindString, name.Type)

	treq := rcl_interfaces_srv.NewGetParameterTypes_Request()
	treq.Names = []string{"camera.fps", "gain", "missing"}
	tresp := rclgotest.Call(t, types, treq).(*rcl_interfaces_srv.GetParameterTypes_Response)
	require.Equal(t, []uint8{params.KindInt64, params.KindDouble, params.KindNotSet}, tresp.Types)
}

// /parameter_events carries declarations and changes with the node's name.
func TestParameterEventsPublished(t *testing.T) {
	m, server, client := newParamNode(t, "events")
	events := make(chan *rcl_interfaces_msg.ParameterEvent, 64)
	_, err := client.NewSubscription("/parameter_events", rcl_interfaces_msg.ParameterEventTypeSupport,
		&rclgo.SubscriptionOptions{Qos: qos.NewParameterEventsProfile()}, func(s *rclgo.Subscription) {
			e := rcl_interfaces_msg.NewParameterEvent()
			if _, err := s.TakeMessage(e); err == nil {
				events <- e
			}
		})
	require.NoError(t, err)
	rclgotest.Eventually(t, "event subscription matched", func() error {
		if _, err := m.Declare(fmt.Sprintf("probe_%d", time.Now().UnixNano()), params.Value{Kind: params.KindBool}, params.Descriptor{}); err != nil {
			return err
		}
		select {
		case <-events:
			return nil
		case <-time.After(200 * time.Millisecond):
			return fmt.Errorf("no event")
		}
	})
	for len(events) > 0 {
		<-events
	}

	_, err = m.Declare("declared_later", params.Value{Kind: params.KindInt64, Int64: 7}, params.Descriptor{})
	require.NoError(t, err)
	_, err = m.SetValue("gain", params.Value{Kind: params.KindDouble, Double: 0.25})
	require.NoError(t, err)

	// Events arrive in publication order, but late probe events from the
	// discovery loop above may still be in flight; skip those.
	want := []func(*rcl_interfaces_msg.ParameterEvent) bool{
		func(e *rcl_interfaces_msg.ParameterEvent) bool {
			return len(e.NewParameters) == 1 && e.NewParameters[0].Name == "declared_later" &&
				e.NewParameters[0].Value.IntegerValue == 7 && len(e.ChangedParameters) == 0
		},
		func(e *rcl_interfaces_msg.ParameterEvent) bool {
			return len(e.ChangedParameters) == 1 && e.ChangedParameters[0].Name == "gain" &&
				e.ChangedParameters[0].Value.DoubleValue == 0.25 && len(e.NewParameters) == 0
		},
	}
	for i, ok := range want {
		for {
			select {
			case e := <-events:
				// /parameter_events is shared by every node in the domain.
				if e.Node != server.FullyQualifiedName() ||
					(len(e.NewParameters) == 1 && strings.HasPrefix(e.NewParameters[0].Name, "probe_")) {
					continue
				}
				require.Equal(t, server.FullyQualifiedName(), e.Node)
				require.True(t, ok(e), "event %d: %+v", i, e)
				require.NotZero(t, e.Stamp.Sec, "event %d has no stamp", i)
			case <-time.After(rclgotest.Timeout):
				t.Fatalf("event %d not received", i)
			}
			break
		}
	}
}

// The ros2 command-line tools work against an rclgo node.
func TestRos2ParamCLI(t *testing.T) {
	rclgotest.RequireROS2(t)
	m, _, _ := newParamNode(t, "cli")
	node := "/params_test/cli"
	rclgotest.EventuallyROS2(t, "camera.fps", "param", "list", "--no-daemon", node)
	out := rclgotest.EventuallyROS2(t, "15", "param", "get", "--no-daemon", node, "camera.fps")
	require.Contains(t, out, "Integer value is: 15")

	out, err := rclgotest.ROS2(t, "param", "set", "--no-daemon", node, "camera.fps", "42")
	require.NoError(t, err, out)
	require.Contains(t, out, "Set parameter successful")
	p, _ := m.Get("camera.fps")
	require.EqualValues(t, 42, p.Value.Int64)

	out, _ = rclgotest.ROS2(t, "param", "set", "--no-daemon", node, "camera.fps", "500")
	require.True(t, strings.Contains(out, "Setting parameter failed"), "out-of-range set via ros2: %s", out)

	out, err = rclgotest.ROS2(t, "param", "describe", "--no-daemon", node, "camera.fps")
	require.NoError(t, err, out)
	require.Contains(t, out, "Type: integer")
	require.Contains(t, out, "Min value: 1")
	require.Contains(t, out, "Max value: 60")

	out, err = rclgotest.ROS2(t, "param", "dump", "--no-daemon", node)
	require.NoError(t, err, out)
	require.Contains(t, out, "ros__parameters")
	require.Contains(t, out, "fps: 42")
}
