package params

import (
	"testing"

	"github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/msg"
	"github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

// captureSender implements rclgo.ServiceResponseSender for tests.
type captureSender struct {
	last types.Message
}

func (c *captureSender) SendResponse(msg types.Message) error {
	c.last = msg
	return nil
}

func TestOnGetParameters_UnknownAndKnown(t *testing.T) {
	m := &Manager{st: newStore()} // pub/n are nil for unit tests

	// Declare one known parameter
	_, _ = m.Declare("camera.fps", Value{Kind: KindInt64, Int64: 15}, Descriptor{})

	req := rcl_interfaces_srv.NewGetParameters_Request()
	req.Names = []string{"camera.fps", "missing"}

	s := &captureSender{}
	m.onGetParameters((*rclgo.ServiceInfo)(nil), req, s)

	resp := s.last.(*rcl_interfaces_srv.GetParameters_Response)
	if got, want := len(resp.Values), 2; got != want {
		t.Fatalf("len(values)=%d, want %d", got, want)
	}
	if resp.Values[0].Type != KindInt64 || resp.Values[0].IntegerValue != 15 {
		t.Fatalf("first value wrong: %+v", resp.Values[0])
	}
	if resp.Values[1].Type != KindNotSet {
		t.Fatalf("second should be NotSet, got %+v", resp.Values[1])
	}
}

func TestOnSetParameters_SetsStoreAndReturnsResult(t *testing.T) {
	m := &Manager{st: newStore()}

	// Declare with bounds; later set to a valid new value
	_, _ = m.Declare("gain", Value{Kind: KindDouble, Double: 0.2},
		Descriptor{MinDouble: f64(0.0), MaxDouble: f64(1.0)})

	req := rcl_interfaces_srv.NewSetParameters_Request()
	req.Parameters = []rcl_interfaces_msg.Parameter{
		{
			Name: "gain",
			Value: rcl_interfaces_msg.ParameterValue{
				Type:        KindDouble,
				DoubleValue: 0.5,
			},
		},
	}

	s := &captureSender{}
	m.onSetParameters(nil, req, s)

	resp := s.last.(*rcl_interfaces_srv.SetParameters_Response)
	if got := len(resp.Results); got != 1 || !resp.Results[0].Successful {
		t.Fatalf("unexpected results: %+v", resp.Results)
	}
	// Store updated?
	if p, ok := m.Get("gain"); !ok || p.Value.Double != 0.5 {
		t.Fatalf("store not updated: ok=%v param=%+v", ok, p)
	}
}

func TestOnSetParameters_RejectedByValidation(t *testing.T) {
	m := &Manager{st: newStore()}
	_, _ = m.Declare("count", Value{Kind: KindInt64, Int64: 5},
		Descriptor{MinInt: i64(0), MaxInt: i64(10)})

	// Try above max
	req := rcl_interfaces_srv.NewSetParameters_Request()
	req.Parameters = []rcl_interfaces_msg.Parameter{
		{
			Name: "count",
			Value: rcl_interfaces_msg.ParameterValue{
				Type:         KindInt64,
				IntegerValue: 99,
			},
		},
	}

	s := &captureSender{}
	m.onSetParameters(nil, req, s)
	resp := s.last.(*rcl_interfaces_srv.SetParameters_Response)
	if len(resp.Results) != 1 || resp.Results[0].Successful {
		t.Fatalf("expected rejection, got %+v", resp.Results)
	}
	// unchanged
	if p, ok := m.Get("count"); !ok || p.Value.Int64 != 5 {
		t.Fatalf("store changed unexpectedly: ok=%v param=%+v", ok, p)
	}
}

func TestOnSetParameters_PerItemResultsLength(t *testing.T) {
	m := &Manager{st: newStore()}
	_, _ = m.Declare("a", Value{Kind: KindInt64, Int64: 1}, Descriptor{})
	_, _ = m.Declare("b", Value{Kind: KindInt64, Int64: 2}, Descriptor{})

	req := rcl_interfaces_srv.NewSetParameters_Request()
	req.Parameters = []rcl_interfaces_msg.Parameter{
		{Name: "a", Value: rcl_interfaces_msg.ParameterValue{Type: KindInt64, IntegerValue: 10}},
		{Name: "b", Value: rcl_interfaces_msg.ParameterValue{Type: KindInt64, IntegerValue: 20}},
	}

	s := &captureSender{}
	m.onSetParameters(nil, req, s)
	resp := s.last.(*rcl_interfaces_srv.SetParameters_Response)
	if len(resp.Results) != 2 {
		t.Fatalf("want 2 results, got %d", len(resp.Results))
	}
	if !resp.Results[0].Successful || !resp.Results[1].Successful {
		t.Fatalf("both results should be successful: %+v", resp.Results)
	}
}

func TestOnListParameters_ReturnsAllNames(t *testing.T) {
	m := &Manager{st: newStore()}
	_, _ = m.Declare("a", Value{Kind: KindBool, Bool: true}, Descriptor{})
	_, _ = m.Declare("b", Value{Kind: KindString, Str: "x"}, Descriptor{})

	req := rcl_interfaces_srv.NewListParameters_Request()
	s := &captureSender{}
	m.onListParameters(nil, req, s)

	resp := s.last.(*rcl_interfaces_srv.ListParameters_Response)
	got := map[string]bool{}
	for _, n := range resp.Result.Names {
		got[n] = true
	}
	if !got["a"] || !got["b"] || len(resp.Result.Names) != 2 {
		t.Fatalf("unexpected names: %+v", resp.Result.Names)
	}
}

func TestOnListParameters_PrefixAndDepth(t *testing.T) {
	m := &Manager{st: newStore()}
	_, _ = m.Declare("camera.fps", Value{Kind: KindInt64, Int64: 10}, Descriptor{})
	_, _ = m.Declare("camera.frame.id", Value{Kind: KindString, Str: "x"}, Descriptor{})
	_, _ = m.Declare("other.key", Value{Kind: KindString, Str: "y"}, Descriptor{})

	req := rcl_interfaces_srv.NewListParameters_Request()
	req.Prefixes = []string{"camera"}
	req.Depth = 1 // include only 1 segment after "camera"
	s := &captureSender{}
	m.onListParameters(nil, req, s)

	resp := s.last.(*rcl_interfaces_srv.ListParameters_Response)
	got := map[string]bool{}
	for _, n := range resp.Result.Names {
		got[n] = true
	}
	if !got["camera.fps"] {
		t.Fatalf("expected camera.fps present, got %+v", resp.Result.Names)
	}
	if got["camera.frame.id"] {
		t.Fatalf("depth=1 should exclude camera.frame.id, got %+v", resp.Result.Names)
	}
	if got["other.key"] {
		t.Fatalf("prefix filter should exclude other.key, got %+v", resp.Result.Names)
	}
}

func TestOnDescribeParameters_TypesAndPlaceholders(t *testing.T) {
	m := &Manager{st: newStore()}
	_, _ = m.Declare("x", Value{Kind: KindString, Str: "hi"}, Descriptor{})
	_, _ = m.Declare("y", Value{Kind: KindDouble, Double: 1.0}, Descriptor{})

	req := rcl_interfaces_srv.NewDescribeParameters_Request()
	req.Names = []string{"x", "y", "missing"}

	s := &captureSender{}
	m.onDescribeParameters(nil, req, s)

	resp := s.last.(*rcl_interfaces_srv.DescribeParameters_Response)
	if len(resp.Descriptors) != 3 {
		t.Fatalf("descriptors len=%d, want 3", len(resp.Descriptors))
	}
	var dx, dy, dm *rcl_interfaces_msg.ParameterDescriptor
	for i := range resp.Descriptors {
		switch resp.Descriptors[i].Name {
		case "x":
			dx = &resp.Descriptors[i]
		case "y":
			dy = &resp.Descriptors[i]
		case "missing":
			dm = &resp.Descriptors[i]
		}
	}
	if dx == nil || dx.Type != KindString {
		t.Fatalf("descriptor x wrong: %+v", dx)
	}
	if dy == nil || dy.Type != KindDouble {
		t.Fatalf("descriptor y wrong: %+v", dy)
	}
	if dm == nil || dm.Type != KindNotSet {
		t.Fatalf("placeholder for missing wrong: %+v", dm)
	}
}

func TestUndeclare_RemovesParam(t *testing.T) {
	m := &Manager{st: newStore()}
	_, _ = m.Declare("tmp", Value{Kind: KindInt64, Int64: 1}, Descriptor{})
	if !m.Undeclare("tmp") {
		t.Fatal("expected undeclare to return true")
	}
	if _, ok := m.Get("tmp"); ok {
		t.Fatal("expected tmp to be removed")
	}
}
