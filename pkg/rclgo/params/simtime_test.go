package params

import (
	"testing"

	rclif_msg "github.com/merlindrones/rclgo/internal/msgs/rcl_interfaces/msg"
	rclif_srv "github.com/merlindrones/rclgo/internal/msgs/rcl_interfaces/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

type sinkSender struct {
	last types.Message
}

func (s *sinkSender) SendResponse(m types.Message) error { s.last = m; return nil }

func TestEnableUseSimTimeHook_TogglesOnSet(t *testing.T) {
	m := &Manager{st: newStore()}

	var toggles []bool
	m.EnableUseSimTimeHook(func(enabled bool) {
		toggles = append(toggles, enabled)
	})

	// Set true
	req1 := rclif_srv.NewSetParameters_Request()
	req1.Parameters = []rclif_msg.Parameter{
		{
			Name: "use_sim_time",
			Value: rclif_msg.ParameterValue{
				Type:      KindBool,
				BoolValue: true,
			},
		},
	}
	s1 := &sinkSender{}
	m.onSetParameters(nil, req1, s1)
	resp1 := s1.last.(*rclif_srv.SetParameters_Response)
	if len(resp1.Results) != 1 || !resp1.Results[0].Successful {
		t.Fatalf("expected success, got %+v", resp1.Results)
	}
	if len(toggles) != 1 || toggles[0] != true {
		t.Fatalf("expected toggle true, got %+v", toggles)
	}

	// Verify store reflects it
	if p, ok := m.Get("use_sim_time"); !ok || p.Value.Kind != KindBool || !p.Value.Bool {
		t.Fatalf("store not updated to true: ok=%v param=%+v", ok, p)
	}

	// Set false
	req2 := rclif_srv.NewSetParameters_Request()
	req2.Parameters = []rclif_msg.Parameter{
		{
			Name: "use_sim_time",
			Value: rclif_msg.ParameterValue{
				Type:      KindBool,
				BoolValue: false,
			},
		},
	}
	s2 := &sinkSender{}
	m.onSetParameters(nil, req2, s2)
	resp2 := s2.last.(*rclif_srv.SetParameters_Response)
	if len(resp2.Results) != 1 || !resp2.Results[0].Successful {
		t.Fatalf("expected success, got %+v", resp2.Results)
	}
	if len(toggles) != 2 || toggles[1] != false {
		t.Fatalf("expected toggle false, got %+v", toggles)
	}
}
