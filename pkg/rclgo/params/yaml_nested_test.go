package params

import "testing"

// Nested maps under ros__parameters are flattened into dotted names, as in
// rclcpp and rclpy.
func TestLoadYAMLNestedMaps(t *testing.T) {
	m := managerWithYAML(t, `  log_level: info
  ipc:
    port: 5600
    host: "10.0.0.1"
    retry:
      count: 3
      backoff: 0.5
  election:
    enabled: true
    peers: [1, 2, 3]
`)
	want := map[string]Value{
		"log_level":         {Kind: KindString, Str: "info"},
		"ipc.port":          {Kind: KindInt64, Int64: 5600},
		"ipc.host":          {Kind: KindString, Str: "10.0.0.1"},
		"ipc.retry.count":   {Kind: KindInt64, Int64: 3},
		"ipc.retry.backoff": {Kind: KindDouble, Double: 0.5},
		"election.enabled":  {Kind: KindBool, Bool: true},
		"election.peers":    {Kind: KindInt64Array, Int64s: []int64{1, 2, 3}},
	}
	for name, w := range want {
		p, ok := m.Get(name)
		if !ok {
			t.Errorf("%s not loaded", name)
			continue
		}
		if p.Value.String() != w.String() || p.Value.Kind != w.Kind {
			t.Errorf("%s = %v (kind %d), want %v (kind %d)", name, p.Value, p.Value.Kind, w, w.Kind)
		}
	}
	if got := len(m.List()); got != len(want) {
		t.Errorf("loaded %d parameters, want %d: %v", got, len(want), m.List())
	}
}
