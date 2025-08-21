package params

// Hook for `/use_sim_time`. Calls apply(enabled) whenever toggled.

func (m *Manager) EnableUseSimTimeHook(apply func(enabled bool)) {
	_, _ = m.Declare("use_sim_time", Value{Kind: KindBool, Bool: false}, Descriptor{Description: "Use /clock for ROS time"})
	m.OnSet(func(ps []Parameter) SetResult {
		for _, p := range ps {
			if p.Name == "use_sim_time" && p.Value.Kind == KindBool {
				apply(p.Value.Bool)
			}
		}
		return SetResult{Successful: true}
	})
}
