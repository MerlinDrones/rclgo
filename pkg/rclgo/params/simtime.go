package params

// EnableUseSimTimeHook calls apply(enabled) whenever use_sim_time changes,
// declaring the parameter if needed. It does not replace an OnSet callback.
func (m *Manager) EnableUseSimTimeHook(apply func(enabled bool)) {
	_ = m.DeclareIfMissing("use_sim_time", Value{Kind: KindBool, Bool: false}, Descriptor{Description: "Use /clock for ROS time"})
	m.AddPostSetCallback(func(ps []Parameter) {
		for _, p := range ps {
			if p.Name == "use_sim_time" && p.Value.Kind == KindBool {
				apply(p.Value.Bool)
			}
		}
	})
}
