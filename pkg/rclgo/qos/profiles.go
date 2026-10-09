package qos

// rclcpp::ClockQoS — KeepLast(1), BestEffort, Volatile
func NewClockProfile() Profile {
	return Profile{
		History:     HistoryKeepLast,
		Depth:       1,
		Reliability: ReliabilityBestEffort,
		Durability:  DurabilityVolatile,
		Liveliness:  LivelinessAutomatic,
	}
}

// rclcpp::SensorDataQoS — KeepLast(5), BestEffort, Volatile
func NewSensorDataProfile() Profile {
	return Profile{
		History:     HistoryKeepLast,
		Depth:       5,
		Reliability: ReliabilityBestEffort,
		Durability:  DurabilityVolatile,
		Liveliness:  LivelinessAutomatic,
	}
}

// NewBestAvailableProfile matches rmw_qos_profile_best_available (Jazzy+).
// Reliability, durability, and liveliness are chosen at creation time to match
// the majority of discovered endpoints.
func NewBestAvailableProfile() Profile {
	return Profile{
		History:     HistoryKeepLast,
		Depth:       10,
		Reliability: ReliabilityBestAvailable,
		Durability:  DurabilityBestAvailable,
		Liveliness:  LivelinessBestAvailable,
	}
}

// Parameter events: KeepLast(1000), Reliable, Volatile (rmw_qos_profile_parameter_events)
func NewParameterEventsProfile() Profile {
	// rmw_qos_profile_parameter_events: late subscribers get no history, as
	// with rclcpp and rclpy nodes.
	return Profile{
		History:     HistoryKeepLast,
		Depth:       1000,
		Reliability: ReliabilityReliable,
		Durability:  DurabilityVolatile,
		Liveliness:  LivelinessSystemDefault,
	}
}
