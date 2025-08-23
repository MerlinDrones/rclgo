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

// Parameter events: KeepAll, Reliable, TransientLocal
func NewParameterEventsProfile() Profile {
	return Profile{
		History:     HistoryKeepAll,
		Depth:       0,
		Reliability: ReliabilityReliable,
		Durability:  DurabilityTransientLocal,
		Liveliness:  LivelinessAutomatic,
	}
}
