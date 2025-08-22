package rclgo

// NewClockQosProfile returns the standard ROS 2 QoS profile for /clock.
// Matches rclcpp::ClockQoS (KeepLast(1), BestEffort, Volatile).
func NewClockQosProfile() QosProfile {
	return QosProfile{
		History:                      HistoryKeepLast,
		Depth:                        1,
		Reliability:                  ReliabilityBestEffort,
		Durability:                   DurabilityVolatile,
		Lifespan:                     0,
		Deadline:                     0,
		Liveliness:                   LivelinessAutomatic,
		LivelinessLeaseDuration:      0,
		AvoidRosNamespaceConventions: false,
	}
}

// NewSensorDataQosProfile returns the standard ROS 2 QoS for high-rate sensors
// (e.g., images, point clouds, IMU). Mirrors rclcpp::SensorDataQoS:
// KeepLast(5), BestEffort, Volatile.
// NewSensorDataQosProfile matches rclcpp::SensorDataQoS:
// KeepLast(5), BestEffort, Volatile.
func NewSensorDataQosProfile() QosProfile {
	return QosProfile{
		History:                      HistoryKeepLast,
		Depth:                        5,
		Reliability:                  ReliabilityBestEffort,
		Durability:                   DurabilityVolatile,
		Deadline:                     DeadlineDefault,
		Lifespan:                     LifespanDefault,
		Liveliness:                   LivelinessAutomatic,
		LivelinessLeaseDuration:      LivelinessLeaseDurationDefault,
		AvoidRosNamespaceConventions: false,
	}
}
