package qos

import "time"

// NOTE: enum values mirror your current rclgo values so C-casts still match.

type HistoryPolicy int

const (
	HistorySystemDefault HistoryPolicy = iota
	HistoryKeepLast
	HistoryKeepAll
	HistoryUnknown
)

type ReliabilityPolicy int

const (
	ReliabilitySystemDefault ReliabilityPolicy = iota
	ReliabilityReliable
	ReliabilityBestEffort
	ReliabilityUnknown
)

type DurabilityPolicy int

const (
	DurabilitySystemDefault DurabilityPolicy = iota
	DurabilityTransientLocal
	DurabilityVolatile
	DurabilityUnknown
)

type LivelinessPolicy int

const (
	LivelinessSystemDefault LivelinessPolicy = iota
	LivelinessAutomatic
	_
	LivelinessManualByTopic
	LivelinessUnknown
)

// Duration defaults match your current constants.
const (
	DurationUnspecified            = time.Duration(0)
	DeadlineDefault                = DurationUnspecified
	LifespanDefault                = DurationUnspecified
	LivelinessLeaseDurationDefault = DurationUnspecified
)

// Profile is the canonical QoS container used across rclgo.
type Profile struct {
	History                      HistoryPolicy     `yaml:"history"`
	Depth                        int               `yaml:"depth"`
	Reliability                  ReliabilityPolicy `yaml:"reliability"`
	Durability                   DurabilityPolicy  `yaml:"durability"`
	Deadline                     time.Duration     `yaml:"deadline"`
	Lifespan                     time.Duration     `yaml:"lifespan"`
	Liveliness                   LivelinessPolicy  `yaml:"liveliness"`
	LivelinessLeaseDuration      time.Duration     `yaml:"liveliness_lease_duration"`
	AvoidROSNamespaceConventions bool              `yaml:"avoid_ros_namespace_conventions"`
}

// Constructors (mirroring your existing helpers)
func NewSystemDefault() Profile {
	return Profile{
		History:                      HistorySystemDefault,
		Depth:                        0,
		Reliability:                  ReliabilitySystemDefault,
		Durability:                   DurabilitySystemDefault,
		Deadline:                     DeadlineDefault,
		Lifespan:                     LifespanDefault,
		Liveliness:                   LivelinessSystemDefault,
		LivelinessLeaseDuration:      LivelinessLeaseDurationDefault,
		AvoidROSNamespaceConventions: false,
	}
}

func NewDefault() Profile { // rclcpp::QoS(10): KeepLast(10), Reliable, Volatile
	return Profile{
		History:     HistoryKeepLast,
		Depth:       10,
		Reliability: ReliabilityReliable,
		Durability:  DurabilityVolatile,
		Deadline:    DeadlineDefault,
		Lifespan:    LifespanDefault,
		Liveliness:  LivelinessSystemDefault,
	}
}

func NewKeepLast(depth int) Profile {
	q := NewDefault()
	q.History = HistoryKeepLast
	q.Depth = depth
	return q
}

func NewKeepAll() Profile {
	q := NewDefault()
	q.History = HistoryKeepAll
	q.Depth = 0
	return q
}

func (p Profile) WithReliability(r ReliabilityPolicy) Profile { p.Reliability = r; return p }
func (p Profile) WithDurability(d DurabilityPolicy) Profile   { p.Durability = d; return p }
func (p Profile) WithHistory(h HistoryPolicy, depth int) Profile {
	p.History = h
	p.Depth = depth
	return p
}
