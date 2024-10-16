// Code generated by rclgo-gen. DO NOT EDIT.

package px4_msgs_msg

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	primitives "github.com/merlindrones/rclgo/pkg/rclgo/primitives"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <px4_msgs/msg/sensor_gnss_relative.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/SensorGnssRelative", SensorGnssRelativeTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/SensorGnssRelative", SensorGnssRelativeTypeSupport)
}

type SensorGnssRelative struct {
	Timestamp                  uint64     `yaml:"timestamp"`             // time since system start (microseconds)
	TimestampSample            uint64     `yaml:"timestamp_sample"`      // time since system start (microseconds)
	DeviceId                   uint32     `yaml:"device_id"`             // unique device ID for the sensor that does not change between power cycles
	TimeUtcUsec                uint64     `yaml:"time_utc_usec"`         // Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0
	ReferenceStationId         uint16     `yaml:"reference_station_id"`  // Reference Station ID
	Position                   [3]float32 `yaml:"position"`              // GPS NED relative position vector (m)
	PositionAccuracy           [3]float32 `yaml:"position_accuracy"`     // Accuracy of relative position (m)
	Heading                    float32    `yaml:"heading"`               // Heading of the relative position vector (radians)
	HeadingAccuracy            float32    `yaml:"heading_accuracy"`      // Accuracy of heading of the relative position vector (radians)
	PositionLength             float32    `yaml:"position_length"`       // Length of the position vector (m)
	AccuracyLength             float32    `yaml:"accuracy_length"`       // Accuracy of the position length (m)
	GnssFixOk                  bool       `yaml:"gnss_fix_ok"`           // GNSS valid fix (i.e within DOP & accuracy masks)
	DifferentialSolution       bool       `yaml:"differential_solution"` // differential corrections were applied
	RelativePositionValid      bool       `yaml:"relative_position_valid"`
	CarrierSolutionFloating    bool       `yaml:"carrier_solution_floating"`   // carrier phase range solution with floating ambiguities
	CarrierSolutionFixed       bool       `yaml:"carrier_solution_fixed"`      // carrier phase range solution with fixed ambiguities
	MovingBaseMode             bool       `yaml:"moving_base_mode"`            // if the receiver is operating in moving base mode
	ReferencePositionMiss      bool       `yaml:"reference_position_miss"`     // extrapolated reference position was used to compute moving base solution this epoch
	ReferenceObservationsMiss  bool       `yaml:"reference_observations_miss"` // extrapolated reference observations were used to compute moving base solution this epoch
	HeadingValid               bool       `yaml:"heading_valid"`
	RelativePositionNormalized bool       `yaml:"relative_position_normalized"` // the components of the relative position vector (including the high-precision parts) are normalized
}

// NewSensorGnssRelative creates a new SensorGnssRelative with default values.
func NewSensorGnssRelative() *SensorGnssRelative {
	self := SensorGnssRelative{}
	self.SetDefaults()
	return &self
}

func (t *SensorGnssRelative) Clone() *SensorGnssRelative {
	c := &SensorGnssRelative{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.DeviceId = t.DeviceId
	c.TimeUtcUsec = t.TimeUtcUsec
	c.ReferenceStationId = t.ReferenceStationId
	c.Position = t.Position
	c.PositionAccuracy = t.PositionAccuracy
	c.Heading = t.Heading
	c.HeadingAccuracy = t.HeadingAccuracy
	c.PositionLength = t.PositionLength
	c.AccuracyLength = t.AccuracyLength
	c.GnssFixOk = t.GnssFixOk
	c.DifferentialSolution = t.DifferentialSolution
	c.RelativePositionValid = t.RelativePositionValid
	c.CarrierSolutionFloating = t.CarrierSolutionFloating
	c.CarrierSolutionFixed = t.CarrierSolutionFixed
	c.MovingBaseMode = t.MovingBaseMode
	c.ReferencePositionMiss = t.ReferencePositionMiss
	c.ReferenceObservationsMiss = t.ReferenceObservationsMiss
	c.HeadingValid = t.HeadingValid
	c.RelativePositionNormalized = t.RelativePositionNormalized
	return c
}

func (t *SensorGnssRelative) CloneMsg() types.Message {
	return t.Clone()
}

func (t *SensorGnssRelative) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.DeviceId = 0
	t.TimeUtcUsec = 0
	t.ReferenceStationId = 0
	t.Position = [3]float32{}
	t.PositionAccuracy = [3]float32{}
	t.Heading = 0
	t.HeadingAccuracy = 0
	t.PositionLength = 0
	t.AccuracyLength = 0
	t.GnssFixOk = false
	t.DifferentialSolution = false
	t.RelativePositionValid = false
	t.CarrierSolutionFloating = false
	t.CarrierSolutionFixed = false
	t.MovingBaseMode = false
	t.ReferencePositionMiss = false
	t.ReferenceObservationsMiss = false
	t.HeadingValid = false
	t.RelativePositionNormalized = false
}

func (t *SensorGnssRelative) GetTypeSupport() types.MessageTypeSupport {
	return SensorGnssRelativeTypeSupport
}

// SensorGnssRelativePublisher wraps rclgo.Publisher to provide type safe helper
// functions
type SensorGnssRelativePublisher struct {
	*rclgo.Publisher
}

// NewSensorGnssRelativePublisher creates and returns a new publisher for the
// SensorGnssRelative
func NewSensorGnssRelativePublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*SensorGnssRelativePublisher, error) {
	pub, err := node.NewPublisher(topic_name, SensorGnssRelativeTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &SensorGnssRelativePublisher{pub}, nil
}

func (p *SensorGnssRelativePublisher) Publish(msg *SensorGnssRelative) error {
	return p.Publisher.Publish(msg)
}

// SensorGnssRelativeSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type SensorGnssRelativeSubscription struct {
	*rclgo.Subscription
}

// SensorGnssRelativeSubscriptionCallback type is used to provide a subscription
// handler function for a SensorGnssRelativeSubscription.
type SensorGnssRelativeSubscriptionCallback func(msg *SensorGnssRelative, info *rclgo.MessageInfo, err error)

// NewSensorGnssRelativeSubscription creates and returns a new subscription for the
// SensorGnssRelative
func NewSensorGnssRelativeSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback SensorGnssRelativeSubscriptionCallback) (*SensorGnssRelativeSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg SensorGnssRelative
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, SensorGnssRelativeTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &SensorGnssRelativeSubscription{sub}, nil
}

func (s *SensorGnssRelativeSubscription) TakeMessage(out *SensorGnssRelative) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneSensorGnssRelativeSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneSensorGnssRelativeSlice(dst, src []SensorGnssRelative) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var SensorGnssRelativeTypeSupport types.MessageTypeSupport = _SensorGnssRelativeTypeSupport{}

type _SensorGnssRelativeTypeSupport struct{}

func (t _SensorGnssRelativeTypeSupport) New() types.Message {
	return NewSensorGnssRelative()
}

func (t _SensorGnssRelativeTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__SensorGnssRelative
	return (unsafe.Pointer)(C.px4_msgs__msg__SensorGnssRelative__create())
}

func (t _SensorGnssRelativeTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__SensorGnssRelative__destroy((*C.px4_msgs__msg__SensorGnssRelative)(pointer_to_free))
}

func (t _SensorGnssRelativeTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*SensorGnssRelative)
	mem := (*C.px4_msgs__msg__SensorGnssRelative)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	mem.device_id = C.uint32_t(m.DeviceId)
	mem.time_utc_usec = C.uint64_t(m.TimeUtcUsec)
	mem.reference_station_id = C.uint16_t(m.ReferenceStationId)
	cSlice_position := mem.position[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_position)), m.Position[:])
	cSlice_position_accuracy := mem.position_accuracy[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_position_accuracy)), m.PositionAccuracy[:])
	mem.heading = C.float(m.Heading)
	mem.heading_accuracy = C.float(m.HeadingAccuracy)
	mem.position_length = C.float(m.PositionLength)
	mem.accuracy_length = C.float(m.AccuracyLength)
	mem.gnss_fix_ok = C.bool(m.GnssFixOk)
	mem.differential_solution = C.bool(m.DifferentialSolution)
	mem.relative_position_valid = C.bool(m.RelativePositionValid)
	mem.carrier_solution_floating = C.bool(m.CarrierSolutionFloating)
	mem.carrier_solution_fixed = C.bool(m.CarrierSolutionFixed)
	mem.moving_base_mode = C.bool(m.MovingBaseMode)
	mem.reference_position_miss = C.bool(m.ReferencePositionMiss)
	mem.reference_observations_miss = C.bool(m.ReferenceObservationsMiss)
	mem.heading_valid = C.bool(m.HeadingValid)
	mem.relative_position_normalized = C.bool(m.RelativePositionNormalized)
}

func (t _SensorGnssRelativeTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*SensorGnssRelative)
	mem := (*C.px4_msgs__msg__SensorGnssRelative)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	m.DeviceId = uint32(mem.device_id)
	m.TimeUtcUsec = uint64(mem.time_utc_usec)
	m.ReferenceStationId = uint16(mem.reference_station_id)
	cSlice_position := mem.position[:]
	primitives.Float32__Array_to_Go(m.Position[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_position)))
	cSlice_position_accuracy := mem.position_accuracy[:]
	primitives.Float32__Array_to_Go(m.PositionAccuracy[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_position_accuracy)))
	m.Heading = float32(mem.heading)
	m.HeadingAccuracy = float32(mem.heading_accuracy)
	m.PositionLength = float32(mem.position_length)
	m.AccuracyLength = float32(mem.accuracy_length)
	m.GnssFixOk = bool(mem.gnss_fix_ok)
	m.DifferentialSolution = bool(mem.differential_solution)
	m.RelativePositionValid = bool(mem.relative_position_valid)
	m.CarrierSolutionFloating = bool(mem.carrier_solution_floating)
	m.CarrierSolutionFixed = bool(mem.carrier_solution_fixed)
	m.MovingBaseMode = bool(mem.moving_base_mode)
	m.ReferencePositionMiss = bool(mem.reference_position_miss)
	m.ReferenceObservationsMiss = bool(mem.reference_observations_miss)
	m.HeadingValid = bool(mem.heading_valid)
	m.RelativePositionNormalized = bool(mem.relative_position_normalized)
}

func (t _SensorGnssRelativeTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__SensorGnssRelative())
}

type CSensorGnssRelative = C.px4_msgs__msg__SensorGnssRelative
type CSensorGnssRelative__Sequence = C.px4_msgs__msg__SensorGnssRelative__Sequence

func SensorGnssRelative__Sequence_to_Go(goSlice *[]SensorGnssRelative, cSlice CSensorGnssRelative__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]SensorGnssRelative, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		SensorGnssRelativeTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func SensorGnssRelative__Sequence_to_C(cSlice *CSensorGnssRelative__Sequence, goSlice []SensorGnssRelative) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__SensorGnssRelative)(C.malloc(C.sizeof_struct_px4_msgs__msg__SensorGnssRelative * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		SensorGnssRelativeTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func SensorGnssRelative__Array_to_Go(goSlice []SensorGnssRelative, cSlice []CSensorGnssRelative) {
	for i := 0; i < len(cSlice); i++ {
		SensorGnssRelativeTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func SensorGnssRelative__Array_to_C(cSlice []CSensorGnssRelative, goSlice []SensorGnssRelative) {
	for i := 0; i < len(goSlice); i++ {
		SensorGnssRelativeTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
