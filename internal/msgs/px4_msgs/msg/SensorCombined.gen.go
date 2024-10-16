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

#include <px4_msgs/msg/sensor_combined.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/SensorCombined", SensorCombinedTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/SensorCombined", SensorCombinedTypeSupport)
}

const (
	SensorCombined_RELATIVE_TIMESTAMP_INVALID int32 = 2147483647 // (0x7fffffff) If one of the relative timestamps is set to this value, it means the associated sensor values are invalid
	SensorCombined_CLIPPING_X                 uint8 = 1
	SensorCombined_CLIPPING_Y                 uint8 = 2
	SensorCombined_CLIPPING_Z                 uint8 = 4
)

type SensorCombined struct {
	Timestamp                      uint64     `yaml:"timestamp"`                        // time since system start (microseconds)
	GyroRad                        [3]float32 `yaml:"gyro_rad"`                         // average angular rate measured in the FRD body frame XYZ-axis in rad/s over the last gyro sampling period. gyro timstamp is equal to the timestamp of the message
	GyroIntegralDt                 uint32     `yaml:"gyro_integral_dt"`                 // gyro measurement sampling period in microseconds
	AccelerometerTimestampRelative int32      `yaml:"accelerometer_timestamp_relative"` // timestamp + accelerometer_timestamp_relative = Accelerometer timestamp
	AccelerometerMS2               [3]float32 `yaml:"accelerometer_m_s2"`               // average value acceleration measured in the FRD body frame XYZ-axis in m/s^2 over the last accelerometer sampling period
	AccelerometerIntegralDt        uint32     `yaml:"accelerometer_integral_dt"`        // accelerometer measurement sampling period in microseconds
	AccelerometerClipping          uint8      `yaml:"accelerometer_clipping"`           // bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame
	GyroClipping                   uint8      `yaml:"gyro_clipping"`                    // bitfield indicating if there was any gyro clipping (per axis) during the integration time frame
	AccelCalibrationCount          uint8      `yaml:"accel_calibration_count"`          // Calibration changed counter. Monotonically increases whenever accelermeter calibration changes.
	GyroCalibrationCount           uint8      `yaml:"gyro_calibration_count"`           // Calibration changed counter. Monotonically increases whenever rate gyro calibration changes.
}

// NewSensorCombined creates a new SensorCombined with default values.
func NewSensorCombined() *SensorCombined {
	self := SensorCombined{}
	self.SetDefaults()
	return &self
}

func (t *SensorCombined) Clone() *SensorCombined {
	c := &SensorCombined{}
	c.Timestamp = t.Timestamp
	c.GyroRad = t.GyroRad
	c.GyroIntegralDt = t.GyroIntegralDt
	c.AccelerometerTimestampRelative = t.AccelerometerTimestampRelative
	c.AccelerometerMS2 = t.AccelerometerMS2
	c.AccelerometerIntegralDt = t.AccelerometerIntegralDt
	c.AccelerometerClipping = t.AccelerometerClipping
	c.GyroClipping = t.GyroClipping
	c.AccelCalibrationCount = t.AccelCalibrationCount
	c.GyroCalibrationCount = t.GyroCalibrationCount
	return c
}

func (t *SensorCombined) CloneMsg() types.Message {
	return t.Clone()
}

func (t *SensorCombined) SetDefaults() {
	t.Timestamp = 0
	t.GyroRad = [3]float32{}
	t.GyroIntegralDt = 0
	t.AccelerometerTimestampRelative = 0
	t.AccelerometerMS2 = [3]float32{}
	t.AccelerometerIntegralDt = 0
	t.AccelerometerClipping = 0
	t.GyroClipping = 0
	t.AccelCalibrationCount = 0
	t.GyroCalibrationCount = 0
}

func (t *SensorCombined) GetTypeSupport() types.MessageTypeSupport {
	return SensorCombinedTypeSupport
}

// SensorCombinedPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type SensorCombinedPublisher struct {
	*rclgo.Publisher
}

// NewSensorCombinedPublisher creates and returns a new publisher for the
// SensorCombined
func NewSensorCombinedPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*SensorCombinedPublisher, error) {
	pub, err := node.NewPublisher(topic_name, SensorCombinedTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &SensorCombinedPublisher{pub}, nil
}

func (p *SensorCombinedPublisher) Publish(msg *SensorCombined) error {
	return p.Publisher.Publish(msg)
}

// SensorCombinedSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type SensorCombinedSubscription struct {
	*rclgo.Subscription
}

// SensorCombinedSubscriptionCallback type is used to provide a subscription
// handler function for a SensorCombinedSubscription.
type SensorCombinedSubscriptionCallback func(msg *SensorCombined, info *rclgo.MessageInfo, err error)

// NewSensorCombinedSubscription creates and returns a new subscription for the
// SensorCombined
func NewSensorCombinedSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback SensorCombinedSubscriptionCallback) (*SensorCombinedSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg SensorCombined
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, SensorCombinedTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &SensorCombinedSubscription{sub}, nil
}

func (s *SensorCombinedSubscription) TakeMessage(out *SensorCombined) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneSensorCombinedSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneSensorCombinedSlice(dst, src []SensorCombined) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var SensorCombinedTypeSupport types.MessageTypeSupport = _SensorCombinedTypeSupport{}

type _SensorCombinedTypeSupport struct{}

func (t _SensorCombinedTypeSupport) New() types.Message {
	return NewSensorCombined()
}

func (t _SensorCombinedTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__SensorCombined
	return (unsafe.Pointer)(C.px4_msgs__msg__SensorCombined__create())
}

func (t _SensorCombinedTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__SensorCombined__destroy((*C.px4_msgs__msg__SensorCombined)(pointer_to_free))
}

func (t _SensorCombinedTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*SensorCombined)
	mem := (*C.px4_msgs__msg__SensorCombined)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	cSlice_gyro_rad := mem.gyro_rad[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_gyro_rad)), m.GyroRad[:])
	mem.gyro_integral_dt = C.uint32_t(m.GyroIntegralDt)
	mem.accelerometer_timestamp_relative = C.int32_t(m.AccelerometerTimestampRelative)
	cSlice_accelerometer_m_s2 := mem.accelerometer_m_s2[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_accelerometer_m_s2)), m.AccelerometerMS2[:])
	mem.accelerometer_integral_dt = C.uint32_t(m.AccelerometerIntegralDt)
	mem.accelerometer_clipping = C.uint8_t(m.AccelerometerClipping)
	mem.gyro_clipping = C.uint8_t(m.GyroClipping)
	mem.accel_calibration_count = C.uint8_t(m.AccelCalibrationCount)
	mem.gyro_calibration_count = C.uint8_t(m.GyroCalibrationCount)
}

func (t _SensorCombinedTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*SensorCombined)
	mem := (*C.px4_msgs__msg__SensorCombined)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	cSlice_gyro_rad := mem.gyro_rad[:]
	primitives.Float32__Array_to_Go(m.GyroRad[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_gyro_rad)))
	m.GyroIntegralDt = uint32(mem.gyro_integral_dt)
	m.AccelerometerTimestampRelative = int32(mem.accelerometer_timestamp_relative)
	cSlice_accelerometer_m_s2 := mem.accelerometer_m_s2[:]
	primitives.Float32__Array_to_Go(m.AccelerometerMS2[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_accelerometer_m_s2)))
	m.AccelerometerIntegralDt = uint32(mem.accelerometer_integral_dt)
	m.AccelerometerClipping = uint8(mem.accelerometer_clipping)
	m.GyroClipping = uint8(mem.gyro_clipping)
	m.AccelCalibrationCount = uint8(mem.accel_calibration_count)
	m.GyroCalibrationCount = uint8(mem.gyro_calibration_count)
}

func (t _SensorCombinedTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__SensorCombined())
}

type CSensorCombined = C.px4_msgs__msg__SensorCombined
type CSensorCombined__Sequence = C.px4_msgs__msg__SensorCombined__Sequence

func SensorCombined__Sequence_to_Go(goSlice *[]SensorCombined, cSlice CSensorCombined__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]SensorCombined, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		SensorCombinedTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func SensorCombined__Sequence_to_C(cSlice *CSensorCombined__Sequence, goSlice []SensorCombined) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__SensorCombined)(C.malloc(C.sizeof_struct_px4_msgs__msg__SensorCombined * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		SensorCombinedTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func SensorCombined__Array_to_Go(goSlice []SensorCombined, cSlice []CSensorCombined) {
	for i := 0; i < len(cSlice); i++ {
		SensorCombinedTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func SensorCombined__Array_to_C(cSlice []CSensorCombined, goSlice []SensorCombined) {
	for i := 0; i < len(goSlice); i++ {
		SensorCombinedTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
