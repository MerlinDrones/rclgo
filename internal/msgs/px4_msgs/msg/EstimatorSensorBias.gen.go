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

#include <px4_msgs/msg/estimator_sensor_bias.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/EstimatorSensorBias", EstimatorSensorBiasTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/EstimatorSensorBias", EstimatorSensorBiasTypeSupport)
}

type EstimatorSensorBias struct {
	Timestamp         uint64     `yaml:"timestamp"`        // time since system start (microseconds)
	TimestampSample   uint64     `yaml:"timestamp_sample"` // the timestamp of the raw data (microseconds)
	GyroDeviceId      uint32     `yaml:"gyro_device_id"`   // unique device ID for the sensor that does not change between power cycles
	GyroBias          [3]float32 `yaml:"gyro_bias"`        // gyroscope in-run bias in body frame (rad/s)
	GyroBiasLimit     float32    `yaml:"gyro_bias_limit"`  // magnitude of maximum gyroscope in-run bias in body frame (rad/s)
	GyroBiasVariance  [3]float32 `yaml:"gyro_bias_variance"`
	GyroBiasValid     bool       `yaml:"gyro_bias_valid"`
	GyroBiasStable    bool       `yaml:"gyro_bias_stable"` // true when the gyro bias estimate is stable enough to use for calibration
	AccelDeviceId     uint32     `yaml:"accel_device_id"`  // unique device ID for the sensor that does not change between power cycles
	AccelBias         [3]float32 `yaml:"accel_bias"`       // accelerometer in-run bias in body frame (m/s^2)
	AccelBiasLimit    float32    `yaml:"accel_bias_limit"` // magnitude of maximum accelerometer in-run bias in body frame (m/s^2)
	AccelBiasVariance [3]float32 `yaml:"accel_bias_variance"`
	AccelBiasValid    bool       `yaml:"accel_bias_valid"`
	AccelBiasStable   bool       `yaml:"accel_bias_stable"` // true when the accel bias estimate is stable enough to use for calibration
	MagDeviceId       uint32     `yaml:"mag_device_id"`     // unique device ID for the sensor that does not change between power cycles
	MagBias           [3]float32 `yaml:"mag_bias"`          // magnetometer in-run bias in body frame (Gauss)
	MagBiasLimit      float32    `yaml:"mag_bias_limit"`    // magnitude of maximum magnetometer in-run bias in body frame (Gauss)
	MagBiasVariance   [3]float32 `yaml:"mag_bias_variance"`
	MagBiasValid      bool       `yaml:"mag_bias_valid"`
	MagBiasStable     bool       `yaml:"mag_bias_stable"` // true when the mag bias estimate is stable enough to use for calibration
}

// NewEstimatorSensorBias creates a new EstimatorSensorBias with default values.
func NewEstimatorSensorBias() *EstimatorSensorBias {
	self := EstimatorSensorBias{}
	self.SetDefaults()
	return &self
}

func (t *EstimatorSensorBias) Clone() *EstimatorSensorBias {
	c := &EstimatorSensorBias{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.GyroDeviceId = t.GyroDeviceId
	c.GyroBias = t.GyroBias
	c.GyroBiasLimit = t.GyroBiasLimit
	c.GyroBiasVariance = t.GyroBiasVariance
	c.GyroBiasValid = t.GyroBiasValid
	c.GyroBiasStable = t.GyroBiasStable
	c.AccelDeviceId = t.AccelDeviceId
	c.AccelBias = t.AccelBias
	c.AccelBiasLimit = t.AccelBiasLimit
	c.AccelBiasVariance = t.AccelBiasVariance
	c.AccelBiasValid = t.AccelBiasValid
	c.AccelBiasStable = t.AccelBiasStable
	c.MagDeviceId = t.MagDeviceId
	c.MagBias = t.MagBias
	c.MagBiasLimit = t.MagBiasLimit
	c.MagBiasVariance = t.MagBiasVariance
	c.MagBiasValid = t.MagBiasValid
	c.MagBiasStable = t.MagBiasStable
	return c
}

func (t *EstimatorSensorBias) CloneMsg() types.Message {
	return t.Clone()
}

func (t *EstimatorSensorBias) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.GyroDeviceId = 0
	t.GyroBias = [3]float32{}
	t.GyroBiasLimit = 0
	t.GyroBiasVariance = [3]float32{}
	t.GyroBiasValid = false
	t.GyroBiasStable = false
	t.AccelDeviceId = 0
	t.AccelBias = [3]float32{}
	t.AccelBiasLimit = 0
	t.AccelBiasVariance = [3]float32{}
	t.AccelBiasValid = false
	t.AccelBiasStable = false
	t.MagDeviceId = 0
	t.MagBias = [3]float32{}
	t.MagBiasLimit = 0
	t.MagBiasVariance = [3]float32{}
	t.MagBiasValid = false
	t.MagBiasStable = false
}

func (t *EstimatorSensorBias) GetTypeSupport() types.MessageTypeSupport {
	return EstimatorSensorBiasTypeSupport
}

// EstimatorSensorBiasPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type EstimatorSensorBiasPublisher struct {
	*rclgo.Publisher
}

// NewEstimatorSensorBiasPublisher creates and returns a new publisher for the
// EstimatorSensorBias
func NewEstimatorSensorBiasPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*EstimatorSensorBiasPublisher, error) {
	pub, err := node.NewPublisher(topic_name, EstimatorSensorBiasTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &EstimatorSensorBiasPublisher{pub}, nil
}

func (p *EstimatorSensorBiasPublisher) Publish(msg *EstimatorSensorBias) error {
	return p.Publisher.Publish(msg)
}

// EstimatorSensorBiasSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type EstimatorSensorBiasSubscription struct {
	*rclgo.Subscription
}

// EstimatorSensorBiasSubscriptionCallback type is used to provide a subscription
// handler function for a EstimatorSensorBiasSubscription.
type EstimatorSensorBiasSubscriptionCallback func(msg *EstimatorSensorBias, info *rclgo.MessageInfo, err error)

// NewEstimatorSensorBiasSubscription creates and returns a new subscription for the
// EstimatorSensorBias
func NewEstimatorSensorBiasSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback EstimatorSensorBiasSubscriptionCallback) (*EstimatorSensorBiasSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg EstimatorSensorBias
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, EstimatorSensorBiasTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &EstimatorSensorBiasSubscription{sub}, nil
}

func (s *EstimatorSensorBiasSubscription) TakeMessage(out *EstimatorSensorBias) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneEstimatorSensorBiasSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneEstimatorSensorBiasSlice(dst, src []EstimatorSensorBias) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var EstimatorSensorBiasTypeSupport types.MessageTypeSupport = _EstimatorSensorBiasTypeSupport{}

type _EstimatorSensorBiasTypeSupport struct{}

func (t _EstimatorSensorBiasTypeSupport) New() types.Message {
	return NewEstimatorSensorBias()
}

func (t _EstimatorSensorBiasTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__EstimatorSensorBias
	return (unsafe.Pointer)(C.px4_msgs__msg__EstimatorSensorBias__create())
}

func (t _EstimatorSensorBiasTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__EstimatorSensorBias__destroy((*C.px4_msgs__msg__EstimatorSensorBias)(pointer_to_free))
}

func (t _EstimatorSensorBiasTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*EstimatorSensorBias)
	mem := (*C.px4_msgs__msg__EstimatorSensorBias)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	mem.gyro_device_id = C.uint32_t(m.GyroDeviceId)
	cSlice_gyro_bias := mem.gyro_bias[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_gyro_bias)), m.GyroBias[:])
	mem.gyro_bias_limit = C.float(m.GyroBiasLimit)
	cSlice_gyro_bias_variance := mem.gyro_bias_variance[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_gyro_bias_variance)), m.GyroBiasVariance[:])
	mem.gyro_bias_valid = C.bool(m.GyroBiasValid)
	mem.gyro_bias_stable = C.bool(m.GyroBiasStable)
	mem.accel_device_id = C.uint32_t(m.AccelDeviceId)
	cSlice_accel_bias := mem.accel_bias[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_accel_bias)), m.AccelBias[:])
	mem.accel_bias_limit = C.float(m.AccelBiasLimit)
	cSlice_accel_bias_variance := mem.accel_bias_variance[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_accel_bias_variance)), m.AccelBiasVariance[:])
	mem.accel_bias_valid = C.bool(m.AccelBiasValid)
	mem.accel_bias_stable = C.bool(m.AccelBiasStable)
	mem.mag_device_id = C.uint32_t(m.MagDeviceId)
	cSlice_mag_bias := mem.mag_bias[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_mag_bias)), m.MagBias[:])
	mem.mag_bias_limit = C.float(m.MagBiasLimit)
	cSlice_mag_bias_variance := mem.mag_bias_variance[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_mag_bias_variance)), m.MagBiasVariance[:])
	mem.mag_bias_valid = C.bool(m.MagBiasValid)
	mem.mag_bias_stable = C.bool(m.MagBiasStable)
}

func (t _EstimatorSensorBiasTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*EstimatorSensorBias)
	mem := (*C.px4_msgs__msg__EstimatorSensorBias)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	m.GyroDeviceId = uint32(mem.gyro_device_id)
	cSlice_gyro_bias := mem.gyro_bias[:]
	primitives.Float32__Array_to_Go(m.GyroBias[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_gyro_bias)))
	m.GyroBiasLimit = float32(mem.gyro_bias_limit)
	cSlice_gyro_bias_variance := mem.gyro_bias_variance[:]
	primitives.Float32__Array_to_Go(m.GyroBiasVariance[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_gyro_bias_variance)))
	m.GyroBiasValid = bool(mem.gyro_bias_valid)
	m.GyroBiasStable = bool(mem.gyro_bias_stable)
	m.AccelDeviceId = uint32(mem.accel_device_id)
	cSlice_accel_bias := mem.accel_bias[:]
	primitives.Float32__Array_to_Go(m.AccelBias[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_accel_bias)))
	m.AccelBiasLimit = float32(mem.accel_bias_limit)
	cSlice_accel_bias_variance := mem.accel_bias_variance[:]
	primitives.Float32__Array_to_Go(m.AccelBiasVariance[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_accel_bias_variance)))
	m.AccelBiasValid = bool(mem.accel_bias_valid)
	m.AccelBiasStable = bool(mem.accel_bias_stable)
	m.MagDeviceId = uint32(mem.mag_device_id)
	cSlice_mag_bias := mem.mag_bias[:]
	primitives.Float32__Array_to_Go(m.MagBias[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_mag_bias)))
	m.MagBiasLimit = float32(mem.mag_bias_limit)
	cSlice_mag_bias_variance := mem.mag_bias_variance[:]
	primitives.Float32__Array_to_Go(m.MagBiasVariance[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_mag_bias_variance)))
	m.MagBiasValid = bool(mem.mag_bias_valid)
	m.MagBiasStable = bool(mem.mag_bias_stable)
}

func (t _EstimatorSensorBiasTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__EstimatorSensorBias())
}

type CEstimatorSensorBias = C.px4_msgs__msg__EstimatorSensorBias
type CEstimatorSensorBias__Sequence = C.px4_msgs__msg__EstimatorSensorBias__Sequence

func EstimatorSensorBias__Sequence_to_Go(goSlice *[]EstimatorSensorBias, cSlice CEstimatorSensorBias__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]EstimatorSensorBias, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		EstimatorSensorBiasTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func EstimatorSensorBias__Sequence_to_C(cSlice *CEstimatorSensorBias__Sequence, goSlice []EstimatorSensorBias) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__EstimatorSensorBias)(C.malloc(C.sizeof_struct_px4_msgs__msg__EstimatorSensorBias * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		EstimatorSensorBiasTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func EstimatorSensorBias__Array_to_Go(goSlice []EstimatorSensorBias, cSlice []CEstimatorSensorBias) {
	for i := 0; i < len(cSlice); i++ {
		EstimatorSensorBiasTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func EstimatorSensorBias__Array_to_C(cSlice []CEstimatorSensorBias, goSlice []EstimatorSensorBias) {
	for i := 0; i < len(goSlice); i++ {
		EstimatorSensorBiasTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
