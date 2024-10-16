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

#include <px4_msgs/msg/sensor_accel.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/SensorAccel", SensorAccelTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/SensorAccel", SensorAccelTypeSupport)
}

const (
	SensorAccel_ORB_QUEUE_LENGTH uint8 = 8
)

type SensorAccel struct {
	Timestamp       uint64   `yaml:"timestamp"` // time since system start (microseconds)
	TimestampSample uint64   `yaml:"timestamp_sample"`
	DeviceId        uint32   `yaml:"device_id"`   // unique device ID for the sensor that does not change between power cycles
	X               float32  `yaml:"x"`           // acceleration in the FRD board frame X-axis in m/s^2
	Y               float32  `yaml:"y"`           // acceleration in the FRD board frame Y-axis in m/s^2
	Z               float32  `yaml:"z"`           // acceleration in the FRD board frame Z-axis in m/s^2
	Temperature     float32  `yaml:"temperature"` // temperature in degrees Celsius
	ErrorCount      uint32   `yaml:"error_count"`
	ClipCounter     [3]uint8 `yaml:"clip_counter"` // clip count per axis in the sample period
	Samples         uint8    `yaml:"samples"`      // number of raw samples that went into this message
}

// NewSensorAccel creates a new SensorAccel with default values.
func NewSensorAccel() *SensorAccel {
	self := SensorAccel{}
	self.SetDefaults()
	return &self
}

func (t *SensorAccel) Clone() *SensorAccel {
	c := &SensorAccel{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.DeviceId = t.DeviceId
	c.X = t.X
	c.Y = t.Y
	c.Z = t.Z
	c.Temperature = t.Temperature
	c.ErrorCount = t.ErrorCount
	c.ClipCounter = t.ClipCounter
	c.Samples = t.Samples
	return c
}

func (t *SensorAccel) CloneMsg() types.Message {
	return t.Clone()
}

func (t *SensorAccel) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.DeviceId = 0
	t.X = 0
	t.Y = 0
	t.Z = 0
	t.Temperature = 0
	t.ErrorCount = 0
	t.ClipCounter = [3]uint8{}
	t.Samples = 0
}

func (t *SensorAccel) GetTypeSupport() types.MessageTypeSupport {
	return SensorAccelTypeSupport
}

// SensorAccelPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type SensorAccelPublisher struct {
	*rclgo.Publisher
}

// NewSensorAccelPublisher creates and returns a new publisher for the
// SensorAccel
func NewSensorAccelPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*SensorAccelPublisher, error) {
	pub, err := node.NewPublisher(topic_name, SensorAccelTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &SensorAccelPublisher{pub}, nil
}

func (p *SensorAccelPublisher) Publish(msg *SensorAccel) error {
	return p.Publisher.Publish(msg)
}

// SensorAccelSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type SensorAccelSubscription struct {
	*rclgo.Subscription
}

// SensorAccelSubscriptionCallback type is used to provide a subscription
// handler function for a SensorAccelSubscription.
type SensorAccelSubscriptionCallback func(msg *SensorAccel, info *rclgo.MessageInfo, err error)

// NewSensorAccelSubscription creates and returns a new subscription for the
// SensorAccel
func NewSensorAccelSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback SensorAccelSubscriptionCallback) (*SensorAccelSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg SensorAccel
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, SensorAccelTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &SensorAccelSubscription{sub}, nil
}

func (s *SensorAccelSubscription) TakeMessage(out *SensorAccel) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneSensorAccelSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneSensorAccelSlice(dst, src []SensorAccel) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var SensorAccelTypeSupport types.MessageTypeSupport = _SensorAccelTypeSupport{}

type _SensorAccelTypeSupport struct{}

func (t _SensorAccelTypeSupport) New() types.Message {
	return NewSensorAccel()
}

func (t _SensorAccelTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__SensorAccel
	return (unsafe.Pointer)(C.px4_msgs__msg__SensorAccel__create())
}

func (t _SensorAccelTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__SensorAccel__destroy((*C.px4_msgs__msg__SensorAccel)(pointer_to_free))
}

func (t _SensorAccelTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*SensorAccel)
	mem := (*C.px4_msgs__msg__SensorAccel)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	mem.device_id = C.uint32_t(m.DeviceId)
	mem.x = C.float(m.X)
	mem.y = C.float(m.Y)
	mem.z = C.float(m.Z)
	mem.temperature = C.float(m.Temperature)
	mem.error_count = C.uint32_t(m.ErrorCount)
	cSlice_clip_counter := mem.clip_counter[:]
	primitives.Uint8__Array_to_C(*(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_clip_counter)), m.ClipCounter[:])
	mem.samples = C.uint8_t(m.Samples)
}

func (t _SensorAccelTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*SensorAccel)
	mem := (*C.px4_msgs__msg__SensorAccel)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	m.DeviceId = uint32(mem.device_id)
	m.X = float32(mem.x)
	m.Y = float32(mem.y)
	m.Z = float32(mem.z)
	m.Temperature = float32(mem.temperature)
	m.ErrorCount = uint32(mem.error_count)
	cSlice_clip_counter := mem.clip_counter[:]
	primitives.Uint8__Array_to_Go(m.ClipCounter[:], *(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_clip_counter)))
	m.Samples = uint8(mem.samples)
}

func (t _SensorAccelTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__SensorAccel())
}

type CSensorAccel = C.px4_msgs__msg__SensorAccel
type CSensorAccel__Sequence = C.px4_msgs__msg__SensorAccel__Sequence

func SensorAccel__Sequence_to_Go(goSlice *[]SensorAccel, cSlice CSensorAccel__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]SensorAccel, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		SensorAccelTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func SensorAccel__Sequence_to_C(cSlice *CSensorAccel__Sequence, goSlice []SensorAccel) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__SensorAccel)(C.malloc(C.sizeof_struct_px4_msgs__msg__SensorAccel * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		SensorAccelTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func SensorAccel__Array_to_Go(goSlice []SensorAccel, cSlice []CSensorAccel) {
	for i := 0; i < len(cSlice); i++ {
		SensorAccelTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func SensorAccel__Array_to_C(cSlice []CSensorAccel, goSlice []SensorAccel) {
	for i := 0; i < len(goSlice); i++ {
		SensorAccelTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
