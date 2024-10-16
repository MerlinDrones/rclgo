// Code generated by rclgo-gen. DO NOT EDIT.

package px4_msgs_msg

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <px4_msgs/msg/sensor_hygrometer.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/SensorHygrometer", SensorHygrometerTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/SensorHygrometer", SensorHygrometerTypeSupport)
}

type SensorHygrometer struct {
	Timestamp       uint64  `yaml:"timestamp"` // time since system start (microseconds)
	TimestampSample uint64  `yaml:"timestamp_sample"`
	DeviceId        uint32  `yaml:"device_id"`   // unique device ID for the sensor that does not change between power cycles
	Temperature     float32 `yaml:"temperature"` // Temperature provided by sensor (Celsius)
	Humidity        float32 `yaml:"humidity"`    // Humidity provided by sensor
}

// NewSensorHygrometer creates a new SensorHygrometer with default values.
func NewSensorHygrometer() *SensorHygrometer {
	self := SensorHygrometer{}
	self.SetDefaults()
	return &self
}

func (t *SensorHygrometer) Clone() *SensorHygrometer {
	c := &SensorHygrometer{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.DeviceId = t.DeviceId
	c.Temperature = t.Temperature
	c.Humidity = t.Humidity
	return c
}

func (t *SensorHygrometer) CloneMsg() types.Message {
	return t.Clone()
}

func (t *SensorHygrometer) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.DeviceId = 0
	t.Temperature = 0
	t.Humidity = 0
}

func (t *SensorHygrometer) GetTypeSupport() types.MessageTypeSupport {
	return SensorHygrometerTypeSupport
}

// SensorHygrometerPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type SensorHygrometerPublisher struct {
	*rclgo.Publisher
}

// NewSensorHygrometerPublisher creates and returns a new publisher for the
// SensorHygrometer
func NewSensorHygrometerPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*SensorHygrometerPublisher, error) {
	pub, err := node.NewPublisher(topic_name, SensorHygrometerTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &SensorHygrometerPublisher{pub}, nil
}

func (p *SensorHygrometerPublisher) Publish(msg *SensorHygrometer) error {
	return p.Publisher.Publish(msg)
}

// SensorHygrometerSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type SensorHygrometerSubscription struct {
	*rclgo.Subscription
}

// SensorHygrometerSubscriptionCallback type is used to provide a subscription
// handler function for a SensorHygrometerSubscription.
type SensorHygrometerSubscriptionCallback func(msg *SensorHygrometer, info *rclgo.MessageInfo, err error)

// NewSensorHygrometerSubscription creates and returns a new subscription for the
// SensorHygrometer
func NewSensorHygrometerSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback SensorHygrometerSubscriptionCallback) (*SensorHygrometerSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg SensorHygrometer
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, SensorHygrometerTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &SensorHygrometerSubscription{sub}, nil
}

func (s *SensorHygrometerSubscription) TakeMessage(out *SensorHygrometer) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneSensorHygrometerSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneSensorHygrometerSlice(dst, src []SensorHygrometer) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var SensorHygrometerTypeSupport types.MessageTypeSupport = _SensorHygrometerTypeSupport{}

type _SensorHygrometerTypeSupport struct{}

func (t _SensorHygrometerTypeSupport) New() types.Message {
	return NewSensorHygrometer()
}

func (t _SensorHygrometerTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__SensorHygrometer
	return (unsafe.Pointer)(C.px4_msgs__msg__SensorHygrometer__create())
}

func (t _SensorHygrometerTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__SensorHygrometer__destroy((*C.px4_msgs__msg__SensorHygrometer)(pointer_to_free))
}

func (t _SensorHygrometerTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*SensorHygrometer)
	mem := (*C.px4_msgs__msg__SensorHygrometer)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	mem.device_id = C.uint32_t(m.DeviceId)
	mem.temperature = C.float(m.Temperature)
	mem.humidity = C.float(m.Humidity)
}

func (t _SensorHygrometerTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*SensorHygrometer)
	mem := (*C.px4_msgs__msg__SensorHygrometer)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	m.DeviceId = uint32(mem.device_id)
	m.Temperature = float32(mem.temperature)
	m.Humidity = float32(mem.humidity)
}

func (t _SensorHygrometerTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__SensorHygrometer())
}

type CSensorHygrometer = C.px4_msgs__msg__SensorHygrometer
type CSensorHygrometer__Sequence = C.px4_msgs__msg__SensorHygrometer__Sequence

func SensorHygrometer__Sequence_to_Go(goSlice *[]SensorHygrometer, cSlice CSensorHygrometer__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]SensorHygrometer, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		SensorHygrometerTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func SensorHygrometer__Sequence_to_C(cSlice *CSensorHygrometer__Sequence, goSlice []SensorHygrometer) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__SensorHygrometer)(C.malloc(C.sizeof_struct_px4_msgs__msg__SensorHygrometer * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		SensorHygrometerTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func SensorHygrometer__Array_to_Go(goSlice []SensorHygrometer, cSlice []CSensorHygrometer) {
	for i := 0; i < len(cSlice); i++ {
		SensorHygrometerTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func SensorHygrometer__Array_to_C(cSlice []CSensorHygrometer, goSlice []SensorHygrometer) {
	for i := 0; i < len(goSlice); i++ {
		SensorHygrometerTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
