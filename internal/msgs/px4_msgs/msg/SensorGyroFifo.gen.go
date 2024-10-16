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

#include <px4_msgs/msg/sensor_gyro_fifo.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/SensorGyroFifo", SensorGyroFifoTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/SensorGyroFifo", SensorGyroFifoTypeSupport)
}

const (
	SensorGyroFifo_ORB_QUEUE_LENGTH uint8 = 4
)

type SensorGyroFifo struct {
	Timestamp       uint64    `yaml:"timestamp"` // time since system start (microseconds)
	TimestampSample uint64    `yaml:"timestamp_sample"`
	DeviceId        uint32    `yaml:"device_id"` // unique device ID for the sensor that does not change between power cycles
	Dt              float32   `yaml:"dt"`        // delta time between samples (microseconds)
	Scale           float32   `yaml:"scale"`
	Samples         uint8     `yaml:"samples"` // number of valid samples
	X               [32]int16 `yaml:"x"`       // angular velocity in the FRD board frame X-axis in rad/s
	Y               [32]int16 `yaml:"y"`       // angular velocity in the FRD board frame Y-axis in rad/s
	Z               [32]int16 `yaml:"z"`       // angular velocity in the FRD board frame Z-axis in rad/s
}

// NewSensorGyroFifo creates a new SensorGyroFifo with default values.
func NewSensorGyroFifo() *SensorGyroFifo {
	self := SensorGyroFifo{}
	self.SetDefaults()
	return &self
}

func (t *SensorGyroFifo) Clone() *SensorGyroFifo {
	c := &SensorGyroFifo{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.DeviceId = t.DeviceId
	c.Dt = t.Dt
	c.Scale = t.Scale
	c.Samples = t.Samples
	c.X = t.X
	c.Y = t.Y
	c.Z = t.Z
	return c
}

func (t *SensorGyroFifo) CloneMsg() types.Message {
	return t.Clone()
}

func (t *SensorGyroFifo) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.DeviceId = 0
	t.Dt = 0
	t.Scale = 0
	t.Samples = 0
	t.X = [32]int16{}
	t.Y = [32]int16{}
	t.Z = [32]int16{}
}

func (t *SensorGyroFifo) GetTypeSupport() types.MessageTypeSupport {
	return SensorGyroFifoTypeSupport
}

// SensorGyroFifoPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type SensorGyroFifoPublisher struct {
	*rclgo.Publisher
}

// NewSensorGyroFifoPublisher creates and returns a new publisher for the
// SensorGyroFifo
func NewSensorGyroFifoPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*SensorGyroFifoPublisher, error) {
	pub, err := node.NewPublisher(topic_name, SensorGyroFifoTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &SensorGyroFifoPublisher{pub}, nil
}

func (p *SensorGyroFifoPublisher) Publish(msg *SensorGyroFifo) error {
	return p.Publisher.Publish(msg)
}

// SensorGyroFifoSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type SensorGyroFifoSubscription struct {
	*rclgo.Subscription
}

// SensorGyroFifoSubscriptionCallback type is used to provide a subscription
// handler function for a SensorGyroFifoSubscription.
type SensorGyroFifoSubscriptionCallback func(msg *SensorGyroFifo, info *rclgo.MessageInfo, err error)

// NewSensorGyroFifoSubscription creates and returns a new subscription for the
// SensorGyroFifo
func NewSensorGyroFifoSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback SensorGyroFifoSubscriptionCallback) (*SensorGyroFifoSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg SensorGyroFifo
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, SensorGyroFifoTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &SensorGyroFifoSubscription{sub}, nil
}

func (s *SensorGyroFifoSubscription) TakeMessage(out *SensorGyroFifo) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneSensorGyroFifoSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneSensorGyroFifoSlice(dst, src []SensorGyroFifo) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var SensorGyroFifoTypeSupport types.MessageTypeSupport = _SensorGyroFifoTypeSupport{}

type _SensorGyroFifoTypeSupport struct{}

func (t _SensorGyroFifoTypeSupport) New() types.Message {
	return NewSensorGyroFifo()
}

func (t _SensorGyroFifoTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__SensorGyroFifo
	return (unsafe.Pointer)(C.px4_msgs__msg__SensorGyroFifo__create())
}

func (t _SensorGyroFifoTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__SensorGyroFifo__destroy((*C.px4_msgs__msg__SensorGyroFifo)(pointer_to_free))
}

func (t _SensorGyroFifoTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*SensorGyroFifo)
	mem := (*C.px4_msgs__msg__SensorGyroFifo)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	mem.device_id = C.uint32_t(m.DeviceId)
	mem.dt = C.float(m.Dt)
	mem.scale = C.float(m.Scale)
	mem.samples = C.uint8_t(m.Samples)
	cSlice_x := mem.x[:]
	primitives.Int16__Array_to_C(*(*[]primitives.CInt16)(unsafe.Pointer(&cSlice_x)), m.X[:])
	cSlice_y := mem.y[:]
	primitives.Int16__Array_to_C(*(*[]primitives.CInt16)(unsafe.Pointer(&cSlice_y)), m.Y[:])
	cSlice_z := mem.z[:]
	primitives.Int16__Array_to_C(*(*[]primitives.CInt16)(unsafe.Pointer(&cSlice_z)), m.Z[:])
}

func (t _SensorGyroFifoTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*SensorGyroFifo)
	mem := (*C.px4_msgs__msg__SensorGyroFifo)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	m.DeviceId = uint32(mem.device_id)
	m.Dt = float32(mem.dt)
	m.Scale = float32(mem.scale)
	m.Samples = uint8(mem.samples)
	cSlice_x := mem.x[:]
	primitives.Int16__Array_to_Go(m.X[:], *(*[]primitives.CInt16)(unsafe.Pointer(&cSlice_x)))
	cSlice_y := mem.y[:]
	primitives.Int16__Array_to_Go(m.Y[:], *(*[]primitives.CInt16)(unsafe.Pointer(&cSlice_y)))
	cSlice_z := mem.z[:]
	primitives.Int16__Array_to_Go(m.Z[:], *(*[]primitives.CInt16)(unsafe.Pointer(&cSlice_z)))
}

func (t _SensorGyroFifoTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__SensorGyroFifo())
}

type CSensorGyroFifo = C.px4_msgs__msg__SensorGyroFifo
type CSensorGyroFifo__Sequence = C.px4_msgs__msg__SensorGyroFifo__Sequence

func SensorGyroFifo__Sequence_to_Go(goSlice *[]SensorGyroFifo, cSlice CSensorGyroFifo__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]SensorGyroFifo, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		SensorGyroFifoTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func SensorGyroFifo__Sequence_to_C(cSlice *CSensorGyroFifo__Sequence, goSlice []SensorGyroFifo) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__SensorGyroFifo)(C.malloc(C.sizeof_struct_px4_msgs__msg__SensorGyroFifo * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		SensorGyroFifoTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func SensorGyroFifo__Array_to_Go(goSlice []SensorGyroFifo, cSlice []CSensorGyroFifo) {
	for i := 0; i < len(cSlice); i++ {
		SensorGyroFifoTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func SensorGyroFifo__Array_to_C(cSlice []CSensorGyroFifo, goSlice []SensorGyroFifo) {
	for i := 0; i < len(goSlice); i++ {
		SensorGyroFifoTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
