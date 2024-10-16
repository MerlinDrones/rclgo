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

#include <px4_msgs/msg/actuator_servos.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/ActuatorServos", ActuatorServosTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/ActuatorServos", ActuatorServosTypeSupport)
}

const (
	ActuatorServos_NUM_CONTROLS uint8 = 8
)

type ActuatorServos struct {
	Timestamp       uint64     `yaml:"timestamp"`        // time since system start (microseconds). Servo control message
	TimestampSample uint64     `yaml:"timestamp_sample"` // the timestamp the data this control response is based on was sampled
	Control         [8]float32 `yaml:"control"`          // range: [-1, 1], where 1 means maximum positive position,
}

// NewActuatorServos creates a new ActuatorServos with default values.
func NewActuatorServos() *ActuatorServos {
	self := ActuatorServos{}
	self.SetDefaults()
	return &self
}

func (t *ActuatorServos) Clone() *ActuatorServos {
	c := &ActuatorServos{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.Control = t.Control
	return c
}

func (t *ActuatorServos) CloneMsg() types.Message {
	return t.Clone()
}

func (t *ActuatorServos) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.Control = [8]float32{}
}

func (t *ActuatorServos) GetTypeSupport() types.MessageTypeSupport {
	return ActuatorServosTypeSupport
}

// ActuatorServosPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type ActuatorServosPublisher struct {
	*rclgo.Publisher
}

// NewActuatorServosPublisher creates and returns a new publisher for the
// ActuatorServos
func NewActuatorServosPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*ActuatorServosPublisher, error) {
	pub, err := node.NewPublisher(topic_name, ActuatorServosTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &ActuatorServosPublisher{pub}, nil
}

func (p *ActuatorServosPublisher) Publish(msg *ActuatorServos) error {
	return p.Publisher.Publish(msg)
}

// ActuatorServosSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type ActuatorServosSubscription struct {
	*rclgo.Subscription
}

// ActuatorServosSubscriptionCallback type is used to provide a subscription
// handler function for a ActuatorServosSubscription.
type ActuatorServosSubscriptionCallback func(msg *ActuatorServos, info *rclgo.MessageInfo, err error)

// NewActuatorServosSubscription creates and returns a new subscription for the
// ActuatorServos
func NewActuatorServosSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback ActuatorServosSubscriptionCallback) (*ActuatorServosSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg ActuatorServos
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, ActuatorServosTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &ActuatorServosSubscription{sub}, nil
}

func (s *ActuatorServosSubscription) TakeMessage(out *ActuatorServos) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneActuatorServosSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneActuatorServosSlice(dst, src []ActuatorServos) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var ActuatorServosTypeSupport types.MessageTypeSupport = _ActuatorServosTypeSupport{}

type _ActuatorServosTypeSupport struct{}

func (t _ActuatorServosTypeSupport) New() types.Message {
	return NewActuatorServos()
}

func (t _ActuatorServosTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__ActuatorServos
	return (unsafe.Pointer)(C.px4_msgs__msg__ActuatorServos__create())
}

func (t _ActuatorServosTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__ActuatorServos__destroy((*C.px4_msgs__msg__ActuatorServos)(pointer_to_free))
}

func (t _ActuatorServosTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*ActuatorServos)
	mem := (*C.px4_msgs__msg__ActuatorServos)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	cSlice_control := mem.control[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_control)), m.Control[:])
}

func (t _ActuatorServosTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*ActuatorServos)
	mem := (*C.px4_msgs__msg__ActuatorServos)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	cSlice_control := mem.control[:]
	primitives.Float32__Array_to_Go(m.Control[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_control)))
}

func (t _ActuatorServosTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__ActuatorServos())
}

type CActuatorServos = C.px4_msgs__msg__ActuatorServos
type CActuatorServos__Sequence = C.px4_msgs__msg__ActuatorServos__Sequence

func ActuatorServos__Sequence_to_Go(goSlice *[]ActuatorServos, cSlice CActuatorServos__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]ActuatorServos, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		ActuatorServosTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func ActuatorServos__Sequence_to_C(cSlice *CActuatorServos__Sequence, goSlice []ActuatorServos) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__ActuatorServos)(C.malloc(C.sizeof_struct_px4_msgs__msg__ActuatorServos * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		ActuatorServosTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func ActuatorServos__Array_to_Go(goSlice []ActuatorServos, cSlice []CActuatorServos) {
	for i := 0; i < len(cSlice); i++ {
		ActuatorServosTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func ActuatorServos__Array_to_C(cSlice []CActuatorServos, goSlice []ActuatorServos) {
	for i := 0; i < len(goSlice); i++ {
		ActuatorServosTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
