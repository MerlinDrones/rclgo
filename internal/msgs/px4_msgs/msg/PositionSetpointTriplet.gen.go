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

#include <px4_msgs/msg/position_setpoint_triplet.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/PositionSetpointTriplet", PositionSetpointTripletTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/PositionSetpointTriplet", PositionSetpointTripletTypeSupport)
}

type PositionSetpointTriplet struct {
	Timestamp uint64           `yaml:"timestamp"` // time since system start (microseconds)
	Previous  PositionSetpoint `yaml:"previous"`
	Current   PositionSetpoint `yaml:"current"`
	Next      PositionSetpoint `yaml:"next"`
}

// NewPositionSetpointTriplet creates a new PositionSetpointTriplet with default values.
func NewPositionSetpointTriplet() *PositionSetpointTriplet {
	self := PositionSetpointTriplet{}
	self.SetDefaults()
	return &self
}

func (t *PositionSetpointTriplet) Clone() *PositionSetpointTriplet {
	c := &PositionSetpointTriplet{}
	c.Timestamp = t.Timestamp
	c.Previous = *t.Previous.Clone()
	c.Current = *t.Current.Clone()
	c.Next = *t.Next.Clone()
	return c
}

func (t *PositionSetpointTriplet) CloneMsg() types.Message {
	return t.Clone()
}

func (t *PositionSetpointTriplet) SetDefaults() {
	t.Timestamp = 0
	t.Previous.SetDefaults()
	t.Current.SetDefaults()
	t.Next.SetDefaults()
}

func (t *PositionSetpointTriplet) GetTypeSupport() types.MessageTypeSupport {
	return PositionSetpointTripletTypeSupport
}

// PositionSetpointTripletPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type PositionSetpointTripletPublisher struct {
	*rclgo.Publisher
}

// NewPositionSetpointTripletPublisher creates and returns a new publisher for the
// PositionSetpointTriplet
func NewPositionSetpointTripletPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*PositionSetpointTripletPublisher, error) {
	pub, err := node.NewPublisher(topic_name, PositionSetpointTripletTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &PositionSetpointTripletPublisher{pub}, nil
}

func (p *PositionSetpointTripletPublisher) Publish(msg *PositionSetpointTriplet) error {
	return p.Publisher.Publish(msg)
}

// PositionSetpointTripletSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type PositionSetpointTripletSubscription struct {
	*rclgo.Subscription
}

// PositionSetpointTripletSubscriptionCallback type is used to provide a subscription
// handler function for a PositionSetpointTripletSubscription.
type PositionSetpointTripletSubscriptionCallback func(msg *PositionSetpointTriplet, info *rclgo.MessageInfo, err error)

// NewPositionSetpointTripletSubscription creates and returns a new subscription for the
// PositionSetpointTriplet
func NewPositionSetpointTripletSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback PositionSetpointTripletSubscriptionCallback) (*PositionSetpointTripletSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg PositionSetpointTriplet
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, PositionSetpointTripletTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &PositionSetpointTripletSubscription{sub}, nil
}

func (s *PositionSetpointTripletSubscription) TakeMessage(out *PositionSetpointTriplet) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// ClonePositionSetpointTripletSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func ClonePositionSetpointTripletSlice(dst, src []PositionSetpointTriplet) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var PositionSetpointTripletTypeSupport types.MessageTypeSupport = _PositionSetpointTripletTypeSupport{}

type _PositionSetpointTripletTypeSupport struct{}

func (t _PositionSetpointTripletTypeSupport) New() types.Message {
	return NewPositionSetpointTriplet()
}

func (t _PositionSetpointTripletTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__PositionSetpointTriplet
	return (unsafe.Pointer)(C.px4_msgs__msg__PositionSetpointTriplet__create())
}

func (t _PositionSetpointTripletTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__PositionSetpointTriplet__destroy((*C.px4_msgs__msg__PositionSetpointTriplet)(pointer_to_free))
}

func (t _PositionSetpointTripletTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*PositionSetpointTriplet)
	mem := (*C.px4_msgs__msg__PositionSetpointTriplet)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	PositionSetpointTypeSupport.AsCStruct(unsafe.Pointer(&mem.previous), &m.Previous)
	PositionSetpointTypeSupport.AsCStruct(unsafe.Pointer(&mem.current), &m.Current)
	PositionSetpointTypeSupport.AsCStruct(unsafe.Pointer(&mem.next), &m.Next)
}

func (t _PositionSetpointTripletTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*PositionSetpointTriplet)
	mem := (*C.px4_msgs__msg__PositionSetpointTriplet)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	PositionSetpointTypeSupport.AsGoStruct(&m.Previous, unsafe.Pointer(&mem.previous))
	PositionSetpointTypeSupport.AsGoStruct(&m.Current, unsafe.Pointer(&mem.current))
	PositionSetpointTypeSupport.AsGoStruct(&m.Next, unsafe.Pointer(&mem.next))
}

func (t _PositionSetpointTripletTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__PositionSetpointTriplet())
}

type CPositionSetpointTriplet = C.px4_msgs__msg__PositionSetpointTriplet
type CPositionSetpointTriplet__Sequence = C.px4_msgs__msg__PositionSetpointTriplet__Sequence

func PositionSetpointTriplet__Sequence_to_Go(goSlice *[]PositionSetpointTriplet, cSlice CPositionSetpointTriplet__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]PositionSetpointTriplet, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		PositionSetpointTripletTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func PositionSetpointTriplet__Sequence_to_C(cSlice *CPositionSetpointTriplet__Sequence, goSlice []PositionSetpointTriplet) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__PositionSetpointTriplet)(C.malloc(C.sizeof_struct_px4_msgs__msg__PositionSetpointTriplet * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		PositionSetpointTripletTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func PositionSetpointTriplet__Array_to_Go(goSlice []PositionSetpointTriplet, cSlice []CPositionSetpointTriplet) {
	for i := 0; i < len(cSlice); i++ {
		PositionSetpointTripletTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func PositionSetpointTriplet__Array_to_C(cSlice []CPositionSetpointTriplet, goSlice []PositionSetpointTriplet) {
	for i := 0; i < len(goSlice); i++ {
		PositionSetpointTripletTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
