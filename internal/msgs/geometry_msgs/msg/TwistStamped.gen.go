/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package geometry_msgs_msg
import (
	"unsafe"

	"github.com/PolibaX/rclgo/pkg/rclgo"
	"github.com/PolibaX/rclgo/pkg/rclgo/types"
	"github.com/PolibaX/rclgo/pkg/rclgo/typemap"
	std_msgs_msg "github.com/PolibaX/rclgo/internal/msgs/std_msgs/msg"
	
)
/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <geometry_msgs/msg/twist_stamped.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("geometry_msgs/TwistStamped", TwistStampedTypeSupport)
	typemap.RegisterMessage("geometry_msgs/msg/TwistStamped", TwistStampedTypeSupport)
}

type TwistStamped struct {
	Header std_msgs_msg.Header `yaml:"header"`
	Twist Twist `yaml:"twist"`
}

// NewTwistStamped creates a new TwistStamped with default values.
func NewTwistStamped() *TwistStamped {
	self := TwistStamped{}
	self.SetDefaults()
	return &self
}

func (t *TwistStamped) Clone() *TwistStamped {
	c := &TwistStamped{}
	c.Header = *t.Header.Clone()
	c.Twist = *t.Twist.Clone()
	return c
}

func (t *TwistStamped) CloneMsg() types.Message {
	return t.Clone()
}

func (t *TwistStamped) SetDefaults() {
	t.Header.SetDefaults()
	t.Twist.SetDefaults()
}

func (t *TwistStamped) GetTypeSupport() types.MessageTypeSupport {
	return TwistStampedTypeSupport
}

// TwistStampedPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type TwistStampedPublisher struct {
	*rclgo.Publisher
}

// NewTwistStampedPublisher creates and returns a new publisher for the
// TwistStamped
func NewTwistStampedPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*TwistStampedPublisher, error) {
	pub, err := node.NewPublisher(topic_name, TwistStampedTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &TwistStampedPublisher{pub}, nil
}

func (p *TwistStampedPublisher) Publish(msg *TwistStamped) error {
	return p.Publisher.Publish(msg)
}

// TwistStampedSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type TwistStampedSubscription struct {
	*rclgo.Subscription
}

// TwistStampedSubscriptionCallback type is used to provide a subscription
// handler function for a TwistStampedSubscription.
type TwistStampedSubscriptionCallback func(msg *TwistStamped, info *rclgo.MessageInfo, err error)

// NewTwistStampedSubscription creates and returns a new subscription for the
// TwistStamped
func NewTwistStampedSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback TwistStampedSubscriptionCallback) (*TwistStampedSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg TwistStamped
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, TwistStampedTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &TwistStampedSubscription{sub}, nil
}

func (s *TwistStampedSubscription) TakeMessage(out *TwistStamped) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneTwistStampedSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneTwistStampedSlice(dst, src []TwistStamped) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var TwistStampedTypeSupport types.MessageTypeSupport = _TwistStampedTypeSupport{}

type _TwistStampedTypeSupport struct{}

func (t _TwistStampedTypeSupport) New() types.Message {
	return NewTwistStamped()
}

func (t _TwistStampedTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.geometry_msgs__msg__TwistStamped
	return (unsafe.Pointer)(C.geometry_msgs__msg__TwistStamped__create())
}

func (t _TwistStampedTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.geometry_msgs__msg__TwistStamped__destroy((*C.geometry_msgs__msg__TwistStamped)(pointer_to_free))
}

func (t _TwistStampedTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*TwistStamped)
	mem := (*C.geometry_msgs__msg__TwistStamped)(dst)
	std_msgs_msg.HeaderTypeSupport.AsCStruct(unsafe.Pointer(&mem.header), &m.Header)
	TwistTypeSupport.AsCStruct(unsafe.Pointer(&mem.twist), &m.Twist)
}

func (t _TwistStampedTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*TwistStamped)
	mem := (*C.geometry_msgs__msg__TwistStamped)(ros2_message_buffer)
	std_msgs_msg.HeaderTypeSupport.AsGoStruct(&m.Header, unsafe.Pointer(&mem.header))
	TwistTypeSupport.AsGoStruct(&m.Twist, unsafe.Pointer(&mem.twist))
}

func (t _TwistStampedTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__TwistStamped())
}

type CTwistStamped = C.geometry_msgs__msg__TwistStamped
type CTwistStamped__Sequence = C.geometry_msgs__msg__TwistStamped__Sequence

func TwistStamped__Sequence_to_Go(goSlice *[]TwistStamped, cSlice CTwistStamped__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]TwistStamped, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		TwistStampedTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func TwistStamped__Sequence_to_C(cSlice *CTwistStamped__Sequence, goSlice []TwistStamped) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.geometry_msgs__msg__TwistStamped)(C.malloc(C.sizeof_struct_geometry_msgs__msg__TwistStamped * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		TwistStampedTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func TwistStamped__Array_to_Go(goSlice []TwistStamped, cSlice []CTwistStamped) {
	for i := 0; i < len(cSlice); i++ {
		TwistStampedTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func TwistStamped__Array_to_C(cSlice []CTwistStamped, goSlice []TwistStamped) {
	for i := 0; i < len(goSlice); i++ {
		TwistStampedTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
