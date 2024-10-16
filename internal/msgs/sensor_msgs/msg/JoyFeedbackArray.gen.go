/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package sensor_msgs_msg

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <sensor_msgs/msg/joy_feedback_array.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("sensor_msgs/JoyFeedbackArray", JoyFeedbackArrayTypeSupport)
	typemap.RegisterMessage("sensor_msgs/msg/JoyFeedbackArray", JoyFeedbackArrayTypeSupport)
}

type JoyFeedbackArray struct {
	Array []JoyFeedback `yaml:"array"` // This message publishes values for multiple feedback at once.
}

// NewJoyFeedbackArray creates a new JoyFeedbackArray with default values.
func NewJoyFeedbackArray() *JoyFeedbackArray {
	self := JoyFeedbackArray{}
	self.SetDefaults()
	return &self
}

func (t *JoyFeedbackArray) Clone() *JoyFeedbackArray {
	c := &JoyFeedbackArray{}
	if t.Array != nil {
		c.Array = make([]JoyFeedback, len(t.Array))
		CloneJoyFeedbackSlice(c.Array, t.Array)
	}
	return c
}

func (t *JoyFeedbackArray) CloneMsg() types.Message {
	return t.Clone()
}

func (t *JoyFeedbackArray) SetDefaults() {
	t.Array = nil
}

func (t *JoyFeedbackArray) GetTypeSupport() types.MessageTypeSupport {
	return JoyFeedbackArrayTypeSupport
}

// JoyFeedbackArrayPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type JoyFeedbackArrayPublisher struct {
	*rclgo.Publisher
}

// NewJoyFeedbackArrayPublisher creates and returns a new publisher for the
// JoyFeedbackArray
func NewJoyFeedbackArrayPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*JoyFeedbackArrayPublisher, error) {
	pub, err := node.NewPublisher(topic_name, JoyFeedbackArrayTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &JoyFeedbackArrayPublisher{pub}, nil
}

func (p *JoyFeedbackArrayPublisher) Publish(msg *JoyFeedbackArray) error {
	return p.Publisher.Publish(msg)
}

// JoyFeedbackArraySubscription wraps rclgo.Subscription to provide type safe helper
// functions
type JoyFeedbackArraySubscription struct {
	*rclgo.Subscription
}

// JoyFeedbackArraySubscriptionCallback type is used to provide a subscription
// handler function for a JoyFeedbackArraySubscription.
type JoyFeedbackArraySubscriptionCallback func(msg *JoyFeedbackArray, info *rclgo.MessageInfo, err error)

// NewJoyFeedbackArraySubscription creates and returns a new subscription for the
// JoyFeedbackArray
func NewJoyFeedbackArraySubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback JoyFeedbackArraySubscriptionCallback) (*JoyFeedbackArraySubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg JoyFeedbackArray
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, JoyFeedbackArrayTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &JoyFeedbackArraySubscription{sub}, nil
}

func (s *JoyFeedbackArraySubscription) TakeMessage(out *JoyFeedbackArray) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneJoyFeedbackArraySlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneJoyFeedbackArraySlice(dst, src []JoyFeedbackArray) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var JoyFeedbackArrayTypeSupport types.MessageTypeSupport = _JoyFeedbackArrayTypeSupport{}

type _JoyFeedbackArrayTypeSupport struct{}

func (t _JoyFeedbackArrayTypeSupport) New() types.Message {
	return NewJoyFeedbackArray()
}

func (t _JoyFeedbackArrayTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.sensor_msgs__msg__JoyFeedbackArray
	return (unsafe.Pointer)(C.sensor_msgs__msg__JoyFeedbackArray__create())
}

func (t _JoyFeedbackArrayTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.sensor_msgs__msg__JoyFeedbackArray__destroy((*C.sensor_msgs__msg__JoyFeedbackArray)(pointer_to_free))
}

func (t _JoyFeedbackArrayTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*JoyFeedbackArray)
	mem := (*C.sensor_msgs__msg__JoyFeedbackArray)(dst)
	JoyFeedback__Sequence_to_C(&mem.array, m.Array)
}

func (t _JoyFeedbackArrayTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*JoyFeedbackArray)
	mem := (*C.sensor_msgs__msg__JoyFeedbackArray)(ros2_message_buffer)
	JoyFeedback__Sequence_to_Go(&m.Array, mem.array)
}

func (t _JoyFeedbackArrayTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__JoyFeedbackArray())
}

type CJoyFeedbackArray = C.sensor_msgs__msg__JoyFeedbackArray
type CJoyFeedbackArray__Sequence = C.sensor_msgs__msg__JoyFeedbackArray__Sequence

func JoyFeedbackArray__Sequence_to_Go(goSlice *[]JoyFeedbackArray, cSlice CJoyFeedbackArray__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]JoyFeedbackArray, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		JoyFeedbackArrayTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func JoyFeedbackArray__Sequence_to_C(cSlice *CJoyFeedbackArray__Sequence, goSlice []JoyFeedbackArray) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.sensor_msgs__msg__JoyFeedbackArray)(C.malloc(C.sizeof_struct_sensor_msgs__msg__JoyFeedbackArray * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		JoyFeedbackArrayTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func JoyFeedbackArray__Array_to_Go(goSlice []JoyFeedbackArray, cSlice []CJoyFeedbackArray) {
	for i := 0; i < len(cSlice); i++ {
		JoyFeedbackArrayTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func JoyFeedbackArray__Array_to_C(cSlice []CJoyFeedbackArray, goSlice []JoyFeedbackArray) {
	for i := 0; i < len(goSlice); i++ {
		JoyFeedbackArrayTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
