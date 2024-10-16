/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package std_msgs_msg

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <std_msgs/msg/u_int32.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("std_msgs/UInt32", UInt32TypeSupport)
	typemap.RegisterMessage("std_msgs/msg/UInt32", UInt32TypeSupport)
}

type UInt32 struct {
	Data uint32 `yaml:"data"`
}

// NewUInt32 creates a new UInt32 with default values.
func NewUInt32() *UInt32 {
	self := UInt32{}
	self.SetDefaults()
	return &self
}

func (t *UInt32) Clone() *UInt32 {
	c := &UInt32{}
	c.Data = t.Data
	return c
}

func (t *UInt32) CloneMsg() types.Message {
	return t.Clone()
}

func (t *UInt32) SetDefaults() {
	t.Data = 0
}

func (t *UInt32) GetTypeSupport() types.MessageTypeSupport {
	return UInt32TypeSupport
}

// UInt32Publisher wraps rclgo.Publisher to provide type safe helper
// functions
type UInt32Publisher struct {
	*rclgo.Publisher
}

// NewUInt32Publisher creates and returns a new publisher for the
// UInt32
func NewUInt32Publisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*UInt32Publisher, error) {
	pub, err := node.NewPublisher(topic_name, UInt32TypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &UInt32Publisher{pub}, nil
}

func (p *UInt32Publisher) Publish(msg *UInt32) error {
	return p.Publisher.Publish(msg)
}

// UInt32Subscription wraps rclgo.Subscription to provide type safe helper
// functions
type UInt32Subscription struct {
	*rclgo.Subscription
}

// UInt32SubscriptionCallback type is used to provide a subscription
// handler function for a UInt32Subscription.
type UInt32SubscriptionCallback func(msg *UInt32, info *rclgo.MessageInfo, err error)

// NewUInt32Subscription creates and returns a new subscription for the
// UInt32
func NewUInt32Subscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback UInt32SubscriptionCallback) (*UInt32Subscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg UInt32
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, UInt32TypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &UInt32Subscription{sub}, nil
}

func (s *UInt32Subscription) TakeMessage(out *UInt32) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneUInt32Slice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneUInt32Slice(dst, src []UInt32) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var UInt32TypeSupport types.MessageTypeSupport = _UInt32TypeSupport{}

type _UInt32TypeSupport struct{}

func (t _UInt32TypeSupport) New() types.Message {
	return NewUInt32()
}

func (t _UInt32TypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.std_msgs__msg__UInt32
	return (unsafe.Pointer)(C.std_msgs__msg__UInt32__create())
}

func (t _UInt32TypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.std_msgs__msg__UInt32__destroy((*C.std_msgs__msg__UInt32)(pointer_to_free))
}

func (t _UInt32TypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*UInt32)
	mem := (*C.std_msgs__msg__UInt32)(dst)
	mem.data = C.uint32_t(m.Data)
}

func (t _UInt32TypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*UInt32)
	mem := (*C.std_msgs__msg__UInt32)(ros2_message_buffer)
	m.Data = uint32(mem.data)
}

func (t _UInt32TypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt32())
}

type CUInt32 = C.std_msgs__msg__UInt32
type CUInt32__Sequence = C.std_msgs__msg__UInt32__Sequence

func UInt32__Sequence_to_Go(goSlice *[]UInt32, cSlice CUInt32__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]UInt32, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		UInt32TypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func UInt32__Sequence_to_C(cSlice *CUInt32__Sequence, goSlice []UInt32) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.std_msgs__msg__UInt32)(C.malloc(C.sizeof_struct_std_msgs__msg__UInt32 * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		UInt32TypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func UInt32__Array_to_Go(goSlice []UInt32, cSlice []CUInt32) {
	for i := 0; i < len(cSlice); i++ {
		UInt32TypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func UInt32__Array_to_C(cSlice []CUInt32, goSlice []UInt32) {
	for i := 0; i < len(goSlice); i++ {
		UInt32TypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
