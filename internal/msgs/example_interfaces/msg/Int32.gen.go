/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package example_interfaces_msg

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <example_interfaces/msg/int32.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("example_interfaces/Int32", Int32TypeSupport)
	typemap.RegisterMessage("example_interfaces/msg/Int32", Int32TypeSupport)
}

type Int32 struct {
	Data int32 `yaml:"data"` // This is an example message of using a primitive datatype, int32.If you want to test with this that's fine, but if you are deployingit into a system you should create a semantically meaningful message type.If you want to embed it in another message, use the primitive data type instead.
}

// NewInt32 creates a new Int32 with default values.
func NewInt32() *Int32 {
	self := Int32{}
	self.SetDefaults()
	return &self
}

func (t *Int32) Clone() *Int32 {
	c := &Int32{}
	c.Data = t.Data
	return c
}

func (t *Int32) CloneMsg() types.Message {
	return t.Clone()
}

func (t *Int32) SetDefaults() {
	t.Data = 0
}

func (t *Int32) GetTypeSupport() types.MessageTypeSupport {
	return Int32TypeSupport
}

// Int32Publisher wraps rclgo.Publisher to provide type safe helper
// functions
type Int32Publisher struct {
	*rclgo.Publisher
}

// NewInt32Publisher creates and returns a new publisher for the
// Int32
func NewInt32Publisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*Int32Publisher, error) {
	pub, err := node.NewPublisher(topic_name, Int32TypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &Int32Publisher{pub}, nil
}

func (p *Int32Publisher) Publish(msg *Int32) error {
	return p.Publisher.Publish(msg)
}

// Int32Subscription wraps rclgo.Subscription to provide type safe helper
// functions
type Int32Subscription struct {
	*rclgo.Subscription
}

// Int32SubscriptionCallback type is used to provide a subscription
// handler function for a Int32Subscription.
type Int32SubscriptionCallback func(msg *Int32, info *rclgo.MessageInfo, err error)

// NewInt32Subscription creates and returns a new subscription for the
// Int32
func NewInt32Subscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback Int32SubscriptionCallback) (*Int32Subscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg Int32
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, Int32TypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &Int32Subscription{sub}, nil
}

func (s *Int32Subscription) TakeMessage(out *Int32) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneInt32Slice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneInt32Slice(dst, src []Int32) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var Int32TypeSupport types.MessageTypeSupport = _Int32TypeSupport{}

type _Int32TypeSupport struct{}

func (t _Int32TypeSupport) New() types.Message {
	return NewInt32()
}

func (t _Int32TypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.example_interfaces__msg__Int32
	return (unsafe.Pointer)(C.example_interfaces__msg__Int32__create())
}

func (t _Int32TypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.example_interfaces__msg__Int32__destroy((*C.example_interfaces__msg__Int32)(pointer_to_free))
}

func (t _Int32TypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*Int32)
	mem := (*C.example_interfaces__msg__Int32)(dst)
	mem.data = C.int32_t(m.Data)
}

func (t _Int32TypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*Int32)
	mem := (*C.example_interfaces__msg__Int32)(ros2_message_buffer)
	m.Data = int32(mem.data)
}

func (t _Int32TypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__msg__Int32())
}

type CInt32 = C.example_interfaces__msg__Int32
type CInt32__Sequence = C.example_interfaces__msg__Int32__Sequence

func Int32__Sequence_to_Go(goSlice *[]Int32, cSlice CInt32__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]Int32, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		Int32TypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func Int32__Sequence_to_C(cSlice *CInt32__Sequence, goSlice []Int32) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.example_interfaces__msg__Int32)(C.malloc(C.sizeof_struct_example_interfaces__msg__Int32 * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		Int32TypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func Int32__Array_to_Go(goSlice []Int32, cSlice []CInt32) {
	for i := 0; i < len(cSlice); i++ {
		Int32TypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func Int32__Array_to_C(cSlice []CInt32, goSlice []Int32) {
	for i := 0; i < len(goSlice); i++ {
		Int32TypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
