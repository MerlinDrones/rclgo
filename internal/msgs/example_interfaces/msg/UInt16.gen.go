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

	"github.com/PolibaX/rclgo/pkg/rclgo"
	"github.com/PolibaX/rclgo/pkg/rclgo/types"
	"github.com/PolibaX/rclgo/pkg/rclgo/typemap"
	
)
/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <example_interfaces/msg/u_int16.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("example_interfaces/UInt16", UInt16TypeSupport)
	typemap.RegisterMessage("example_interfaces/msg/UInt16", UInt16TypeSupport)
}

type UInt16 struct {
	Data uint16 `yaml:"data"`// This is an example message of using a primitive datatype, uint16.If you want to test with this that's fine, but if you are deployingit into a system you should create a semantically meaningful message type.If you want to embed it in another message, use the primitive data type instead.
}

// NewUInt16 creates a new UInt16 with default values.
func NewUInt16() *UInt16 {
	self := UInt16{}
	self.SetDefaults()
	return &self
}

func (t *UInt16) Clone() *UInt16 {
	c := &UInt16{}
	c.Data = t.Data
	return c
}

func (t *UInt16) CloneMsg() types.Message {
	return t.Clone()
}

func (t *UInt16) SetDefaults() {
	t.Data = 0
}

func (t *UInt16) GetTypeSupport() types.MessageTypeSupport {
	return UInt16TypeSupport
}

// UInt16Publisher wraps rclgo.Publisher to provide type safe helper
// functions
type UInt16Publisher struct {
	*rclgo.Publisher
}

// NewUInt16Publisher creates and returns a new publisher for the
// UInt16
func NewUInt16Publisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*UInt16Publisher, error) {
	pub, err := node.NewPublisher(topic_name, UInt16TypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &UInt16Publisher{pub}, nil
}

func (p *UInt16Publisher) Publish(msg *UInt16) error {
	return p.Publisher.Publish(msg)
}

// UInt16Subscription wraps rclgo.Subscription to provide type safe helper
// functions
type UInt16Subscription struct {
	*rclgo.Subscription
}

// UInt16SubscriptionCallback type is used to provide a subscription
// handler function for a UInt16Subscription.
type UInt16SubscriptionCallback func(msg *UInt16, info *rclgo.MessageInfo, err error)

// NewUInt16Subscription creates and returns a new subscription for the
// UInt16
func NewUInt16Subscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback UInt16SubscriptionCallback) (*UInt16Subscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg UInt16
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, UInt16TypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &UInt16Subscription{sub}, nil
}

func (s *UInt16Subscription) TakeMessage(out *UInt16) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneUInt16Slice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneUInt16Slice(dst, src []UInt16) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var UInt16TypeSupport types.MessageTypeSupport = _UInt16TypeSupport{}

type _UInt16TypeSupport struct{}

func (t _UInt16TypeSupport) New() types.Message {
	return NewUInt16()
}

func (t _UInt16TypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.example_interfaces__msg__UInt16
	return (unsafe.Pointer)(C.example_interfaces__msg__UInt16__create())
}

func (t _UInt16TypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.example_interfaces__msg__UInt16__destroy((*C.example_interfaces__msg__UInt16)(pointer_to_free))
}

func (t _UInt16TypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*UInt16)
	mem := (*C.example_interfaces__msg__UInt16)(dst)
	mem.data = C.uint16_t(m.Data)
}

func (t _UInt16TypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*UInt16)
	mem := (*C.example_interfaces__msg__UInt16)(ros2_message_buffer)
	m.Data = uint16(mem.data)
}

func (t _UInt16TypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__msg__UInt16())
}

type CUInt16 = C.example_interfaces__msg__UInt16
type CUInt16__Sequence = C.example_interfaces__msg__UInt16__Sequence

func UInt16__Sequence_to_Go(goSlice *[]UInt16, cSlice CUInt16__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]UInt16, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		UInt16TypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func UInt16__Sequence_to_C(cSlice *CUInt16__Sequence, goSlice []UInt16) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.example_interfaces__msg__UInt16)(C.malloc(C.sizeof_struct_example_interfaces__msg__UInt16 * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		UInt16TypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func UInt16__Array_to_Go(goSlice []UInt16, cSlice []CUInt16) {
	for i := 0; i < len(cSlice); i++ {
		UInt16TypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func UInt16__Array_to_C(cSlice []CUInt16, goSlice []UInt16) {
	for i := 0; i < len(goSlice); i++ {
		UInt16TypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
