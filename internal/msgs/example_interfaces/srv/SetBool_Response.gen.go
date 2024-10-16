/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package example_interfaces_srv

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	primitives "github.com/merlindrones/rclgo/pkg/rclgo/primitives"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <example_interfaces/srv/set_bool.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("example_interfaces/SetBool_Response", SetBool_ResponseTypeSupport)
	typemap.RegisterMessage("example_interfaces/srv/SetBool_Response", SetBool_ResponseTypeSupport)
}

type SetBool_Response struct {
	Success bool   `yaml:"success"` // indicate successful run of triggered service
	Message string `yaml:"message"` // informational, e.g. for error messages
}

// NewSetBool_Response creates a new SetBool_Response with default values.
func NewSetBool_Response() *SetBool_Response {
	self := SetBool_Response{}
	self.SetDefaults()
	return &self
}

func (t *SetBool_Response) Clone() *SetBool_Response {
	c := &SetBool_Response{}
	c.Success = t.Success
	c.Message = t.Message
	return c
}

func (t *SetBool_Response) CloneMsg() types.Message {
	return t.Clone()
}

func (t *SetBool_Response) SetDefaults() {
	t.Success = false
	t.Message = ""
}

func (t *SetBool_Response) GetTypeSupport() types.MessageTypeSupport {
	return SetBool_ResponseTypeSupport
}

// SetBool_ResponsePublisher wraps rclgo.Publisher to provide type safe helper
// functions
type SetBool_ResponsePublisher struct {
	*rclgo.Publisher
}

// NewSetBool_ResponsePublisher creates and returns a new publisher for the
// SetBool_Response
func NewSetBool_ResponsePublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*SetBool_ResponsePublisher, error) {
	pub, err := node.NewPublisher(topic_name, SetBool_ResponseTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &SetBool_ResponsePublisher{pub}, nil
}

func (p *SetBool_ResponsePublisher) Publish(msg *SetBool_Response) error {
	return p.Publisher.Publish(msg)
}

// SetBool_ResponseSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type SetBool_ResponseSubscription struct {
	*rclgo.Subscription
}

// SetBool_ResponseSubscriptionCallback type is used to provide a subscription
// handler function for a SetBool_ResponseSubscription.
type SetBool_ResponseSubscriptionCallback func(msg *SetBool_Response, info *rclgo.MessageInfo, err error)

// NewSetBool_ResponseSubscription creates and returns a new subscription for the
// SetBool_Response
func NewSetBool_ResponseSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback SetBool_ResponseSubscriptionCallback) (*SetBool_ResponseSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg SetBool_Response
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, SetBool_ResponseTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &SetBool_ResponseSubscription{sub}, nil
}

func (s *SetBool_ResponseSubscription) TakeMessage(out *SetBool_Response) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneSetBool_ResponseSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneSetBool_ResponseSlice(dst, src []SetBool_Response) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var SetBool_ResponseTypeSupport types.MessageTypeSupport = _SetBool_ResponseTypeSupport{}

type _SetBool_ResponseTypeSupport struct{}

func (t _SetBool_ResponseTypeSupport) New() types.Message {
	return NewSetBool_Response()
}

func (t _SetBool_ResponseTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.example_interfaces__srv__SetBool_Response
	return (unsafe.Pointer)(C.example_interfaces__srv__SetBool_Response__create())
}

func (t _SetBool_ResponseTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.example_interfaces__srv__SetBool_Response__destroy((*C.example_interfaces__srv__SetBool_Response)(pointer_to_free))
}

func (t _SetBool_ResponseTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*SetBool_Response)
	mem := (*C.example_interfaces__srv__SetBool_Response)(dst)
	mem.success = C.bool(m.Success)
	primitives.StringAsCStruct(unsafe.Pointer(&mem.message), m.Message)
}

func (t _SetBool_ResponseTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*SetBool_Response)
	mem := (*C.example_interfaces__srv__SetBool_Response)(ros2_message_buffer)
	m.Success = bool(mem.success)
	primitives.StringAsGoStruct(&m.Message, unsafe.Pointer(&mem.message))
}

func (t _SetBool_ResponseTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__SetBool_Response())
}

type CSetBool_Response = C.example_interfaces__srv__SetBool_Response
type CSetBool_Response__Sequence = C.example_interfaces__srv__SetBool_Response__Sequence

func SetBool_Response__Sequence_to_Go(goSlice *[]SetBool_Response, cSlice CSetBool_Response__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]SetBool_Response, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		SetBool_ResponseTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func SetBool_Response__Sequence_to_C(cSlice *CSetBool_Response__Sequence, goSlice []SetBool_Response) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.example_interfaces__srv__SetBool_Response)(C.malloc(C.sizeof_struct_example_interfaces__srv__SetBool_Response * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		SetBool_ResponseTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func SetBool_Response__Array_to_Go(goSlice []SetBool_Response, cSlice []CSetBool_Response) {
	for i := 0; i < len(cSlice); i++ {
		SetBool_ResponseTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func SetBool_Response__Array_to_C(cSlice []CSetBool_Response, goSlice []SetBool_Response) {
	for i := 0; i < len(goSlice); i++ {
		SetBool_ResponseTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
