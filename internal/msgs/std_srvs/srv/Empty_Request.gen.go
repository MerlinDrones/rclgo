/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package std_srvs_srv

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <std_srvs/srv/empty.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("std_srvs/Empty_Request", Empty_RequestTypeSupport)
	typemap.RegisterMessage("std_srvs/srv/Empty_Request", Empty_RequestTypeSupport)
}

type Empty_Request struct {
}

// NewEmpty_Request creates a new Empty_Request with default values.
func NewEmpty_Request() *Empty_Request {
	self := Empty_Request{}
	self.SetDefaults()
	return &self
}

func (t *Empty_Request) Clone() *Empty_Request {
	c := &Empty_Request{}
	return c
}

func (t *Empty_Request) CloneMsg() types.Message {
	return t.Clone()
}

func (t *Empty_Request) SetDefaults() {
}

func (t *Empty_Request) GetTypeSupport() types.MessageTypeSupport {
	return Empty_RequestTypeSupport
}

// Empty_RequestPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type Empty_RequestPublisher struct {
	*rclgo.Publisher
}

// NewEmpty_RequestPublisher creates and returns a new publisher for the
// Empty_Request
func NewEmpty_RequestPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*Empty_RequestPublisher, error) {
	pub, err := node.NewPublisher(topic_name, Empty_RequestTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &Empty_RequestPublisher{pub}, nil
}

func (p *Empty_RequestPublisher) Publish(msg *Empty_Request) error {
	return p.Publisher.Publish(msg)
}

// Empty_RequestSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type Empty_RequestSubscription struct {
	*rclgo.Subscription
}

// Empty_RequestSubscriptionCallback type is used to provide a subscription
// handler function for a Empty_RequestSubscription.
type Empty_RequestSubscriptionCallback func(msg *Empty_Request, info *rclgo.MessageInfo, err error)

// NewEmpty_RequestSubscription creates and returns a new subscription for the
// Empty_Request
func NewEmpty_RequestSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback Empty_RequestSubscriptionCallback) (*Empty_RequestSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg Empty_Request
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, Empty_RequestTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &Empty_RequestSubscription{sub}, nil
}

func (s *Empty_RequestSubscription) TakeMessage(out *Empty_Request) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneEmpty_RequestSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneEmpty_RequestSlice(dst, src []Empty_Request) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var Empty_RequestTypeSupport types.MessageTypeSupport = _Empty_RequestTypeSupport{}

type _Empty_RequestTypeSupport struct{}

func (t _Empty_RequestTypeSupport) New() types.Message {
	return NewEmpty_Request()
}

func (t _Empty_RequestTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.std_srvs__srv__Empty_Request
	return (unsafe.Pointer)(C.std_srvs__srv__Empty_Request__create())
}

func (t _Empty_RequestTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.std_srvs__srv__Empty_Request__destroy((*C.std_srvs__srv__Empty_Request)(pointer_to_free))
}

func (t _Empty_RequestTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {

}

func (t _Empty_RequestTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {

}

func (t _Empty_RequestTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__Empty_Request())
}

type CEmpty_Request = C.std_srvs__srv__Empty_Request
type CEmpty_Request__Sequence = C.std_srvs__srv__Empty_Request__Sequence

func Empty_Request__Sequence_to_Go(goSlice *[]Empty_Request, cSlice CEmpty_Request__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]Empty_Request, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		Empty_RequestTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func Empty_Request__Sequence_to_C(cSlice *CEmpty_Request__Sequence, goSlice []Empty_Request) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.std_srvs__srv__Empty_Request)(C.malloc(C.sizeof_struct_std_srvs__srv__Empty_Request * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		Empty_RequestTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func Empty_Request__Array_to_Go(goSlice []Empty_Request, cSlice []CEmpty_Request) {
	for i := 0; i < len(cSlice); i++ {
		Empty_RequestTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func Empty_Request__Array_to_C(cSlice []CEmpty_Request, goSlice []Empty_Request) {
	for i := 0; i < len(goSlice); i++ {
		Empty_RequestTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
