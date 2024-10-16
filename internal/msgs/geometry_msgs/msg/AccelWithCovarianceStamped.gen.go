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

	std_msgs_msg "github.com/merlindrones/rclgo/internal/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <geometry_msgs/msg/accel_with_covariance_stamped.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("geometry_msgs/AccelWithCovarianceStamped", AccelWithCovarianceStampedTypeSupport)
	typemap.RegisterMessage("geometry_msgs/msg/AccelWithCovarianceStamped", AccelWithCovarianceStampedTypeSupport)
}

type AccelWithCovarianceStamped struct {
	Header std_msgs_msg.Header `yaml:"header"` // This represents an estimated accel with reference coordinate frame and timestamp.
	Accel  AccelWithCovariance `yaml:"accel"`
}

// NewAccelWithCovarianceStamped creates a new AccelWithCovarianceStamped with default values.
func NewAccelWithCovarianceStamped() *AccelWithCovarianceStamped {
	self := AccelWithCovarianceStamped{}
	self.SetDefaults()
	return &self
}

func (t *AccelWithCovarianceStamped) Clone() *AccelWithCovarianceStamped {
	c := &AccelWithCovarianceStamped{}
	c.Header = *t.Header.Clone()
	c.Accel = *t.Accel.Clone()
	return c
}

func (t *AccelWithCovarianceStamped) CloneMsg() types.Message {
	return t.Clone()
}

func (t *AccelWithCovarianceStamped) SetDefaults() {
	t.Header.SetDefaults()
	t.Accel.SetDefaults()
}

func (t *AccelWithCovarianceStamped) GetTypeSupport() types.MessageTypeSupport {
	return AccelWithCovarianceStampedTypeSupport
}

// AccelWithCovarianceStampedPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type AccelWithCovarianceStampedPublisher struct {
	*rclgo.Publisher
}

// NewAccelWithCovarianceStampedPublisher creates and returns a new publisher for the
// AccelWithCovarianceStamped
func NewAccelWithCovarianceStampedPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*AccelWithCovarianceStampedPublisher, error) {
	pub, err := node.NewPublisher(topic_name, AccelWithCovarianceStampedTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &AccelWithCovarianceStampedPublisher{pub}, nil
}

func (p *AccelWithCovarianceStampedPublisher) Publish(msg *AccelWithCovarianceStamped) error {
	return p.Publisher.Publish(msg)
}

// AccelWithCovarianceStampedSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type AccelWithCovarianceStampedSubscription struct {
	*rclgo.Subscription
}

// AccelWithCovarianceStampedSubscriptionCallback type is used to provide a subscription
// handler function for a AccelWithCovarianceStampedSubscription.
type AccelWithCovarianceStampedSubscriptionCallback func(msg *AccelWithCovarianceStamped, info *rclgo.MessageInfo, err error)

// NewAccelWithCovarianceStampedSubscription creates and returns a new subscription for the
// AccelWithCovarianceStamped
func NewAccelWithCovarianceStampedSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback AccelWithCovarianceStampedSubscriptionCallback) (*AccelWithCovarianceStampedSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg AccelWithCovarianceStamped
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, AccelWithCovarianceStampedTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &AccelWithCovarianceStampedSubscription{sub}, nil
}

func (s *AccelWithCovarianceStampedSubscription) TakeMessage(out *AccelWithCovarianceStamped) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneAccelWithCovarianceStampedSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneAccelWithCovarianceStampedSlice(dst, src []AccelWithCovarianceStamped) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var AccelWithCovarianceStampedTypeSupport types.MessageTypeSupport = _AccelWithCovarianceStampedTypeSupport{}

type _AccelWithCovarianceStampedTypeSupport struct{}

func (t _AccelWithCovarianceStampedTypeSupport) New() types.Message {
	return NewAccelWithCovarianceStamped()
}

func (t _AccelWithCovarianceStampedTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.geometry_msgs__msg__AccelWithCovarianceStamped
	return (unsafe.Pointer)(C.geometry_msgs__msg__AccelWithCovarianceStamped__create())
}

func (t _AccelWithCovarianceStampedTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.geometry_msgs__msg__AccelWithCovarianceStamped__destroy((*C.geometry_msgs__msg__AccelWithCovarianceStamped)(pointer_to_free))
}

func (t _AccelWithCovarianceStampedTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*AccelWithCovarianceStamped)
	mem := (*C.geometry_msgs__msg__AccelWithCovarianceStamped)(dst)
	std_msgs_msg.HeaderTypeSupport.AsCStruct(unsafe.Pointer(&mem.header), &m.Header)
	AccelWithCovarianceTypeSupport.AsCStruct(unsafe.Pointer(&mem.accel), &m.Accel)
}

func (t _AccelWithCovarianceStampedTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*AccelWithCovarianceStamped)
	mem := (*C.geometry_msgs__msg__AccelWithCovarianceStamped)(ros2_message_buffer)
	std_msgs_msg.HeaderTypeSupport.AsGoStruct(&m.Header, unsafe.Pointer(&mem.header))
	AccelWithCovarianceTypeSupport.AsGoStruct(&m.Accel, unsafe.Pointer(&mem.accel))
}

func (t _AccelWithCovarianceStampedTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__AccelWithCovarianceStamped())
}

type CAccelWithCovarianceStamped = C.geometry_msgs__msg__AccelWithCovarianceStamped
type CAccelWithCovarianceStamped__Sequence = C.geometry_msgs__msg__AccelWithCovarianceStamped__Sequence

func AccelWithCovarianceStamped__Sequence_to_Go(goSlice *[]AccelWithCovarianceStamped, cSlice CAccelWithCovarianceStamped__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]AccelWithCovarianceStamped, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		AccelWithCovarianceStampedTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func AccelWithCovarianceStamped__Sequence_to_C(cSlice *CAccelWithCovarianceStamped__Sequence, goSlice []AccelWithCovarianceStamped) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.geometry_msgs__msg__AccelWithCovarianceStamped)(C.malloc(C.sizeof_struct_geometry_msgs__msg__AccelWithCovarianceStamped * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		AccelWithCovarianceStampedTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func AccelWithCovarianceStamped__Array_to_Go(goSlice []AccelWithCovarianceStamped, cSlice []CAccelWithCovarianceStamped) {
	for i := 0; i < len(cSlice); i++ {
		AccelWithCovarianceStampedTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func AccelWithCovarianceStamped__Array_to_C(cSlice []CAccelWithCovarianceStamped, goSlice []AccelWithCovarianceStamped) {
	for i := 0; i < len(goSlice); i++ {
		AccelWithCovarianceStampedTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
