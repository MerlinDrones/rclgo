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
	
)
/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <geometry_msgs/msg/point32.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("geometry_msgs/Point32", Point32TypeSupport)
	typemap.RegisterMessage("geometry_msgs/msg/Point32", Point32TypeSupport)
}

type Point32 struct {
	X float32 `yaml:"x"`
	Y float32 `yaml:"y"`
	Z float32 `yaml:"z"`
}

// NewPoint32 creates a new Point32 with default values.
func NewPoint32() *Point32 {
	self := Point32{}
	self.SetDefaults()
	return &self
}

func (t *Point32) Clone() *Point32 {
	c := &Point32{}
	c.X = t.X
	c.Y = t.Y
	c.Z = t.Z
	return c
}

func (t *Point32) CloneMsg() types.Message {
	return t.Clone()
}

func (t *Point32) SetDefaults() {
	t.X = 0
	t.Y = 0
	t.Z = 0
}

func (t *Point32) GetTypeSupport() types.MessageTypeSupport {
	return Point32TypeSupport
}

// Point32Publisher wraps rclgo.Publisher to provide type safe helper
// functions
type Point32Publisher struct {
	*rclgo.Publisher
}

// NewPoint32Publisher creates and returns a new publisher for the
// Point32
func NewPoint32Publisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*Point32Publisher, error) {
	pub, err := node.NewPublisher(topic_name, Point32TypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &Point32Publisher{pub}, nil
}

func (p *Point32Publisher) Publish(msg *Point32) error {
	return p.Publisher.Publish(msg)
}

// Point32Subscription wraps rclgo.Subscription to provide type safe helper
// functions
type Point32Subscription struct {
	*rclgo.Subscription
}

// Point32SubscriptionCallback type is used to provide a subscription
// handler function for a Point32Subscription.
type Point32SubscriptionCallback func(msg *Point32, info *rclgo.MessageInfo, err error)

// NewPoint32Subscription creates and returns a new subscription for the
// Point32
func NewPoint32Subscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback Point32SubscriptionCallback) (*Point32Subscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg Point32
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, Point32TypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &Point32Subscription{sub}, nil
}

func (s *Point32Subscription) TakeMessage(out *Point32) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// ClonePoint32Slice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func ClonePoint32Slice(dst, src []Point32) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var Point32TypeSupport types.MessageTypeSupport = _Point32TypeSupport{}

type _Point32TypeSupport struct{}

func (t _Point32TypeSupport) New() types.Message {
	return NewPoint32()
}

func (t _Point32TypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.geometry_msgs__msg__Point32
	return (unsafe.Pointer)(C.geometry_msgs__msg__Point32__create())
}

func (t _Point32TypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.geometry_msgs__msg__Point32__destroy((*C.geometry_msgs__msg__Point32)(pointer_to_free))
}

func (t _Point32TypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*Point32)
	mem := (*C.geometry_msgs__msg__Point32)(dst)
	mem.x = C.float(m.X)
	mem.y = C.float(m.Y)
	mem.z = C.float(m.Z)
}

func (t _Point32TypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*Point32)
	mem := (*C.geometry_msgs__msg__Point32)(ros2_message_buffer)
	m.X = float32(mem.x)
	m.Y = float32(mem.y)
	m.Z = float32(mem.z)
}

func (t _Point32TypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Point32())
}

type CPoint32 = C.geometry_msgs__msg__Point32
type CPoint32__Sequence = C.geometry_msgs__msg__Point32__Sequence

func Point32__Sequence_to_Go(goSlice *[]Point32, cSlice CPoint32__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]Point32, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		Point32TypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func Point32__Sequence_to_C(cSlice *CPoint32__Sequence, goSlice []Point32) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.geometry_msgs__msg__Point32)(C.malloc(C.sizeof_struct_geometry_msgs__msg__Point32 * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		Point32TypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func Point32__Array_to_Go(goSlice []Point32, cSlice []CPoint32) {
	for i := 0; i < len(cSlice); i++ {
		Point32TypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func Point32__Array_to_C(cSlice []CPoint32, goSlice []Point32) {
	for i := 0; i < len(goSlice); i++ {
		Point32TypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
