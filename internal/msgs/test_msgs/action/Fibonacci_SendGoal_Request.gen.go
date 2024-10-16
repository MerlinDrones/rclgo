/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package test_msgs_action

import (
	"unsafe"

	unique_identifier_msgs_msg "github.com/merlindrones/rclgo/internal/msgs/unique_identifier_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <test_msgs/action/fibonacci.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("test_msgs/Fibonacci_SendGoal_Request", Fibonacci_SendGoal_RequestTypeSupport)
	typemap.RegisterMessage("test_msgs/action/Fibonacci_SendGoal_Request", Fibonacci_SendGoal_RequestTypeSupport)
}

type Fibonacci_SendGoal_Request struct {
	GoalID unique_identifier_msgs_msg.UUID `yaml:"goal_id"`
	Goal   Fibonacci_Goal                  `yaml:"goal"`
}

// NewFibonacci_SendGoal_Request creates a new Fibonacci_SendGoal_Request with default values.
func NewFibonacci_SendGoal_Request() *Fibonacci_SendGoal_Request {
	self := Fibonacci_SendGoal_Request{}
	self.SetDefaults()
	return &self
}

func (t *Fibonacci_SendGoal_Request) Clone() *Fibonacci_SendGoal_Request {
	c := &Fibonacci_SendGoal_Request{}
	c.GoalID = *t.GoalID.Clone()
	c.Goal = *t.Goal.Clone()
	return c
}

func (t *Fibonacci_SendGoal_Request) CloneMsg() types.Message {
	return t.Clone()
}

func (t *Fibonacci_SendGoal_Request) SetDefaults() {
	t.GoalID.SetDefaults()
	t.Goal.SetDefaults()
}

func (t *Fibonacci_SendGoal_Request) GetTypeSupport() types.MessageTypeSupport {
	return Fibonacci_SendGoal_RequestTypeSupport
}
func (t *Fibonacci_SendGoal_Request) GetGoalID() *types.GoalID {
	return (*types.GoalID)(&t.GoalID.Uuid)
}

func (t *Fibonacci_SendGoal_Request) SetGoalID(id *types.GoalID) {
	t.GoalID.Uuid = *id
}
func (t *Fibonacci_SendGoal_Request) GetGoalDescription() types.Message {
	return &t.Goal
}

func (t *Fibonacci_SendGoal_Request) SetGoalDescription(desc types.Message) {
	t.Goal = *desc.(*Fibonacci_Goal)
}

// Fibonacci_SendGoal_RequestPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type Fibonacci_SendGoal_RequestPublisher struct {
	*rclgo.Publisher
}

// NewFibonacci_SendGoal_RequestPublisher creates and returns a new publisher for the
// Fibonacci_SendGoal_Request
func NewFibonacci_SendGoal_RequestPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*Fibonacci_SendGoal_RequestPublisher, error) {
	pub, err := node.NewPublisher(topic_name, Fibonacci_SendGoal_RequestTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &Fibonacci_SendGoal_RequestPublisher{pub}, nil
}

func (p *Fibonacci_SendGoal_RequestPublisher) Publish(msg *Fibonacci_SendGoal_Request) error {
	return p.Publisher.Publish(msg)
}

// Fibonacci_SendGoal_RequestSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type Fibonacci_SendGoal_RequestSubscription struct {
	*rclgo.Subscription
}

// Fibonacci_SendGoal_RequestSubscriptionCallback type is used to provide a subscription
// handler function for a Fibonacci_SendGoal_RequestSubscription.
type Fibonacci_SendGoal_RequestSubscriptionCallback func(msg *Fibonacci_SendGoal_Request, info *rclgo.MessageInfo, err error)

// NewFibonacci_SendGoal_RequestSubscription creates and returns a new subscription for the
// Fibonacci_SendGoal_Request
func NewFibonacci_SendGoal_RequestSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback Fibonacci_SendGoal_RequestSubscriptionCallback) (*Fibonacci_SendGoal_RequestSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg Fibonacci_SendGoal_Request
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, Fibonacci_SendGoal_RequestTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &Fibonacci_SendGoal_RequestSubscription{sub}, nil
}

func (s *Fibonacci_SendGoal_RequestSubscription) TakeMessage(out *Fibonacci_SendGoal_Request) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneFibonacci_SendGoal_RequestSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneFibonacci_SendGoal_RequestSlice(dst, src []Fibonacci_SendGoal_Request) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var Fibonacci_SendGoal_RequestTypeSupport types.MessageTypeSupport = _Fibonacci_SendGoal_RequestTypeSupport{}

type _Fibonacci_SendGoal_RequestTypeSupport struct{}

func (t _Fibonacci_SendGoal_RequestTypeSupport) New() types.Message {
	return NewFibonacci_SendGoal_Request()
}

func (t _Fibonacci_SendGoal_RequestTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.test_msgs__action__Fibonacci_SendGoal_Request
	return (unsafe.Pointer)(C.test_msgs__action__Fibonacci_SendGoal_Request__create())
}

func (t _Fibonacci_SendGoal_RequestTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.test_msgs__action__Fibonacci_SendGoal_Request__destroy((*C.test_msgs__action__Fibonacci_SendGoal_Request)(pointer_to_free))
}

func (t _Fibonacci_SendGoal_RequestTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*Fibonacci_SendGoal_Request)
	mem := (*C.test_msgs__action__Fibonacci_SendGoal_Request)(dst)
	unique_identifier_msgs_msg.UUIDTypeSupport.AsCStruct(unsafe.Pointer(&mem.goal_id), &m.GoalID)
	Fibonacci_GoalTypeSupport.AsCStruct(unsafe.Pointer(&mem.goal), &m.Goal)
}

func (t _Fibonacci_SendGoal_RequestTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*Fibonacci_SendGoal_Request)
	mem := (*C.test_msgs__action__Fibonacci_SendGoal_Request)(ros2_message_buffer)
	unique_identifier_msgs_msg.UUIDTypeSupport.AsGoStruct(&m.GoalID, unsafe.Pointer(&mem.goal_id))
	Fibonacci_GoalTypeSupport.AsGoStruct(&m.Goal, unsafe.Pointer(&mem.goal))
}

func (t _Fibonacci_SendGoal_RequestTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__test_msgs__action__Fibonacci_SendGoal_Request())
}

type CFibonacci_SendGoal_Request = C.test_msgs__action__Fibonacci_SendGoal_Request
type CFibonacci_SendGoal_Request__Sequence = C.test_msgs__action__Fibonacci_SendGoal_Request__Sequence

func Fibonacci_SendGoal_Request__Sequence_to_Go(goSlice *[]Fibonacci_SendGoal_Request, cSlice CFibonacci_SendGoal_Request__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]Fibonacci_SendGoal_Request, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		Fibonacci_SendGoal_RequestTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func Fibonacci_SendGoal_Request__Sequence_to_C(cSlice *CFibonacci_SendGoal_Request__Sequence, goSlice []Fibonacci_SendGoal_Request) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.test_msgs__action__Fibonacci_SendGoal_Request)(C.malloc(C.sizeof_struct_test_msgs__action__Fibonacci_SendGoal_Request * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		Fibonacci_SendGoal_RequestTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func Fibonacci_SendGoal_Request__Array_to_Go(goSlice []Fibonacci_SendGoal_Request, cSlice []CFibonacci_SendGoal_Request) {
	for i := 0; i < len(cSlice); i++ {
		Fibonacci_SendGoal_RequestTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func Fibonacci_SendGoal_Request__Array_to_C(cSlice []CFibonacci_SendGoal_Request, goSlice []Fibonacci_SendGoal_Request) {
	for i := 0; i < len(goSlice); i++ {
		Fibonacci_SendGoal_RequestTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
