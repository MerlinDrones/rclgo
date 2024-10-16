/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package test_msgs_msg

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <test_msgs/msg/multi_nested.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("test_msgs/MultiNested", MultiNestedTypeSupport)
	typemap.RegisterMessage("test_msgs/msg/MultiNested", MultiNestedTypeSupport)
}

type MultiNested struct {
	ArrayOfArrays                         [3]Arrays             `yaml:"array_of_arrays"` // Mulitple levels of nested messages
	ArrayOfBoundedSequences               [3]BoundedSequences   `yaml:"array_of_bounded_sequences"`
	ArrayOfUnboundedSequences             [3]UnboundedSequences `yaml:"array_of_unbounded_sequences"`
	BoundedSequenceOfArrays               []Arrays              `yaml:"bounded_sequence_of_arrays"`
	BoundedSequenceOfBoundedSequences     []BoundedSequences    `yaml:"bounded_sequence_of_bounded_sequences"`
	BoundedSequenceOfUnboundedSequences   []UnboundedSequences  `yaml:"bounded_sequence_of_unbounded_sequences"`
	UnboundedSequenceOfArrays             []Arrays              `yaml:"unbounded_sequence_of_arrays"`
	UnboundedSequenceOfBoundedSequences   []BoundedSequences    `yaml:"unbounded_sequence_of_bounded_sequences"`
	UnboundedSequenceOfUnboundedSequences []UnboundedSequences  `yaml:"unbounded_sequence_of_unbounded_sequences"`
}

// NewMultiNested creates a new MultiNested with default values.
func NewMultiNested() *MultiNested {
	self := MultiNested{}
	self.SetDefaults()
	return &self
}

func (t *MultiNested) Clone() *MultiNested {
	c := &MultiNested{}
	CloneArraysSlice(c.ArrayOfArrays[:], t.ArrayOfArrays[:])
	CloneBoundedSequencesSlice(c.ArrayOfBoundedSequences[:], t.ArrayOfBoundedSequences[:])
	CloneUnboundedSequencesSlice(c.ArrayOfUnboundedSequences[:], t.ArrayOfUnboundedSequences[:])
	if t.BoundedSequenceOfArrays != nil {
		c.BoundedSequenceOfArrays = make([]Arrays, len(t.BoundedSequenceOfArrays))
		CloneArraysSlice(c.BoundedSequenceOfArrays, t.BoundedSequenceOfArrays)
	}
	if t.BoundedSequenceOfBoundedSequences != nil {
		c.BoundedSequenceOfBoundedSequences = make([]BoundedSequences, len(t.BoundedSequenceOfBoundedSequences))
		CloneBoundedSequencesSlice(c.BoundedSequenceOfBoundedSequences, t.BoundedSequenceOfBoundedSequences)
	}
	if t.BoundedSequenceOfUnboundedSequences != nil {
		c.BoundedSequenceOfUnboundedSequences = make([]UnboundedSequences, len(t.BoundedSequenceOfUnboundedSequences))
		CloneUnboundedSequencesSlice(c.BoundedSequenceOfUnboundedSequences, t.BoundedSequenceOfUnboundedSequences)
	}
	if t.UnboundedSequenceOfArrays != nil {
		c.UnboundedSequenceOfArrays = make([]Arrays, len(t.UnboundedSequenceOfArrays))
		CloneArraysSlice(c.UnboundedSequenceOfArrays, t.UnboundedSequenceOfArrays)
	}
	if t.UnboundedSequenceOfBoundedSequences != nil {
		c.UnboundedSequenceOfBoundedSequences = make([]BoundedSequences, len(t.UnboundedSequenceOfBoundedSequences))
		CloneBoundedSequencesSlice(c.UnboundedSequenceOfBoundedSequences, t.UnboundedSequenceOfBoundedSequences)
	}
	if t.UnboundedSequenceOfUnboundedSequences != nil {
		c.UnboundedSequenceOfUnboundedSequences = make([]UnboundedSequences, len(t.UnboundedSequenceOfUnboundedSequences))
		CloneUnboundedSequencesSlice(c.UnboundedSequenceOfUnboundedSequences, t.UnboundedSequenceOfUnboundedSequences)
	}
	return c
}

func (t *MultiNested) CloneMsg() types.Message {
	return t.Clone()
}

func (t *MultiNested) SetDefaults() {
	for i := range t.ArrayOfArrays {
		t.ArrayOfArrays[i].SetDefaults()
	}
	for i := range t.ArrayOfBoundedSequences {
		t.ArrayOfBoundedSequences[i].SetDefaults()
	}
	for i := range t.ArrayOfUnboundedSequences {
		t.ArrayOfUnboundedSequences[i].SetDefaults()
	}
	t.BoundedSequenceOfArrays = nil
	t.BoundedSequenceOfBoundedSequences = nil
	t.BoundedSequenceOfUnboundedSequences = nil
	t.UnboundedSequenceOfArrays = nil
	t.UnboundedSequenceOfBoundedSequences = nil
	t.UnboundedSequenceOfUnboundedSequences = nil
}

func (t *MultiNested) GetTypeSupport() types.MessageTypeSupport {
	return MultiNestedTypeSupport
}

// MultiNestedPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type MultiNestedPublisher struct {
	*rclgo.Publisher
}

// NewMultiNestedPublisher creates and returns a new publisher for the
// MultiNested
func NewMultiNestedPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*MultiNestedPublisher, error) {
	pub, err := node.NewPublisher(topic_name, MultiNestedTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &MultiNestedPublisher{pub}, nil
}

func (p *MultiNestedPublisher) Publish(msg *MultiNested) error {
	return p.Publisher.Publish(msg)
}

// MultiNestedSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type MultiNestedSubscription struct {
	*rclgo.Subscription
}

// MultiNestedSubscriptionCallback type is used to provide a subscription
// handler function for a MultiNestedSubscription.
type MultiNestedSubscriptionCallback func(msg *MultiNested, info *rclgo.MessageInfo, err error)

// NewMultiNestedSubscription creates and returns a new subscription for the
// MultiNested
func NewMultiNestedSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback MultiNestedSubscriptionCallback) (*MultiNestedSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg MultiNested
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, MultiNestedTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &MultiNestedSubscription{sub}, nil
}

func (s *MultiNestedSubscription) TakeMessage(out *MultiNested) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneMultiNestedSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneMultiNestedSlice(dst, src []MultiNested) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var MultiNestedTypeSupport types.MessageTypeSupport = _MultiNestedTypeSupport{}

type _MultiNestedTypeSupport struct{}

func (t _MultiNestedTypeSupport) New() types.Message {
	return NewMultiNested()
}

func (t _MultiNestedTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.test_msgs__msg__MultiNested
	return (unsafe.Pointer)(C.test_msgs__msg__MultiNested__create())
}

func (t _MultiNestedTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.test_msgs__msg__MultiNested__destroy((*C.test_msgs__msg__MultiNested)(pointer_to_free))
}

func (t _MultiNestedTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*MultiNested)
	mem := (*C.test_msgs__msg__MultiNested)(dst)
	Arrays__Array_to_C(mem.array_of_arrays[:], m.ArrayOfArrays[:])
	BoundedSequences__Array_to_C(mem.array_of_bounded_sequences[:], m.ArrayOfBoundedSequences[:])
	UnboundedSequences__Array_to_C(mem.array_of_unbounded_sequences[:], m.ArrayOfUnboundedSequences[:])
	Arrays__Sequence_to_C(&mem.bounded_sequence_of_arrays, m.BoundedSequenceOfArrays)
	BoundedSequences__Sequence_to_C(&mem.bounded_sequence_of_bounded_sequences, m.BoundedSequenceOfBoundedSequences)
	UnboundedSequences__Sequence_to_C(&mem.bounded_sequence_of_unbounded_sequences, m.BoundedSequenceOfUnboundedSequences)
	Arrays__Sequence_to_C(&mem.unbounded_sequence_of_arrays, m.UnboundedSequenceOfArrays)
	BoundedSequences__Sequence_to_C(&mem.unbounded_sequence_of_bounded_sequences, m.UnboundedSequenceOfBoundedSequences)
	UnboundedSequences__Sequence_to_C(&mem.unbounded_sequence_of_unbounded_sequences, m.UnboundedSequenceOfUnboundedSequences)
}

func (t _MultiNestedTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*MultiNested)
	mem := (*C.test_msgs__msg__MultiNested)(ros2_message_buffer)
	Arrays__Array_to_Go(m.ArrayOfArrays[:], mem.array_of_arrays[:])
	BoundedSequences__Array_to_Go(m.ArrayOfBoundedSequences[:], mem.array_of_bounded_sequences[:])
	UnboundedSequences__Array_to_Go(m.ArrayOfUnboundedSequences[:], mem.array_of_unbounded_sequences[:])
	Arrays__Sequence_to_Go(&m.BoundedSequenceOfArrays, mem.bounded_sequence_of_arrays)
	BoundedSequences__Sequence_to_Go(&m.BoundedSequenceOfBoundedSequences, mem.bounded_sequence_of_bounded_sequences)
	UnboundedSequences__Sequence_to_Go(&m.BoundedSequenceOfUnboundedSequences, mem.bounded_sequence_of_unbounded_sequences)
	Arrays__Sequence_to_Go(&m.UnboundedSequenceOfArrays, mem.unbounded_sequence_of_arrays)
	BoundedSequences__Sequence_to_Go(&m.UnboundedSequenceOfBoundedSequences, mem.unbounded_sequence_of_bounded_sequences)
	UnboundedSequences__Sequence_to_Go(&m.UnboundedSequenceOfUnboundedSequences, mem.unbounded_sequence_of_unbounded_sequences)
}

func (t _MultiNestedTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__MultiNested())
}

type CMultiNested = C.test_msgs__msg__MultiNested
type CMultiNested__Sequence = C.test_msgs__msg__MultiNested__Sequence

func MultiNested__Sequence_to_Go(goSlice *[]MultiNested, cSlice CMultiNested__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]MultiNested, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		MultiNestedTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func MultiNested__Sequence_to_C(cSlice *CMultiNested__Sequence, goSlice []MultiNested) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.test_msgs__msg__MultiNested)(C.malloc(C.sizeof_struct_test_msgs__msg__MultiNested * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		MultiNestedTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func MultiNested__Array_to_Go(goSlice []MultiNested, cSlice []CMultiNested) {
	for i := 0; i < len(cSlice); i++ {
		MultiNestedTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func MultiNested__Array_to_C(cSlice []CMultiNested, goSlice []MultiNested) {
	for i := 0; i < len(goSlice); i++ {
		MultiNestedTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
