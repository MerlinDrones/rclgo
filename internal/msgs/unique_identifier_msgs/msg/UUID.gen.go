/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package unique_identifier_msgs_msg
import (
	"unsafe"

	"github.com/PolibaX/rclgo/pkg/rclgo"
	"github.com/PolibaX/rclgo/pkg/rclgo/types"
	"github.com/PolibaX/rclgo/pkg/rclgo/typemap"
	primitives "github.com/PolibaX/rclgo/pkg/rclgo/primitives"
	
)
/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <unique_identifier_msgs/msg/uuid.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("unique_identifier_msgs/UUID", UUIDTypeSupport)
	typemap.RegisterMessage("unique_identifier_msgs/msg/UUID", UUIDTypeSupport)
}

type UUID struct {
	Uuid [16]uint8 `yaml:"uuid"`
}

// NewUUID creates a new UUID with default values.
func NewUUID() *UUID {
	self := UUID{}
	self.SetDefaults()
	return &self
}

func (t *UUID) Clone() *UUID {
	c := &UUID{}
	c.Uuid = t.Uuid
	return c
}

func (t *UUID) CloneMsg() types.Message {
	return t.Clone()
}

func (t *UUID) SetDefaults() {
	t.Uuid = [16]uint8{}
}

func (t *UUID) GetTypeSupport() types.MessageTypeSupport {
	return UUIDTypeSupport
}

// UUIDPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type UUIDPublisher struct {
	*rclgo.Publisher
}

// NewUUIDPublisher creates and returns a new publisher for the
// UUID
func NewUUIDPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*UUIDPublisher, error) {
	pub, err := node.NewPublisher(topic_name, UUIDTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &UUIDPublisher{pub}, nil
}

func (p *UUIDPublisher) Publish(msg *UUID) error {
	return p.Publisher.Publish(msg)
}

// UUIDSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type UUIDSubscription struct {
	*rclgo.Subscription
}

// UUIDSubscriptionCallback type is used to provide a subscription
// handler function for a UUIDSubscription.
type UUIDSubscriptionCallback func(msg *UUID, info *rclgo.MessageInfo, err error)

// NewUUIDSubscription creates and returns a new subscription for the
// UUID
func NewUUIDSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback UUIDSubscriptionCallback) (*UUIDSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg UUID
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, UUIDTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &UUIDSubscription{sub}, nil
}

func (s *UUIDSubscription) TakeMessage(out *UUID) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneUUIDSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneUUIDSlice(dst, src []UUID) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var UUIDTypeSupport types.MessageTypeSupport = _UUIDTypeSupport{}

type _UUIDTypeSupport struct{}

func (t _UUIDTypeSupport) New() types.Message {
	return NewUUID()
}

func (t _UUIDTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.unique_identifier_msgs__msg__UUID
	return (unsafe.Pointer)(C.unique_identifier_msgs__msg__UUID__create())
}

func (t _UUIDTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.unique_identifier_msgs__msg__UUID__destroy((*C.unique_identifier_msgs__msg__UUID)(pointer_to_free))
}

func (t _UUIDTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*UUID)
	mem := (*C.unique_identifier_msgs__msg__UUID)(dst)
	cSlice_uuid := mem.uuid[:]
	primitives.Uint8__Array_to_C(*(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_uuid)), m.Uuid[:])
}

func (t _UUIDTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*UUID)
	mem := (*C.unique_identifier_msgs__msg__UUID)(ros2_message_buffer)
	cSlice_uuid := mem.uuid[:]
	primitives.Uint8__Array_to_Go(m.Uuid[:], *(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_uuid)))
}

func (t _UUIDTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__unique_identifier_msgs__msg__UUID())
}

type CUUID = C.unique_identifier_msgs__msg__UUID
type CUUID__Sequence = C.unique_identifier_msgs__msg__UUID__Sequence

func UUID__Sequence_to_Go(goSlice *[]UUID, cSlice CUUID__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]UUID, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		UUIDTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func UUID__Sequence_to_C(cSlice *CUUID__Sequence, goSlice []UUID) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.unique_identifier_msgs__msg__UUID)(C.malloc(C.sizeof_struct_unique_identifier_msgs__msg__UUID * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		UUIDTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func UUID__Array_to_Go(goSlice []UUID, cSlice []CUUID) {
	for i := 0; i < len(cSlice); i++ {
		UUIDTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func UUID__Array_to_C(cSlice []CUUID, goSlice []UUID) {
	for i := 0; i < len(goSlice); i++ {
		UUIDTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
