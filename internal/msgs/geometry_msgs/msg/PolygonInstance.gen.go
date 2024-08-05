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

#include <geometry_msgs/msg/polygon_instance.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("geometry_msgs/PolygonInstance", PolygonInstanceTypeSupport)
	typemap.RegisterMessage("geometry_msgs/msg/PolygonInstance", PolygonInstanceTypeSupport)
}

type PolygonInstance struct {
	Polygon Polygon `yaml:"polygon"`
	Id int64 `yaml:"id"`
}

// NewPolygonInstance creates a new PolygonInstance with default values.
func NewPolygonInstance() *PolygonInstance {
	self := PolygonInstance{}
	self.SetDefaults()
	return &self
}

func (t *PolygonInstance) Clone() *PolygonInstance {
	c := &PolygonInstance{}
	c.Polygon = *t.Polygon.Clone()
	c.Id = t.Id
	return c
}

func (t *PolygonInstance) CloneMsg() types.Message {
	return t.Clone()
}

func (t *PolygonInstance) SetDefaults() {
	t.Polygon.SetDefaults()
	t.Id = 0
}

func (t *PolygonInstance) GetTypeSupport() types.MessageTypeSupport {
	return PolygonInstanceTypeSupport
}

// PolygonInstancePublisher wraps rclgo.Publisher to provide type safe helper
// functions
type PolygonInstancePublisher struct {
	*rclgo.Publisher
}

// NewPolygonInstancePublisher creates and returns a new publisher for the
// PolygonInstance
func NewPolygonInstancePublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*PolygonInstancePublisher, error) {
	pub, err := node.NewPublisher(topic_name, PolygonInstanceTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &PolygonInstancePublisher{pub}, nil
}

func (p *PolygonInstancePublisher) Publish(msg *PolygonInstance) error {
	return p.Publisher.Publish(msg)
}

// PolygonInstanceSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type PolygonInstanceSubscription struct {
	*rclgo.Subscription
}

// PolygonInstanceSubscriptionCallback type is used to provide a subscription
// handler function for a PolygonInstanceSubscription.
type PolygonInstanceSubscriptionCallback func(msg *PolygonInstance, info *rclgo.MessageInfo, err error)

// NewPolygonInstanceSubscription creates and returns a new subscription for the
// PolygonInstance
func NewPolygonInstanceSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback PolygonInstanceSubscriptionCallback) (*PolygonInstanceSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg PolygonInstance
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, PolygonInstanceTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &PolygonInstanceSubscription{sub}, nil
}

func (s *PolygonInstanceSubscription) TakeMessage(out *PolygonInstance) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// ClonePolygonInstanceSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func ClonePolygonInstanceSlice(dst, src []PolygonInstance) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var PolygonInstanceTypeSupport types.MessageTypeSupport = _PolygonInstanceTypeSupport{}

type _PolygonInstanceTypeSupport struct{}

func (t _PolygonInstanceTypeSupport) New() types.Message {
	return NewPolygonInstance()
}

func (t _PolygonInstanceTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.geometry_msgs__msg__PolygonInstance
	return (unsafe.Pointer)(C.geometry_msgs__msg__PolygonInstance__create())
}

func (t _PolygonInstanceTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.geometry_msgs__msg__PolygonInstance__destroy((*C.geometry_msgs__msg__PolygonInstance)(pointer_to_free))
}

func (t _PolygonInstanceTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*PolygonInstance)
	mem := (*C.geometry_msgs__msg__PolygonInstance)(dst)
	PolygonTypeSupport.AsCStruct(unsafe.Pointer(&mem.polygon), &m.Polygon)
	mem.id = C.int64_t(m.Id)
}

func (t _PolygonInstanceTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*PolygonInstance)
	mem := (*C.geometry_msgs__msg__PolygonInstance)(ros2_message_buffer)
	PolygonTypeSupport.AsGoStruct(&m.Polygon, unsafe.Pointer(&mem.polygon))
	m.Id = int64(mem.id)
}

func (t _PolygonInstanceTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PolygonInstance())
}

type CPolygonInstance = C.geometry_msgs__msg__PolygonInstance
type CPolygonInstance__Sequence = C.geometry_msgs__msg__PolygonInstance__Sequence

func PolygonInstance__Sequence_to_Go(goSlice *[]PolygonInstance, cSlice CPolygonInstance__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]PolygonInstance, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		PolygonInstanceTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func PolygonInstance__Sequence_to_C(cSlice *CPolygonInstance__Sequence, goSlice []PolygonInstance) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.geometry_msgs__msg__PolygonInstance)(C.malloc(C.sizeof_struct_geometry_msgs__msg__PolygonInstance * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		PolygonInstanceTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func PolygonInstance__Array_to_Go(goSlice []PolygonInstance, cSlice []CPolygonInstance) {
	for i := 0; i < len(cSlice); i++ {
		PolygonInstanceTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func PolygonInstance__Array_to_C(cSlice []CPolygonInstance, goSlice []PolygonInstance) {
	for i := 0; i < len(goSlice); i++ {
		PolygonInstanceTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
