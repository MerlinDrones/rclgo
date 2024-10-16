// Code generated by rclgo-gen. DO NOT EDIT.

package px4_msgs_msg

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <px4_msgs/msg/parameter_reset_request.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/ParameterResetRequest", ParameterResetRequestTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/ParameterResetRequest", ParameterResetRequestTypeSupport)
}

const (
	ParameterResetRequest_ORB_QUEUE_LENGTH uint8 = 4
)

type ParameterResetRequest struct {
	Timestamp      uint64 `yaml:"timestamp"`
	ParameterIndex uint16 `yaml:"parameter_index"`
	ResetAll       bool   `yaml:"reset_all"` // If this is true then ignore parameter_index
}

// NewParameterResetRequest creates a new ParameterResetRequest with default values.
func NewParameterResetRequest() *ParameterResetRequest {
	self := ParameterResetRequest{}
	self.SetDefaults()
	return &self
}

func (t *ParameterResetRequest) Clone() *ParameterResetRequest {
	c := &ParameterResetRequest{}
	c.Timestamp = t.Timestamp
	c.ParameterIndex = t.ParameterIndex
	c.ResetAll = t.ResetAll
	return c
}

func (t *ParameterResetRequest) CloneMsg() types.Message {
	return t.Clone()
}

func (t *ParameterResetRequest) SetDefaults() {
	t.Timestamp = 0
	t.ParameterIndex = 0
	t.ResetAll = false
}

func (t *ParameterResetRequest) GetTypeSupport() types.MessageTypeSupport {
	return ParameterResetRequestTypeSupport
}

// ParameterResetRequestPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type ParameterResetRequestPublisher struct {
	*rclgo.Publisher
}

// NewParameterResetRequestPublisher creates and returns a new publisher for the
// ParameterResetRequest
func NewParameterResetRequestPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*ParameterResetRequestPublisher, error) {
	pub, err := node.NewPublisher(topic_name, ParameterResetRequestTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &ParameterResetRequestPublisher{pub}, nil
}

func (p *ParameterResetRequestPublisher) Publish(msg *ParameterResetRequest) error {
	return p.Publisher.Publish(msg)
}

// ParameterResetRequestSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type ParameterResetRequestSubscription struct {
	*rclgo.Subscription
}

// ParameterResetRequestSubscriptionCallback type is used to provide a subscription
// handler function for a ParameterResetRequestSubscription.
type ParameterResetRequestSubscriptionCallback func(msg *ParameterResetRequest, info *rclgo.MessageInfo, err error)

// NewParameterResetRequestSubscription creates and returns a new subscription for the
// ParameterResetRequest
func NewParameterResetRequestSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback ParameterResetRequestSubscriptionCallback) (*ParameterResetRequestSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg ParameterResetRequest
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, ParameterResetRequestTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &ParameterResetRequestSubscription{sub}, nil
}

func (s *ParameterResetRequestSubscription) TakeMessage(out *ParameterResetRequest) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneParameterResetRequestSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneParameterResetRequestSlice(dst, src []ParameterResetRequest) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var ParameterResetRequestTypeSupport types.MessageTypeSupport = _ParameterResetRequestTypeSupport{}

type _ParameterResetRequestTypeSupport struct{}

func (t _ParameterResetRequestTypeSupport) New() types.Message {
	return NewParameterResetRequest()
}

func (t _ParameterResetRequestTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__ParameterResetRequest
	return (unsafe.Pointer)(C.px4_msgs__msg__ParameterResetRequest__create())
}

func (t _ParameterResetRequestTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__ParameterResetRequest__destroy((*C.px4_msgs__msg__ParameterResetRequest)(pointer_to_free))
}

func (t _ParameterResetRequestTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*ParameterResetRequest)
	mem := (*C.px4_msgs__msg__ParameterResetRequest)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.parameter_index = C.uint16_t(m.ParameterIndex)
	mem.reset_all = C.bool(m.ResetAll)
}

func (t _ParameterResetRequestTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*ParameterResetRequest)
	mem := (*C.px4_msgs__msg__ParameterResetRequest)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.ParameterIndex = uint16(mem.parameter_index)
	m.ResetAll = bool(mem.reset_all)
}

func (t _ParameterResetRequestTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__ParameterResetRequest())
}

type CParameterResetRequest = C.px4_msgs__msg__ParameterResetRequest
type CParameterResetRequest__Sequence = C.px4_msgs__msg__ParameterResetRequest__Sequence

func ParameterResetRequest__Sequence_to_Go(goSlice *[]ParameterResetRequest, cSlice CParameterResetRequest__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]ParameterResetRequest, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		ParameterResetRequestTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func ParameterResetRequest__Sequence_to_C(cSlice *CParameterResetRequest__Sequence, goSlice []ParameterResetRequest) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__ParameterResetRequest)(C.malloc(C.sizeof_struct_px4_msgs__msg__ParameterResetRequest * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		ParameterResetRequestTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func ParameterResetRequest__Array_to_Go(goSlice []ParameterResetRequest, cSlice []CParameterResetRequest) {
	for i := 0; i < len(cSlice); i++ {
		ParameterResetRequestTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func ParameterResetRequest__Array_to_C(cSlice []CParameterResetRequest, goSlice []ParameterResetRequest) {
	for i := 0; i < len(goSlice); i++ {
		ParameterResetRequestTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
