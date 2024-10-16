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

#include <px4_msgs/msg/arming_check_request.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/ArmingCheckRequest", ArmingCheckRequestTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/ArmingCheckRequest", ArmingCheckRequestTypeSupport)
}

type ArmingCheckRequest struct {
	Timestamp uint64 `yaml:"timestamp"` // time since system start (microseconds)
	RequestId uint8  `yaml:"request_id"`
}

// NewArmingCheckRequest creates a new ArmingCheckRequest with default values.
func NewArmingCheckRequest() *ArmingCheckRequest {
	self := ArmingCheckRequest{}
	self.SetDefaults()
	return &self
}

func (t *ArmingCheckRequest) Clone() *ArmingCheckRequest {
	c := &ArmingCheckRequest{}
	c.Timestamp = t.Timestamp
	c.RequestId = t.RequestId
	return c
}

func (t *ArmingCheckRequest) CloneMsg() types.Message {
	return t.Clone()
}

func (t *ArmingCheckRequest) SetDefaults() {
	t.Timestamp = 0
	t.RequestId = 0
}

func (t *ArmingCheckRequest) GetTypeSupport() types.MessageTypeSupport {
	return ArmingCheckRequestTypeSupport
}

// ArmingCheckRequestPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type ArmingCheckRequestPublisher struct {
	*rclgo.Publisher
}

// NewArmingCheckRequestPublisher creates and returns a new publisher for the
// ArmingCheckRequest
func NewArmingCheckRequestPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*ArmingCheckRequestPublisher, error) {
	pub, err := node.NewPublisher(topic_name, ArmingCheckRequestTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &ArmingCheckRequestPublisher{pub}, nil
}

func (p *ArmingCheckRequestPublisher) Publish(msg *ArmingCheckRequest) error {
	return p.Publisher.Publish(msg)
}

// ArmingCheckRequestSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type ArmingCheckRequestSubscription struct {
	*rclgo.Subscription
}

// ArmingCheckRequestSubscriptionCallback type is used to provide a subscription
// handler function for a ArmingCheckRequestSubscription.
type ArmingCheckRequestSubscriptionCallback func(msg *ArmingCheckRequest, info *rclgo.MessageInfo, err error)

// NewArmingCheckRequestSubscription creates and returns a new subscription for the
// ArmingCheckRequest
func NewArmingCheckRequestSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback ArmingCheckRequestSubscriptionCallback) (*ArmingCheckRequestSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg ArmingCheckRequest
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, ArmingCheckRequestTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &ArmingCheckRequestSubscription{sub}, nil
}

func (s *ArmingCheckRequestSubscription) TakeMessage(out *ArmingCheckRequest) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneArmingCheckRequestSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneArmingCheckRequestSlice(dst, src []ArmingCheckRequest) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var ArmingCheckRequestTypeSupport types.MessageTypeSupport = _ArmingCheckRequestTypeSupport{}

type _ArmingCheckRequestTypeSupport struct{}

func (t _ArmingCheckRequestTypeSupport) New() types.Message {
	return NewArmingCheckRequest()
}

func (t _ArmingCheckRequestTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__ArmingCheckRequest
	return (unsafe.Pointer)(C.px4_msgs__msg__ArmingCheckRequest__create())
}

func (t _ArmingCheckRequestTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__ArmingCheckRequest__destroy((*C.px4_msgs__msg__ArmingCheckRequest)(pointer_to_free))
}

func (t _ArmingCheckRequestTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*ArmingCheckRequest)
	mem := (*C.px4_msgs__msg__ArmingCheckRequest)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.request_id = C.uint8_t(m.RequestId)
}

func (t _ArmingCheckRequestTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*ArmingCheckRequest)
	mem := (*C.px4_msgs__msg__ArmingCheckRequest)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.RequestId = uint8(mem.request_id)
}

func (t _ArmingCheckRequestTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__ArmingCheckRequest())
}

type CArmingCheckRequest = C.px4_msgs__msg__ArmingCheckRequest
type CArmingCheckRequest__Sequence = C.px4_msgs__msg__ArmingCheckRequest__Sequence

func ArmingCheckRequest__Sequence_to_Go(goSlice *[]ArmingCheckRequest, cSlice CArmingCheckRequest__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]ArmingCheckRequest, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		ArmingCheckRequestTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func ArmingCheckRequest__Sequence_to_C(cSlice *CArmingCheckRequest__Sequence, goSlice []ArmingCheckRequest) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__ArmingCheckRequest)(C.malloc(C.sizeof_struct_px4_msgs__msg__ArmingCheckRequest * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		ArmingCheckRequestTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func ArmingCheckRequest__Array_to_Go(goSlice []ArmingCheckRequest, cSlice []CArmingCheckRequest) {
	for i := 0; i < len(cSlice); i++ {
		ArmingCheckRequestTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func ArmingCheckRequest__Array_to_C(cSlice []CArmingCheckRequest, goSlice []ArmingCheckRequest) {
	for i := 0; i < len(goSlice); i++ {
		ArmingCheckRequestTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
