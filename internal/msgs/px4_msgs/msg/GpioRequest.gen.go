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

#include <px4_msgs/msg/gpio_request.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/GpioRequest", GpioRequestTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/GpioRequest", GpioRequestTypeSupport)
}

type GpioRequest struct {
	Timestamp uint64 `yaml:"timestamp"` // time since system start (microseconds)
	DeviceId  uint32 `yaml:"device_id"` // Device id
}

// NewGpioRequest creates a new GpioRequest with default values.
func NewGpioRequest() *GpioRequest {
	self := GpioRequest{}
	self.SetDefaults()
	return &self
}

func (t *GpioRequest) Clone() *GpioRequest {
	c := &GpioRequest{}
	c.Timestamp = t.Timestamp
	c.DeviceId = t.DeviceId
	return c
}

func (t *GpioRequest) CloneMsg() types.Message {
	return t.Clone()
}

func (t *GpioRequest) SetDefaults() {
	t.Timestamp = 0
	t.DeviceId = 0
}

func (t *GpioRequest) GetTypeSupport() types.MessageTypeSupport {
	return GpioRequestTypeSupport
}

// GpioRequestPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type GpioRequestPublisher struct {
	*rclgo.Publisher
}

// NewGpioRequestPublisher creates and returns a new publisher for the
// GpioRequest
func NewGpioRequestPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*GpioRequestPublisher, error) {
	pub, err := node.NewPublisher(topic_name, GpioRequestTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &GpioRequestPublisher{pub}, nil
}

func (p *GpioRequestPublisher) Publish(msg *GpioRequest) error {
	return p.Publisher.Publish(msg)
}

// GpioRequestSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type GpioRequestSubscription struct {
	*rclgo.Subscription
}

// GpioRequestSubscriptionCallback type is used to provide a subscription
// handler function for a GpioRequestSubscription.
type GpioRequestSubscriptionCallback func(msg *GpioRequest, info *rclgo.MessageInfo, err error)

// NewGpioRequestSubscription creates and returns a new subscription for the
// GpioRequest
func NewGpioRequestSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback GpioRequestSubscriptionCallback) (*GpioRequestSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg GpioRequest
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, GpioRequestTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &GpioRequestSubscription{sub}, nil
}

func (s *GpioRequestSubscription) TakeMessage(out *GpioRequest) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneGpioRequestSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneGpioRequestSlice(dst, src []GpioRequest) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var GpioRequestTypeSupport types.MessageTypeSupport = _GpioRequestTypeSupport{}

type _GpioRequestTypeSupport struct{}

func (t _GpioRequestTypeSupport) New() types.Message {
	return NewGpioRequest()
}

func (t _GpioRequestTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__GpioRequest
	return (unsafe.Pointer)(C.px4_msgs__msg__GpioRequest__create())
}

func (t _GpioRequestTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__GpioRequest__destroy((*C.px4_msgs__msg__GpioRequest)(pointer_to_free))
}

func (t _GpioRequestTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*GpioRequest)
	mem := (*C.px4_msgs__msg__GpioRequest)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.device_id = C.uint32_t(m.DeviceId)
}

func (t _GpioRequestTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*GpioRequest)
	mem := (*C.px4_msgs__msg__GpioRequest)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.DeviceId = uint32(mem.device_id)
}

func (t _GpioRequestTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__GpioRequest())
}

type CGpioRequest = C.px4_msgs__msg__GpioRequest
type CGpioRequest__Sequence = C.px4_msgs__msg__GpioRequest__Sequence

func GpioRequest__Sequence_to_Go(goSlice *[]GpioRequest, cSlice CGpioRequest__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]GpioRequest, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		GpioRequestTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func GpioRequest__Sequence_to_C(cSlice *CGpioRequest__Sequence, goSlice []GpioRequest) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__GpioRequest)(C.malloc(C.sizeof_struct_px4_msgs__msg__GpioRequest * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		GpioRequestTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func GpioRequest__Array_to_Go(goSlice []GpioRequest, cSlice []CGpioRequest) {
	for i := 0; i < len(cSlice); i++ {
		GpioRequestTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func GpioRequest__Array_to_C(cSlice []CGpioRequest, goSlice []GpioRequest) {
	for i := 0; i < len(goSlice); i++ {
		GpioRequestTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
