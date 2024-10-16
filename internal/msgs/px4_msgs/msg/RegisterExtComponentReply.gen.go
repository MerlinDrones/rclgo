// Code generated by rclgo-gen. DO NOT EDIT.

package px4_msgs_msg

import (
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	primitives "github.com/merlindrones/rclgo/pkg/rclgo/primitives"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

/*
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <px4_msgs/msg/register_ext_component_reply.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/RegisterExtComponentReply", RegisterExtComponentReplyTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/RegisterExtComponentReply", RegisterExtComponentReplyTypeSupport)
}

const (
	RegisterExtComponentReply_ORB_QUEUE_LENGTH uint8 = 2
)

type RegisterExtComponentReply struct {
	Timestamp         uint64   `yaml:"timestamp"`  // time since system start (microseconds)
	RequestId         uint64   `yaml:"request_id"` // ID from the request
	Name              [25]byte `yaml:"name"`       // name from the request
	Px4Ros2ApiVersion uint16   `yaml:"px4_ros2_api_version"`
	Success           bool     `yaml:"success"`
	ArmingCheckId     int8     `yaml:"arming_check_id"`  // arming check registration ID (-1 if invalid)
	ModeId            int8     `yaml:"mode_id"`          // assigned mode ID (-1 if invalid)
	ModeExecutorId    int8     `yaml:"mode_executor_id"` // assigned mode executor ID (-1 if invalid)
}

// NewRegisterExtComponentReply creates a new RegisterExtComponentReply with default values.
func NewRegisterExtComponentReply() *RegisterExtComponentReply {
	self := RegisterExtComponentReply{}
	self.SetDefaults()
	return &self
}

func (t *RegisterExtComponentReply) Clone() *RegisterExtComponentReply {
	c := &RegisterExtComponentReply{}
	c.Timestamp = t.Timestamp
	c.RequestId = t.RequestId
	c.Name = t.Name
	c.Px4Ros2ApiVersion = t.Px4Ros2ApiVersion
	c.Success = t.Success
	c.ArmingCheckId = t.ArmingCheckId
	c.ModeId = t.ModeId
	c.ModeExecutorId = t.ModeExecutorId
	return c
}

func (t *RegisterExtComponentReply) CloneMsg() types.Message {
	return t.Clone()
}

func (t *RegisterExtComponentReply) SetDefaults() {
	t.Timestamp = 0
	t.RequestId = 0
	t.Name = [25]byte{}
	t.Px4Ros2ApiVersion = 0
	t.Success = false
	t.ArmingCheckId = 0
	t.ModeId = 0
	t.ModeExecutorId = 0
}

func (t *RegisterExtComponentReply) GetTypeSupport() types.MessageTypeSupport {
	return RegisterExtComponentReplyTypeSupport
}

// RegisterExtComponentReplyPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type RegisterExtComponentReplyPublisher struct {
	*rclgo.Publisher
}

// NewRegisterExtComponentReplyPublisher creates and returns a new publisher for the
// RegisterExtComponentReply
func NewRegisterExtComponentReplyPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*RegisterExtComponentReplyPublisher, error) {
	pub, err := node.NewPublisher(topic_name, RegisterExtComponentReplyTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &RegisterExtComponentReplyPublisher{pub}, nil
}

func (p *RegisterExtComponentReplyPublisher) Publish(msg *RegisterExtComponentReply) error {
	return p.Publisher.Publish(msg)
}

// RegisterExtComponentReplySubscription wraps rclgo.Subscription to provide type safe helper
// functions
type RegisterExtComponentReplySubscription struct {
	*rclgo.Subscription
}

// RegisterExtComponentReplySubscriptionCallback type is used to provide a subscription
// handler function for a RegisterExtComponentReplySubscription.
type RegisterExtComponentReplySubscriptionCallback func(msg *RegisterExtComponentReply, info *rclgo.MessageInfo, err error)

// NewRegisterExtComponentReplySubscription creates and returns a new subscription for the
// RegisterExtComponentReply
func NewRegisterExtComponentReplySubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback RegisterExtComponentReplySubscriptionCallback) (*RegisterExtComponentReplySubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg RegisterExtComponentReply
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, RegisterExtComponentReplyTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &RegisterExtComponentReplySubscription{sub}, nil
}

func (s *RegisterExtComponentReplySubscription) TakeMessage(out *RegisterExtComponentReply) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneRegisterExtComponentReplySlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneRegisterExtComponentReplySlice(dst, src []RegisterExtComponentReply) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var RegisterExtComponentReplyTypeSupport types.MessageTypeSupport = _RegisterExtComponentReplyTypeSupport{}

type _RegisterExtComponentReplyTypeSupport struct{}

func (t _RegisterExtComponentReplyTypeSupport) New() types.Message {
	return NewRegisterExtComponentReply()
}

func (t _RegisterExtComponentReplyTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__RegisterExtComponentReply
	return (unsafe.Pointer)(C.px4_msgs__msg__RegisterExtComponentReply__create())
}

func (t _RegisterExtComponentReplyTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__RegisterExtComponentReply__destroy((*C.px4_msgs__msg__RegisterExtComponentReply)(pointer_to_free))
}

func (t _RegisterExtComponentReplyTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*RegisterExtComponentReply)
	mem := (*C.px4_msgs__msg__RegisterExtComponentReply)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.request_id = C.uint64_t(m.RequestId)
	cSlice_name := mem.name[:]
	primitives.Char__Array_to_C(*(*[]primitives.CChar)(unsafe.Pointer(&cSlice_name)), m.Name[:])
	mem.px4_ros2_api_version = C.uint16_t(m.Px4Ros2ApiVersion)
	mem.success = C.bool(m.Success)
	mem.arming_check_id = C.int8_t(m.ArmingCheckId)
	mem.mode_id = C.int8_t(m.ModeId)
	mem.mode_executor_id = C.int8_t(m.ModeExecutorId)
}

func (t _RegisterExtComponentReplyTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*RegisterExtComponentReply)
	mem := (*C.px4_msgs__msg__RegisterExtComponentReply)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.RequestId = uint64(mem.request_id)
	cSlice_name := mem.name[:]
	primitives.Char__Array_to_Go(m.Name[:], *(*[]primitives.CChar)(unsafe.Pointer(&cSlice_name)))
	m.Px4Ros2ApiVersion = uint16(mem.px4_ros2_api_version)
	m.Success = bool(mem.success)
	m.ArmingCheckId = int8(mem.arming_check_id)
	m.ModeId = int8(mem.mode_id)
	m.ModeExecutorId = int8(mem.mode_executor_id)
}

func (t _RegisterExtComponentReplyTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__RegisterExtComponentReply())
}

type CRegisterExtComponentReply = C.px4_msgs__msg__RegisterExtComponentReply
type CRegisterExtComponentReply__Sequence = C.px4_msgs__msg__RegisterExtComponentReply__Sequence

func RegisterExtComponentReply__Sequence_to_Go(goSlice *[]RegisterExtComponentReply, cSlice CRegisterExtComponentReply__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]RegisterExtComponentReply, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		RegisterExtComponentReplyTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func RegisterExtComponentReply__Sequence_to_C(cSlice *CRegisterExtComponentReply__Sequence, goSlice []RegisterExtComponentReply) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__RegisterExtComponentReply)(C.malloc(C.sizeof_struct_px4_msgs__msg__RegisterExtComponentReply * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		RegisterExtComponentReplyTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func RegisterExtComponentReply__Array_to_Go(goSlice []RegisterExtComponentReply, cSlice []CRegisterExtComponentReply) {
	for i := 0; i < len(cSlice); i++ {
		RegisterExtComponentReplyTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func RegisterExtComponentReply__Array_to_C(cSlice []CRegisterExtComponentReply, goSlice []RegisterExtComponentReply) {
	for i := 0; i < len(goSlice); i++ {
		RegisterExtComponentReplyTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
