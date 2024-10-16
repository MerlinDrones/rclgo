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

#include <px4_msgs/msg/orb_test_large.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/OrbTestLarge", OrbTestLargeTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/OrbTestLarge", OrbTestLargeTypeSupport)
}

type OrbTestLarge struct {
	Timestamp uint64     `yaml:"timestamp"` // time since system start (microseconds)
	Val       int32      `yaml:"val"`
	Junk      [512]uint8 `yaml:"junk"`
}

// NewOrbTestLarge creates a new OrbTestLarge with default values.
func NewOrbTestLarge() *OrbTestLarge {
	self := OrbTestLarge{}
	self.SetDefaults()
	return &self
}

func (t *OrbTestLarge) Clone() *OrbTestLarge {
	c := &OrbTestLarge{}
	c.Timestamp = t.Timestamp
	c.Val = t.Val
	c.Junk = t.Junk
	return c
}

func (t *OrbTestLarge) CloneMsg() types.Message {
	return t.Clone()
}

func (t *OrbTestLarge) SetDefaults() {
	t.Timestamp = 0
	t.Val = 0
	t.Junk = [512]uint8{}
}

func (t *OrbTestLarge) GetTypeSupport() types.MessageTypeSupport {
	return OrbTestLargeTypeSupport
}

// OrbTestLargePublisher wraps rclgo.Publisher to provide type safe helper
// functions
type OrbTestLargePublisher struct {
	*rclgo.Publisher
}

// NewOrbTestLargePublisher creates and returns a new publisher for the
// OrbTestLarge
func NewOrbTestLargePublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*OrbTestLargePublisher, error) {
	pub, err := node.NewPublisher(topic_name, OrbTestLargeTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &OrbTestLargePublisher{pub}, nil
}

func (p *OrbTestLargePublisher) Publish(msg *OrbTestLarge) error {
	return p.Publisher.Publish(msg)
}

// OrbTestLargeSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type OrbTestLargeSubscription struct {
	*rclgo.Subscription
}

// OrbTestLargeSubscriptionCallback type is used to provide a subscription
// handler function for a OrbTestLargeSubscription.
type OrbTestLargeSubscriptionCallback func(msg *OrbTestLarge, info *rclgo.MessageInfo, err error)

// NewOrbTestLargeSubscription creates and returns a new subscription for the
// OrbTestLarge
func NewOrbTestLargeSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback OrbTestLargeSubscriptionCallback) (*OrbTestLargeSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg OrbTestLarge
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, OrbTestLargeTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &OrbTestLargeSubscription{sub}, nil
}

func (s *OrbTestLargeSubscription) TakeMessage(out *OrbTestLarge) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneOrbTestLargeSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneOrbTestLargeSlice(dst, src []OrbTestLarge) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var OrbTestLargeTypeSupport types.MessageTypeSupport = _OrbTestLargeTypeSupport{}

type _OrbTestLargeTypeSupport struct{}

func (t _OrbTestLargeTypeSupport) New() types.Message {
	return NewOrbTestLarge()
}

func (t _OrbTestLargeTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__OrbTestLarge
	return (unsafe.Pointer)(C.px4_msgs__msg__OrbTestLarge__create())
}

func (t _OrbTestLargeTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__OrbTestLarge__destroy((*C.px4_msgs__msg__OrbTestLarge)(pointer_to_free))
}

func (t _OrbTestLargeTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*OrbTestLarge)
	mem := (*C.px4_msgs__msg__OrbTestLarge)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.val = C.int32_t(m.Val)
	cSlice_junk := mem.junk[:]
	primitives.Uint8__Array_to_C(*(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_junk)), m.Junk[:])
}

func (t _OrbTestLargeTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*OrbTestLarge)
	mem := (*C.px4_msgs__msg__OrbTestLarge)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.Val = int32(mem.val)
	cSlice_junk := mem.junk[:]
	primitives.Uint8__Array_to_Go(m.Junk[:], *(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_junk)))
}

func (t _OrbTestLargeTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__OrbTestLarge())
}

type COrbTestLarge = C.px4_msgs__msg__OrbTestLarge
type COrbTestLarge__Sequence = C.px4_msgs__msg__OrbTestLarge__Sequence

func OrbTestLarge__Sequence_to_Go(goSlice *[]OrbTestLarge, cSlice COrbTestLarge__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]OrbTestLarge, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		OrbTestLargeTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func OrbTestLarge__Sequence_to_C(cSlice *COrbTestLarge__Sequence, goSlice []OrbTestLarge) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__OrbTestLarge)(C.malloc(C.sizeof_struct_px4_msgs__msg__OrbTestLarge * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		OrbTestLargeTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func OrbTestLarge__Array_to_Go(goSlice []OrbTestLarge, cSlice []COrbTestLarge) {
	for i := 0; i < len(cSlice); i++ {
		OrbTestLargeTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func OrbTestLarge__Array_to_C(cSlice []COrbTestLarge, goSlice []OrbTestLarge) {
	for i := 0; i < len(goSlice); i++ {
		OrbTestLargeTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
