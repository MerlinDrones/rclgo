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

#include <px4_msgs/msg/orb_test_medium.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/OrbTestMedium", OrbTestMediumTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/OrbTestMedium", OrbTestMediumTypeSupport)
}

const (
	OrbTestMedium_ORB_QUEUE_LENGTH uint8 = 16
)

type OrbTestMedium struct {
	Timestamp uint64    `yaml:"timestamp"` // time since system start (microseconds)
	Val       int32     `yaml:"val"`
	Junk      [64]uint8 `yaml:"junk"`
}

// NewOrbTestMedium creates a new OrbTestMedium with default values.
func NewOrbTestMedium() *OrbTestMedium {
	self := OrbTestMedium{}
	self.SetDefaults()
	return &self
}

func (t *OrbTestMedium) Clone() *OrbTestMedium {
	c := &OrbTestMedium{}
	c.Timestamp = t.Timestamp
	c.Val = t.Val
	c.Junk = t.Junk
	return c
}

func (t *OrbTestMedium) CloneMsg() types.Message {
	return t.Clone()
}

func (t *OrbTestMedium) SetDefaults() {
	t.Timestamp = 0
	t.Val = 0
	t.Junk = [64]uint8{}
}

func (t *OrbTestMedium) GetTypeSupport() types.MessageTypeSupport {
	return OrbTestMediumTypeSupport
}

// OrbTestMediumPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type OrbTestMediumPublisher struct {
	*rclgo.Publisher
}

// NewOrbTestMediumPublisher creates and returns a new publisher for the
// OrbTestMedium
func NewOrbTestMediumPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*OrbTestMediumPublisher, error) {
	pub, err := node.NewPublisher(topic_name, OrbTestMediumTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &OrbTestMediumPublisher{pub}, nil
}

func (p *OrbTestMediumPublisher) Publish(msg *OrbTestMedium) error {
	return p.Publisher.Publish(msg)
}

// OrbTestMediumSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type OrbTestMediumSubscription struct {
	*rclgo.Subscription
}

// OrbTestMediumSubscriptionCallback type is used to provide a subscription
// handler function for a OrbTestMediumSubscription.
type OrbTestMediumSubscriptionCallback func(msg *OrbTestMedium, info *rclgo.MessageInfo, err error)

// NewOrbTestMediumSubscription creates and returns a new subscription for the
// OrbTestMedium
func NewOrbTestMediumSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback OrbTestMediumSubscriptionCallback) (*OrbTestMediumSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg OrbTestMedium
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, OrbTestMediumTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &OrbTestMediumSubscription{sub}, nil
}

func (s *OrbTestMediumSubscription) TakeMessage(out *OrbTestMedium) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneOrbTestMediumSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneOrbTestMediumSlice(dst, src []OrbTestMedium) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var OrbTestMediumTypeSupport types.MessageTypeSupport = _OrbTestMediumTypeSupport{}

type _OrbTestMediumTypeSupport struct{}

func (t _OrbTestMediumTypeSupport) New() types.Message {
	return NewOrbTestMedium()
}

func (t _OrbTestMediumTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__OrbTestMedium
	return (unsafe.Pointer)(C.px4_msgs__msg__OrbTestMedium__create())
}

func (t _OrbTestMediumTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__OrbTestMedium__destroy((*C.px4_msgs__msg__OrbTestMedium)(pointer_to_free))
}

func (t _OrbTestMediumTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*OrbTestMedium)
	mem := (*C.px4_msgs__msg__OrbTestMedium)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.val = C.int32_t(m.Val)
	cSlice_junk := mem.junk[:]
	primitives.Uint8__Array_to_C(*(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_junk)), m.Junk[:])
}

func (t _OrbTestMediumTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*OrbTestMedium)
	mem := (*C.px4_msgs__msg__OrbTestMedium)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.Val = int32(mem.val)
	cSlice_junk := mem.junk[:]
	primitives.Uint8__Array_to_Go(m.Junk[:], *(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_junk)))
}

func (t _OrbTestMediumTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__OrbTestMedium())
}

type COrbTestMedium = C.px4_msgs__msg__OrbTestMedium
type COrbTestMedium__Sequence = C.px4_msgs__msg__OrbTestMedium__Sequence

func OrbTestMedium__Sequence_to_Go(goSlice *[]OrbTestMedium, cSlice COrbTestMedium__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]OrbTestMedium, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		OrbTestMediumTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func OrbTestMedium__Sequence_to_C(cSlice *COrbTestMedium__Sequence, goSlice []OrbTestMedium) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__OrbTestMedium)(C.malloc(C.sizeof_struct_px4_msgs__msg__OrbTestMedium * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		OrbTestMediumTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func OrbTestMedium__Array_to_Go(goSlice []OrbTestMedium, cSlice []COrbTestMedium) {
	for i := 0; i < len(cSlice); i++ {
		OrbTestMediumTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func OrbTestMedium__Array_to_C(cSlice []COrbTestMedium, goSlice []OrbTestMedium) {
	for i := 0; i < len(goSlice); i++ {
		OrbTestMediumTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
