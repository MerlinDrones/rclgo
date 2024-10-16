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

#include <px4_msgs/msg/orb_test.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/OrbTest", OrbTestTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/OrbTest", OrbTestTypeSupport)
}

type OrbTest struct {
	Timestamp uint64 `yaml:"timestamp"` // time since system start (microseconds)
	Val       int32  `yaml:"val"`
}

// NewOrbTest creates a new OrbTest with default values.
func NewOrbTest() *OrbTest {
	self := OrbTest{}
	self.SetDefaults()
	return &self
}

func (t *OrbTest) Clone() *OrbTest {
	c := &OrbTest{}
	c.Timestamp = t.Timestamp
	c.Val = t.Val
	return c
}

func (t *OrbTest) CloneMsg() types.Message {
	return t.Clone()
}

func (t *OrbTest) SetDefaults() {
	t.Timestamp = 0
	t.Val = 0
}

func (t *OrbTest) GetTypeSupport() types.MessageTypeSupport {
	return OrbTestTypeSupport
}

// OrbTestPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type OrbTestPublisher struct {
	*rclgo.Publisher
}

// NewOrbTestPublisher creates and returns a new publisher for the
// OrbTest
func NewOrbTestPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*OrbTestPublisher, error) {
	pub, err := node.NewPublisher(topic_name, OrbTestTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &OrbTestPublisher{pub}, nil
}

func (p *OrbTestPublisher) Publish(msg *OrbTest) error {
	return p.Publisher.Publish(msg)
}

// OrbTestSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type OrbTestSubscription struct {
	*rclgo.Subscription
}

// OrbTestSubscriptionCallback type is used to provide a subscription
// handler function for a OrbTestSubscription.
type OrbTestSubscriptionCallback func(msg *OrbTest, info *rclgo.MessageInfo, err error)

// NewOrbTestSubscription creates and returns a new subscription for the
// OrbTest
func NewOrbTestSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback OrbTestSubscriptionCallback) (*OrbTestSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg OrbTest
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, OrbTestTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &OrbTestSubscription{sub}, nil
}

func (s *OrbTestSubscription) TakeMessage(out *OrbTest) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneOrbTestSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneOrbTestSlice(dst, src []OrbTest) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var OrbTestTypeSupport types.MessageTypeSupport = _OrbTestTypeSupport{}

type _OrbTestTypeSupport struct{}

func (t _OrbTestTypeSupport) New() types.Message {
	return NewOrbTest()
}

func (t _OrbTestTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__OrbTest
	return (unsafe.Pointer)(C.px4_msgs__msg__OrbTest__create())
}

func (t _OrbTestTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__OrbTest__destroy((*C.px4_msgs__msg__OrbTest)(pointer_to_free))
}

func (t _OrbTestTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*OrbTest)
	mem := (*C.px4_msgs__msg__OrbTest)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.val = C.int32_t(m.Val)
}

func (t _OrbTestTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*OrbTest)
	mem := (*C.px4_msgs__msg__OrbTest)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.Val = int32(mem.val)
}

func (t _OrbTestTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__OrbTest())
}

type COrbTest = C.px4_msgs__msg__OrbTest
type COrbTest__Sequence = C.px4_msgs__msg__OrbTest__Sequence

func OrbTest__Sequence_to_Go(goSlice *[]OrbTest, cSlice COrbTest__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]OrbTest, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		OrbTestTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func OrbTest__Sequence_to_C(cSlice *COrbTest__Sequence, goSlice []OrbTest) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__OrbTest)(C.malloc(C.sizeof_struct_px4_msgs__msg__OrbTest * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		OrbTestTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func OrbTest__Array_to_Go(goSlice []OrbTest, cSlice []COrbTest) {
	for i := 0; i < len(cSlice); i++ {
		OrbTestTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func OrbTest__Array_to_C(cSlice []COrbTest, goSlice []OrbTest) {
	for i := 0; i < len(goSlice); i++ {
		OrbTestTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
