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

#include <px4_msgs/msg/landing_target_innovations.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/LandingTargetInnovations", LandingTargetInnovationsTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/LandingTargetInnovations", LandingTargetInnovationsTypeSupport)
}

type LandingTargetInnovations struct {
	Timestamp uint64  `yaml:"timestamp"` // time since system start (microseconds)
	InnovX    float32 `yaml:"innov_x"`   // Innovation of landing target position estimator
	InnovY    float32 `yaml:"innov_y"`
	InnovCovX float32 `yaml:"innov_cov_x"` // Innovation covariance of landing target position estimator
	InnovCovY float32 `yaml:"innov_cov_y"`
}

// NewLandingTargetInnovations creates a new LandingTargetInnovations with default values.
func NewLandingTargetInnovations() *LandingTargetInnovations {
	self := LandingTargetInnovations{}
	self.SetDefaults()
	return &self
}

func (t *LandingTargetInnovations) Clone() *LandingTargetInnovations {
	c := &LandingTargetInnovations{}
	c.Timestamp = t.Timestamp
	c.InnovX = t.InnovX
	c.InnovY = t.InnovY
	c.InnovCovX = t.InnovCovX
	c.InnovCovY = t.InnovCovY
	return c
}

func (t *LandingTargetInnovations) CloneMsg() types.Message {
	return t.Clone()
}

func (t *LandingTargetInnovations) SetDefaults() {
	t.Timestamp = 0
	t.InnovX = 0
	t.InnovY = 0
	t.InnovCovX = 0
	t.InnovCovY = 0
}

func (t *LandingTargetInnovations) GetTypeSupport() types.MessageTypeSupport {
	return LandingTargetInnovationsTypeSupport
}

// LandingTargetInnovationsPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type LandingTargetInnovationsPublisher struct {
	*rclgo.Publisher
}

// NewLandingTargetInnovationsPublisher creates and returns a new publisher for the
// LandingTargetInnovations
func NewLandingTargetInnovationsPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*LandingTargetInnovationsPublisher, error) {
	pub, err := node.NewPublisher(topic_name, LandingTargetInnovationsTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &LandingTargetInnovationsPublisher{pub}, nil
}

func (p *LandingTargetInnovationsPublisher) Publish(msg *LandingTargetInnovations) error {
	return p.Publisher.Publish(msg)
}

// LandingTargetInnovationsSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type LandingTargetInnovationsSubscription struct {
	*rclgo.Subscription
}

// LandingTargetInnovationsSubscriptionCallback type is used to provide a subscription
// handler function for a LandingTargetInnovationsSubscription.
type LandingTargetInnovationsSubscriptionCallback func(msg *LandingTargetInnovations, info *rclgo.MessageInfo, err error)

// NewLandingTargetInnovationsSubscription creates and returns a new subscription for the
// LandingTargetInnovations
func NewLandingTargetInnovationsSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback LandingTargetInnovationsSubscriptionCallback) (*LandingTargetInnovationsSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg LandingTargetInnovations
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, LandingTargetInnovationsTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &LandingTargetInnovationsSubscription{sub}, nil
}

func (s *LandingTargetInnovationsSubscription) TakeMessage(out *LandingTargetInnovations) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneLandingTargetInnovationsSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneLandingTargetInnovationsSlice(dst, src []LandingTargetInnovations) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var LandingTargetInnovationsTypeSupport types.MessageTypeSupport = _LandingTargetInnovationsTypeSupport{}

type _LandingTargetInnovationsTypeSupport struct{}

func (t _LandingTargetInnovationsTypeSupport) New() types.Message {
	return NewLandingTargetInnovations()
}

func (t _LandingTargetInnovationsTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__LandingTargetInnovations
	return (unsafe.Pointer)(C.px4_msgs__msg__LandingTargetInnovations__create())
}

func (t _LandingTargetInnovationsTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__LandingTargetInnovations__destroy((*C.px4_msgs__msg__LandingTargetInnovations)(pointer_to_free))
}

func (t _LandingTargetInnovationsTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*LandingTargetInnovations)
	mem := (*C.px4_msgs__msg__LandingTargetInnovations)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.innov_x = C.float(m.InnovX)
	mem.innov_y = C.float(m.InnovY)
	mem.innov_cov_x = C.float(m.InnovCovX)
	mem.innov_cov_y = C.float(m.InnovCovY)
}

func (t _LandingTargetInnovationsTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*LandingTargetInnovations)
	mem := (*C.px4_msgs__msg__LandingTargetInnovations)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.InnovX = float32(mem.innov_x)
	m.InnovY = float32(mem.innov_y)
	m.InnovCovX = float32(mem.innov_cov_x)
	m.InnovCovY = float32(mem.innov_cov_y)
}

func (t _LandingTargetInnovationsTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__LandingTargetInnovations())
}

type CLandingTargetInnovations = C.px4_msgs__msg__LandingTargetInnovations
type CLandingTargetInnovations__Sequence = C.px4_msgs__msg__LandingTargetInnovations__Sequence

func LandingTargetInnovations__Sequence_to_Go(goSlice *[]LandingTargetInnovations, cSlice CLandingTargetInnovations__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]LandingTargetInnovations, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		LandingTargetInnovationsTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func LandingTargetInnovations__Sequence_to_C(cSlice *CLandingTargetInnovations__Sequence, goSlice []LandingTargetInnovations) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__LandingTargetInnovations)(C.malloc(C.sizeof_struct_px4_msgs__msg__LandingTargetInnovations * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		LandingTargetInnovationsTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func LandingTargetInnovations__Array_to_Go(goSlice []LandingTargetInnovations, cSlice []CLandingTargetInnovations) {
	for i := 0; i < len(cSlice); i++ {
		LandingTargetInnovationsTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func LandingTargetInnovations__Array_to_C(cSlice []CLandingTargetInnovations, goSlice []LandingTargetInnovations) {
	for i := 0; i < len(goSlice); i++ {
		LandingTargetInnovationsTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
