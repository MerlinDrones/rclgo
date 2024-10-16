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

#include <px4_msgs/msg/rover_ackermann_guidance_status.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/RoverAckermannGuidanceStatus", RoverAckermannGuidanceStatusTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/RoverAckermannGuidanceStatus", RoverAckermannGuidanceStatusTypeSupport)
}

type RoverAckermannGuidanceStatus struct {
	Timestamp           uint64  `yaml:"timestamp"`             // time since system start (microseconds)
	DesiredSpeed        float32 `yaml:"desired_speed"`         // [m/s] Rover desired ground speed
	LookaheadDistance   float32 `yaml:"lookahead_distance"`    // [m] Lookahead distance of pure the pursuit controller
	HeadingError        float32 `yaml:"heading_error"`         // [deg] Heading error of the pure pursuit controller
	PidThrottleIntegral float32 `yaml:"pid_throttle_integral"` // [-1, 1] Integral of the PID for the normalized throttle to control the rover speed during missions
}

// NewRoverAckermannGuidanceStatus creates a new RoverAckermannGuidanceStatus with default values.
func NewRoverAckermannGuidanceStatus() *RoverAckermannGuidanceStatus {
	self := RoverAckermannGuidanceStatus{}
	self.SetDefaults()
	return &self
}

func (t *RoverAckermannGuidanceStatus) Clone() *RoverAckermannGuidanceStatus {
	c := &RoverAckermannGuidanceStatus{}
	c.Timestamp = t.Timestamp
	c.DesiredSpeed = t.DesiredSpeed
	c.LookaheadDistance = t.LookaheadDistance
	c.HeadingError = t.HeadingError
	c.PidThrottleIntegral = t.PidThrottleIntegral
	return c
}

func (t *RoverAckermannGuidanceStatus) CloneMsg() types.Message {
	return t.Clone()
}

func (t *RoverAckermannGuidanceStatus) SetDefaults() {
	t.Timestamp = 0
	t.DesiredSpeed = 0
	t.LookaheadDistance = 0
	t.HeadingError = 0
	t.PidThrottleIntegral = 0
}

func (t *RoverAckermannGuidanceStatus) GetTypeSupport() types.MessageTypeSupport {
	return RoverAckermannGuidanceStatusTypeSupport
}

// RoverAckermannGuidanceStatusPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type RoverAckermannGuidanceStatusPublisher struct {
	*rclgo.Publisher
}

// NewRoverAckermannGuidanceStatusPublisher creates and returns a new publisher for the
// RoverAckermannGuidanceStatus
func NewRoverAckermannGuidanceStatusPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*RoverAckermannGuidanceStatusPublisher, error) {
	pub, err := node.NewPublisher(topic_name, RoverAckermannGuidanceStatusTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &RoverAckermannGuidanceStatusPublisher{pub}, nil
}

func (p *RoverAckermannGuidanceStatusPublisher) Publish(msg *RoverAckermannGuidanceStatus) error {
	return p.Publisher.Publish(msg)
}

// RoverAckermannGuidanceStatusSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type RoverAckermannGuidanceStatusSubscription struct {
	*rclgo.Subscription
}

// RoverAckermannGuidanceStatusSubscriptionCallback type is used to provide a subscription
// handler function for a RoverAckermannGuidanceStatusSubscription.
type RoverAckermannGuidanceStatusSubscriptionCallback func(msg *RoverAckermannGuidanceStatus, info *rclgo.MessageInfo, err error)

// NewRoverAckermannGuidanceStatusSubscription creates and returns a new subscription for the
// RoverAckermannGuidanceStatus
func NewRoverAckermannGuidanceStatusSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback RoverAckermannGuidanceStatusSubscriptionCallback) (*RoverAckermannGuidanceStatusSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg RoverAckermannGuidanceStatus
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, RoverAckermannGuidanceStatusTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &RoverAckermannGuidanceStatusSubscription{sub}, nil
}

func (s *RoverAckermannGuidanceStatusSubscription) TakeMessage(out *RoverAckermannGuidanceStatus) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneRoverAckermannGuidanceStatusSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneRoverAckermannGuidanceStatusSlice(dst, src []RoverAckermannGuidanceStatus) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var RoverAckermannGuidanceStatusTypeSupport types.MessageTypeSupport = _RoverAckermannGuidanceStatusTypeSupport{}

type _RoverAckermannGuidanceStatusTypeSupport struct{}

func (t _RoverAckermannGuidanceStatusTypeSupport) New() types.Message {
	return NewRoverAckermannGuidanceStatus()
}

func (t _RoverAckermannGuidanceStatusTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__RoverAckermannGuidanceStatus
	return (unsafe.Pointer)(C.px4_msgs__msg__RoverAckermannGuidanceStatus__create())
}

func (t _RoverAckermannGuidanceStatusTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__RoverAckermannGuidanceStatus__destroy((*C.px4_msgs__msg__RoverAckermannGuidanceStatus)(pointer_to_free))
}

func (t _RoverAckermannGuidanceStatusTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*RoverAckermannGuidanceStatus)
	mem := (*C.px4_msgs__msg__RoverAckermannGuidanceStatus)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.desired_speed = C.float(m.DesiredSpeed)
	mem.lookahead_distance = C.float(m.LookaheadDistance)
	mem.heading_error = C.float(m.HeadingError)
	mem.pid_throttle_integral = C.float(m.PidThrottleIntegral)
}

func (t _RoverAckermannGuidanceStatusTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*RoverAckermannGuidanceStatus)
	mem := (*C.px4_msgs__msg__RoverAckermannGuidanceStatus)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.DesiredSpeed = float32(mem.desired_speed)
	m.LookaheadDistance = float32(mem.lookahead_distance)
	m.HeadingError = float32(mem.heading_error)
	m.PidThrottleIntegral = float32(mem.pid_throttle_integral)
}

func (t _RoverAckermannGuidanceStatusTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__RoverAckermannGuidanceStatus())
}

type CRoverAckermannGuidanceStatus = C.px4_msgs__msg__RoverAckermannGuidanceStatus
type CRoverAckermannGuidanceStatus__Sequence = C.px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence

func RoverAckermannGuidanceStatus__Sequence_to_Go(goSlice *[]RoverAckermannGuidanceStatus, cSlice CRoverAckermannGuidanceStatus__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]RoverAckermannGuidanceStatus, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		RoverAckermannGuidanceStatusTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func RoverAckermannGuidanceStatus__Sequence_to_C(cSlice *CRoverAckermannGuidanceStatus__Sequence, goSlice []RoverAckermannGuidanceStatus) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__RoverAckermannGuidanceStatus)(C.malloc(C.sizeof_struct_px4_msgs__msg__RoverAckermannGuidanceStatus * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		RoverAckermannGuidanceStatusTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func RoverAckermannGuidanceStatus__Array_to_Go(goSlice []RoverAckermannGuidanceStatus, cSlice []CRoverAckermannGuidanceStatus) {
	for i := 0; i < len(cSlice); i++ {
		RoverAckermannGuidanceStatusTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func RoverAckermannGuidanceStatus__Array_to_C(cSlice []CRoverAckermannGuidanceStatus, goSlice []RoverAckermannGuidanceStatus) {
	for i := 0; i < len(goSlice); i++ {
		RoverAckermannGuidanceStatusTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
