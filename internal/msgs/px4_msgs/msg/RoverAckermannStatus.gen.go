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

#include <px4_msgs/msg/rover_ackermann_status.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/RoverAckermannStatus", RoverAckermannStatusTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/RoverAckermannStatus", RoverAckermannStatusTypeSupport)
}

type RoverAckermannStatus struct {
	Timestamp        uint64  `yaml:"timestamp"`         // time since system start (microseconds)
	ThrottleSetpoint float32 `yaml:"throttle_setpoint"` // [-1, 1] Normalized throttle setpoint
	SteeringSetpoint float32 `yaml:"steering_setpoint"` // [-1, 1] Normalized steering setpoint
	ActualSpeed      float32 `yaml:"actual_speed"`      // [m/s] Rover ground speed
}

// NewRoverAckermannStatus creates a new RoverAckermannStatus with default values.
func NewRoverAckermannStatus() *RoverAckermannStatus {
	self := RoverAckermannStatus{}
	self.SetDefaults()
	return &self
}

func (t *RoverAckermannStatus) Clone() *RoverAckermannStatus {
	c := &RoverAckermannStatus{}
	c.Timestamp = t.Timestamp
	c.ThrottleSetpoint = t.ThrottleSetpoint
	c.SteeringSetpoint = t.SteeringSetpoint
	c.ActualSpeed = t.ActualSpeed
	return c
}

func (t *RoverAckermannStatus) CloneMsg() types.Message {
	return t.Clone()
}

func (t *RoverAckermannStatus) SetDefaults() {
	t.Timestamp = 0
	t.ThrottleSetpoint = 0
	t.SteeringSetpoint = 0
	t.ActualSpeed = 0
}

func (t *RoverAckermannStatus) GetTypeSupport() types.MessageTypeSupport {
	return RoverAckermannStatusTypeSupport
}

// RoverAckermannStatusPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type RoverAckermannStatusPublisher struct {
	*rclgo.Publisher
}

// NewRoverAckermannStatusPublisher creates and returns a new publisher for the
// RoverAckermannStatus
func NewRoverAckermannStatusPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*RoverAckermannStatusPublisher, error) {
	pub, err := node.NewPublisher(topic_name, RoverAckermannStatusTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &RoverAckermannStatusPublisher{pub}, nil
}

func (p *RoverAckermannStatusPublisher) Publish(msg *RoverAckermannStatus) error {
	return p.Publisher.Publish(msg)
}

// RoverAckermannStatusSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type RoverAckermannStatusSubscription struct {
	*rclgo.Subscription
}

// RoverAckermannStatusSubscriptionCallback type is used to provide a subscription
// handler function for a RoverAckermannStatusSubscription.
type RoverAckermannStatusSubscriptionCallback func(msg *RoverAckermannStatus, info *rclgo.MessageInfo, err error)

// NewRoverAckermannStatusSubscription creates and returns a new subscription for the
// RoverAckermannStatus
func NewRoverAckermannStatusSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback RoverAckermannStatusSubscriptionCallback) (*RoverAckermannStatusSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg RoverAckermannStatus
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, RoverAckermannStatusTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &RoverAckermannStatusSubscription{sub}, nil
}

func (s *RoverAckermannStatusSubscription) TakeMessage(out *RoverAckermannStatus) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneRoverAckermannStatusSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneRoverAckermannStatusSlice(dst, src []RoverAckermannStatus) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var RoverAckermannStatusTypeSupport types.MessageTypeSupport = _RoverAckermannStatusTypeSupport{}

type _RoverAckermannStatusTypeSupport struct{}

func (t _RoverAckermannStatusTypeSupport) New() types.Message {
	return NewRoverAckermannStatus()
}

func (t _RoverAckermannStatusTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__RoverAckermannStatus
	return (unsafe.Pointer)(C.px4_msgs__msg__RoverAckermannStatus__create())
}

func (t _RoverAckermannStatusTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__RoverAckermannStatus__destroy((*C.px4_msgs__msg__RoverAckermannStatus)(pointer_to_free))
}

func (t _RoverAckermannStatusTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*RoverAckermannStatus)
	mem := (*C.px4_msgs__msg__RoverAckermannStatus)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.throttle_setpoint = C.float(m.ThrottleSetpoint)
	mem.steering_setpoint = C.float(m.SteeringSetpoint)
	mem.actual_speed = C.float(m.ActualSpeed)
}

func (t _RoverAckermannStatusTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*RoverAckermannStatus)
	mem := (*C.px4_msgs__msg__RoverAckermannStatus)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.ThrottleSetpoint = float32(mem.throttle_setpoint)
	m.SteeringSetpoint = float32(mem.steering_setpoint)
	m.ActualSpeed = float32(mem.actual_speed)
}

func (t _RoverAckermannStatusTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__RoverAckermannStatus())
}

type CRoverAckermannStatus = C.px4_msgs__msg__RoverAckermannStatus
type CRoverAckermannStatus__Sequence = C.px4_msgs__msg__RoverAckermannStatus__Sequence

func RoverAckermannStatus__Sequence_to_Go(goSlice *[]RoverAckermannStatus, cSlice CRoverAckermannStatus__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]RoverAckermannStatus, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		RoverAckermannStatusTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func RoverAckermannStatus__Sequence_to_C(cSlice *CRoverAckermannStatus__Sequence, goSlice []RoverAckermannStatus) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__RoverAckermannStatus)(C.malloc(C.sizeof_struct_px4_msgs__msg__RoverAckermannStatus * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		RoverAckermannStatusTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func RoverAckermannStatus__Array_to_Go(goSlice []RoverAckermannStatus, cSlice []CRoverAckermannStatus) {
	for i := 0; i < len(cSlice); i++ {
		RoverAckermannStatusTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func RoverAckermannStatus__Array_to_C(cSlice []CRoverAckermannStatus, goSlice []RoverAckermannStatus) {
	for i := 0; i < len(goSlice); i++ {
		RoverAckermannStatusTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
