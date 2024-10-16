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

#include <px4_msgs/msg/rover_differential_setpoint.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/RoverDifferentialSetpoint", RoverDifferentialSetpointTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/RoverDifferentialSetpoint", RoverDifferentialSetpointTypeSupport)
}

type RoverDifferentialSetpoint struct {
	Timestamp                      uint64  `yaml:"timestamp"`                         // time since system start (microseconds)
	ForwardSpeedSetpoint           float32 `yaml:"forward_speed_setpoint"`            // [m/s] Desired forward speed for the rover
	ForwardSpeedSetpointNormalized float32 `yaml:"forward_speed_setpoint_normalized"` // [-1, 1] Normalized forward speed for the rover
	YawRateSetpoint                float32 `yaml:"yaw_rate_setpoint"`                 // [rad/s] Desired yaw rate for the rover (Overriden by yaw controller if yaw_setpoint is used)
	YawRateSetpointNormalized      float32 `yaml:"yaw_rate_setpoint_normalized"`      // [-1, 1] Normalized yaw rate for the rover
	YawSetpoint                    float32 `yaml:"yaw_setpoint"`                      // [rad] Desired yaw (heading) for the rover
}

// NewRoverDifferentialSetpoint creates a new RoverDifferentialSetpoint with default values.
func NewRoverDifferentialSetpoint() *RoverDifferentialSetpoint {
	self := RoverDifferentialSetpoint{}
	self.SetDefaults()
	return &self
}

func (t *RoverDifferentialSetpoint) Clone() *RoverDifferentialSetpoint {
	c := &RoverDifferentialSetpoint{}
	c.Timestamp = t.Timestamp
	c.ForwardSpeedSetpoint = t.ForwardSpeedSetpoint
	c.ForwardSpeedSetpointNormalized = t.ForwardSpeedSetpointNormalized
	c.YawRateSetpoint = t.YawRateSetpoint
	c.YawRateSetpointNormalized = t.YawRateSetpointNormalized
	c.YawSetpoint = t.YawSetpoint
	return c
}

func (t *RoverDifferentialSetpoint) CloneMsg() types.Message {
	return t.Clone()
}

func (t *RoverDifferentialSetpoint) SetDefaults() {
	t.Timestamp = 0
	t.ForwardSpeedSetpoint = 0
	t.ForwardSpeedSetpointNormalized = 0
	t.YawRateSetpoint = 0
	t.YawRateSetpointNormalized = 0
	t.YawSetpoint = 0
}

func (t *RoverDifferentialSetpoint) GetTypeSupport() types.MessageTypeSupport {
	return RoverDifferentialSetpointTypeSupport
}

// RoverDifferentialSetpointPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type RoverDifferentialSetpointPublisher struct {
	*rclgo.Publisher
}

// NewRoverDifferentialSetpointPublisher creates and returns a new publisher for the
// RoverDifferentialSetpoint
func NewRoverDifferentialSetpointPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*RoverDifferentialSetpointPublisher, error) {
	pub, err := node.NewPublisher(topic_name, RoverDifferentialSetpointTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &RoverDifferentialSetpointPublisher{pub}, nil
}

func (p *RoverDifferentialSetpointPublisher) Publish(msg *RoverDifferentialSetpoint) error {
	return p.Publisher.Publish(msg)
}

// RoverDifferentialSetpointSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type RoverDifferentialSetpointSubscription struct {
	*rclgo.Subscription
}

// RoverDifferentialSetpointSubscriptionCallback type is used to provide a subscription
// handler function for a RoverDifferentialSetpointSubscription.
type RoverDifferentialSetpointSubscriptionCallback func(msg *RoverDifferentialSetpoint, info *rclgo.MessageInfo, err error)

// NewRoverDifferentialSetpointSubscription creates and returns a new subscription for the
// RoverDifferentialSetpoint
func NewRoverDifferentialSetpointSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback RoverDifferentialSetpointSubscriptionCallback) (*RoverDifferentialSetpointSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg RoverDifferentialSetpoint
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, RoverDifferentialSetpointTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &RoverDifferentialSetpointSubscription{sub}, nil
}

func (s *RoverDifferentialSetpointSubscription) TakeMessage(out *RoverDifferentialSetpoint) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneRoverDifferentialSetpointSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneRoverDifferentialSetpointSlice(dst, src []RoverDifferentialSetpoint) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var RoverDifferentialSetpointTypeSupport types.MessageTypeSupport = _RoverDifferentialSetpointTypeSupport{}

type _RoverDifferentialSetpointTypeSupport struct{}

func (t _RoverDifferentialSetpointTypeSupport) New() types.Message {
	return NewRoverDifferentialSetpoint()
}

func (t _RoverDifferentialSetpointTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__RoverDifferentialSetpoint
	return (unsafe.Pointer)(C.px4_msgs__msg__RoverDifferentialSetpoint__create())
}

func (t _RoverDifferentialSetpointTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__RoverDifferentialSetpoint__destroy((*C.px4_msgs__msg__RoverDifferentialSetpoint)(pointer_to_free))
}

func (t _RoverDifferentialSetpointTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*RoverDifferentialSetpoint)
	mem := (*C.px4_msgs__msg__RoverDifferentialSetpoint)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.forward_speed_setpoint = C.float(m.ForwardSpeedSetpoint)
	mem.forward_speed_setpoint_normalized = C.float(m.ForwardSpeedSetpointNormalized)
	mem.yaw_rate_setpoint = C.float(m.YawRateSetpoint)
	mem.yaw_rate_setpoint_normalized = C.float(m.YawRateSetpointNormalized)
	mem.yaw_setpoint = C.float(m.YawSetpoint)
}

func (t _RoverDifferentialSetpointTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*RoverDifferentialSetpoint)
	mem := (*C.px4_msgs__msg__RoverDifferentialSetpoint)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.ForwardSpeedSetpoint = float32(mem.forward_speed_setpoint)
	m.ForwardSpeedSetpointNormalized = float32(mem.forward_speed_setpoint_normalized)
	m.YawRateSetpoint = float32(mem.yaw_rate_setpoint)
	m.YawRateSetpointNormalized = float32(mem.yaw_rate_setpoint_normalized)
	m.YawSetpoint = float32(mem.yaw_setpoint)
}

func (t _RoverDifferentialSetpointTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__RoverDifferentialSetpoint())
}

type CRoverDifferentialSetpoint = C.px4_msgs__msg__RoverDifferentialSetpoint
type CRoverDifferentialSetpoint__Sequence = C.px4_msgs__msg__RoverDifferentialSetpoint__Sequence

func RoverDifferentialSetpoint__Sequence_to_Go(goSlice *[]RoverDifferentialSetpoint, cSlice CRoverDifferentialSetpoint__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]RoverDifferentialSetpoint, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		RoverDifferentialSetpointTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func RoverDifferentialSetpoint__Sequence_to_C(cSlice *CRoverDifferentialSetpoint__Sequence, goSlice []RoverDifferentialSetpoint) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__RoverDifferentialSetpoint)(C.malloc(C.sizeof_struct_px4_msgs__msg__RoverDifferentialSetpoint * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		RoverDifferentialSetpointTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func RoverDifferentialSetpoint__Array_to_Go(goSlice []RoverDifferentialSetpoint, cSlice []CRoverDifferentialSetpoint) {
	for i := 0; i < len(cSlice); i++ {
		RoverDifferentialSetpointTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func RoverDifferentialSetpoint__Array_to_C(cSlice []CRoverDifferentialSetpoint, goSlice []RoverDifferentialSetpoint) {
	for i := 0; i < len(goSlice); i++ {
		RoverDifferentialSetpointTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
