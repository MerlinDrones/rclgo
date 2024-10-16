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

#include <px4_msgs/msg/offboard_control_mode.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/OffboardControlMode", OffboardControlModeTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/OffboardControlMode", OffboardControlModeTypeSupport)
}

type OffboardControlMode struct {
	Timestamp       uint64 `yaml:"timestamp"` // time since system start (microseconds)
	Position        bool   `yaml:"position"`
	Velocity        bool   `yaml:"velocity"`
	Acceleration    bool   `yaml:"acceleration"`
	Attitude        bool   `yaml:"attitude"`
	BodyRate        bool   `yaml:"body_rate"`
	ThrustAndTorque bool   `yaml:"thrust_and_torque"`
	DirectActuator  bool   `yaml:"direct_actuator"`
}

// NewOffboardControlMode creates a new OffboardControlMode with default values.
func NewOffboardControlMode() *OffboardControlMode {
	self := OffboardControlMode{}
	self.SetDefaults()
	return &self
}

func (t *OffboardControlMode) Clone() *OffboardControlMode {
	c := &OffboardControlMode{}
	c.Timestamp = t.Timestamp
	c.Position = t.Position
	c.Velocity = t.Velocity
	c.Acceleration = t.Acceleration
	c.Attitude = t.Attitude
	c.BodyRate = t.BodyRate
	c.ThrustAndTorque = t.ThrustAndTorque
	c.DirectActuator = t.DirectActuator
	return c
}

func (t *OffboardControlMode) CloneMsg() types.Message {
	return t.Clone()
}

func (t *OffboardControlMode) SetDefaults() {
	t.Timestamp = 0
	t.Position = false
	t.Velocity = false
	t.Acceleration = false
	t.Attitude = false
	t.BodyRate = false
	t.ThrustAndTorque = false
	t.DirectActuator = false
}

func (t *OffboardControlMode) GetTypeSupport() types.MessageTypeSupport {
	return OffboardControlModeTypeSupport
}

// OffboardControlModePublisher wraps rclgo.Publisher to provide type safe helper
// functions
type OffboardControlModePublisher struct {
	*rclgo.Publisher
}

// NewOffboardControlModePublisher creates and returns a new publisher for the
// OffboardControlMode
func NewOffboardControlModePublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*OffboardControlModePublisher, error) {
	pub, err := node.NewPublisher(topic_name, OffboardControlModeTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &OffboardControlModePublisher{pub}, nil
}

func (p *OffboardControlModePublisher) Publish(msg *OffboardControlMode) error {
	return p.Publisher.Publish(msg)
}

// OffboardControlModeSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type OffboardControlModeSubscription struct {
	*rclgo.Subscription
}

// OffboardControlModeSubscriptionCallback type is used to provide a subscription
// handler function for a OffboardControlModeSubscription.
type OffboardControlModeSubscriptionCallback func(msg *OffboardControlMode, info *rclgo.MessageInfo, err error)

// NewOffboardControlModeSubscription creates and returns a new subscription for the
// OffboardControlMode
func NewOffboardControlModeSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback OffboardControlModeSubscriptionCallback) (*OffboardControlModeSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg OffboardControlMode
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, OffboardControlModeTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &OffboardControlModeSubscription{sub}, nil
}

func (s *OffboardControlModeSubscription) TakeMessage(out *OffboardControlMode) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneOffboardControlModeSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneOffboardControlModeSlice(dst, src []OffboardControlMode) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var OffboardControlModeTypeSupport types.MessageTypeSupport = _OffboardControlModeTypeSupport{}

type _OffboardControlModeTypeSupport struct{}

func (t _OffboardControlModeTypeSupport) New() types.Message {
	return NewOffboardControlMode()
}

func (t _OffboardControlModeTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__OffboardControlMode
	return (unsafe.Pointer)(C.px4_msgs__msg__OffboardControlMode__create())
}

func (t _OffboardControlModeTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__OffboardControlMode__destroy((*C.px4_msgs__msg__OffboardControlMode)(pointer_to_free))
}

func (t _OffboardControlModeTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*OffboardControlMode)
	mem := (*C.px4_msgs__msg__OffboardControlMode)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.position = C.bool(m.Position)
	mem.velocity = C.bool(m.Velocity)
	mem.acceleration = C.bool(m.Acceleration)
	mem.attitude = C.bool(m.Attitude)
	mem.body_rate = C.bool(m.BodyRate)
	mem.thrust_and_torque = C.bool(m.ThrustAndTorque)
	mem.direct_actuator = C.bool(m.DirectActuator)
}

func (t _OffboardControlModeTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*OffboardControlMode)
	mem := (*C.px4_msgs__msg__OffboardControlMode)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.Position = bool(mem.position)
	m.Velocity = bool(mem.velocity)
	m.Acceleration = bool(mem.acceleration)
	m.Attitude = bool(mem.attitude)
	m.BodyRate = bool(mem.body_rate)
	m.ThrustAndTorque = bool(mem.thrust_and_torque)
	m.DirectActuator = bool(mem.direct_actuator)
}

func (t _OffboardControlModeTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__OffboardControlMode())
}

type COffboardControlMode = C.px4_msgs__msg__OffboardControlMode
type COffboardControlMode__Sequence = C.px4_msgs__msg__OffboardControlMode__Sequence

func OffboardControlMode__Sequence_to_Go(goSlice *[]OffboardControlMode, cSlice COffboardControlMode__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]OffboardControlMode, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		OffboardControlModeTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func OffboardControlMode__Sequence_to_C(cSlice *COffboardControlMode__Sequence, goSlice []OffboardControlMode) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__OffboardControlMode)(C.malloc(C.sizeof_struct_px4_msgs__msg__OffboardControlMode * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		OffboardControlModeTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func OffboardControlMode__Array_to_Go(goSlice []OffboardControlMode, cSlice []COffboardControlMode) {
	for i := 0; i < len(cSlice); i++ {
		OffboardControlModeTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func OffboardControlMode__Array_to_C(cSlice []COffboardControlMode, goSlice []OffboardControlMode) {
	for i := 0; i < len(goSlice); i++ {
		OffboardControlModeTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
