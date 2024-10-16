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

#include <px4_msgs/msg/actuator_controls_status.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/ActuatorControlsStatus", ActuatorControlsStatusTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/ActuatorControlsStatus", ActuatorControlsStatusTypeSupport)
}

type ActuatorControlsStatus struct {
	Timestamp    uint64     `yaml:"timestamp"` // time since system start (microseconds)
	ControlPower [3]float32 `yaml:"control_power"`
}

// NewActuatorControlsStatus creates a new ActuatorControlsStatus with default values.
func NewActuatorControlsStatus() *ActuatorControlsStatus {
	self := ActuatorControlsStatus{}
	self.SetDefaults()
	return &self
}

func (t *ActuatorControlsStatus) Clone() *ActuatorControlsStatus {
	c := &ActuatorControlsStatus{}
	c.Timestamp = t.Timestamp
	c.ControlPower = t.ControlPower
	return c
}

func (t *ActuatorControlsStatus) CloneMsg() types.Message {
	return t.Clone()
}

func (t *ActuatorControlsStatus) SetDefaults() {
	t.Timestamp = 0
	t.ControlPower = [3]float32{}
}

func (t *ActuatorControlsStatus) GetTypeSupport() types.MessageTypeSupport {
	return ActuatorControlsStatusTypeSupport
}

// ActuatorControlsStatusPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type ActuatorControlsStatusPublisher struct {
	*rclgo.Publisher
}

// NewActuatorControlsStatusPublisher creates and returns a new publisher for the
// ActuatorControlsStatus
func NewActuatorControlsStatusPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*ActuatorControlsStatusPublisher, error) {
	pub, err := node.NewPublisher(topic_name, ActuatorControlsStatusTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &ActuatorControlsStatusPublisher{pub}, nil
}

func (p *ActuatorControlsStatusPublisher) Publish(msg *ActuatorControlsStatus) error {
	return p.Publisher.Publish(msg)
}

// ActuatorControlsStatusSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type ActuatorControlsStatusSubscription struct {
	*rclgo.Subscription
}

// ActuatorControlsStatusSubscriptionCallback type is used to provide a subscription
// handler function for a ActuatorControlsStatusSubscription.
type ActuatorControlsStatusSubscriptionCallback func(msg *ActuatorControlsStatus, info *rclgo.MessageInfo, err error)

// NewActuatorControlsStatusSubscription creates and returns a new subscription for the
// ActuatorControlsStatus
func NewActuatorControlsStatusSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback ActuatorControlsStatusSubscriptionCallback) (*ActuatorControlsStatusSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg ActuatorControlsStatus
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, ActuatorControlsStatusTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &ActuatorControlsStatusSubscription{sub}, nil
}

func (s *ActuatorControlsStatusSubscription) TakeMessage(out *ActuatorControlsStatus) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneActuatorControlsStatusSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneActuatorControlsStatusSlice(dst, src []ActuatorControlsStatus) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var ActuatorControlsStatusTypeSupport types.MessageTypeSupport = _ActuatorControlsStatusTypeSupport{}

type _ActuatorControlsStatusTypeSupport struct{}

func (t _ActuatorControlsStatusTypeSupport) New() types.Message {
	return NewActuatorControlsStatus()
}

func (t _ActuatorControlsStatusTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__ActuatorControlsStatus
	return (unsafe.Pointer)(C.px4_msgs__msg__ActuatorControlsStatus__create())
}

func (t _ActuatorControlsStatusTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__ActuatorControlsStatus__destroy((*C.px4_msgs__msg__ActuatorControlsStatus)(pointer_to_free))
}

func (t _ActuatorControlsStatusTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*ActuatorControlsStatus)
	mem := (*C.px4_msgs__msg__ActuatorControlsStatus)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	cSlice_control_power := mem.control_power[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_control_power)), m.ControlPower[:])
}

func (t _ActuatorControlsStatusTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*ActuatorControlsStatus)
	mem := (*C.px4_msgs__msg__ActuatorControlsStatus)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	cSlice_control_power := mem.control_power[:]
	primitives.Float32__Array_to_Go(m.ControlPower[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_control_power)))
}

func (t _ActuatorControlsStatusTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__ActuatorControlsStatus())
}

type CActuatorControlsStatus = C.px4_msgs__msg__ActuatorControlsStatus
type CActuatorControlsStatus__Sequence = C.px4_msgs__msg__ActuatorControlsStatus__Sequence

func ActuatorControlsStatus__Sequence_to_Go(goSlice *[]ActuatorControlsStatus, cSlice CActuatorControlsStatus__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]ActuatorControlsStatus, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		ActuatorControlsStatusTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func ActuatorControlsStatus__Sequence_to_C(cSlice *CActuatorControlsStatus__Sequence, goSlice []ActuatorControlsStatus) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__ActuatorControlsStatus)(C.malloc(C.sizeof_struct_px4_msgs__msg__ActuatorControlsStatus * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		ActuatorControlsStatusTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func ActuatorControlsStatus__Array_to_Go(goSlice []ActuatorControlsStatus, cSlice []CActuatorControlsStatus) {
	for i := 0; i < len(cSlice); i++ {
		ActuatorControlsStatusTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func ActuatorControlsStatus__Array_to_C(cSlice []CActuatorControlsStatus, goSlice []ActuatorControlsStatus) {
	for i := 0; i < len(goSlice); i++ {
		ActuatorControlsStatusTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
