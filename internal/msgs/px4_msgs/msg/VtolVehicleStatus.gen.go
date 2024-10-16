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

#include <px4_msgs/msg/vtol_vehicle_status.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/VtolVehicleStatus", VtolVehicleStatusTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/VtolVehicleStatus", VtolVehicleStatusTypeSupport)
}

const (
	VtolVehicleStatus_VEHICLE_VTOL_STATE_UNDEFINED        uint8 = 0 // VEHICLE_VTOL_STATE, should match 1:1 MAVLinks's MAV_VTOL_STATE
	VtolVehicleStatus_VEHICLE_VTOL_STATE_TRANSITION_TO_FW uint8 = 1
	VtolVehicleStatus_VEHICLE_VTOL_STATE_TRANSITION_TO_MC uint8 = 2
	VtolVehicleStatus_VEHICLE_VTOL_STATE_MC               uint8 = 3
	VtolVehicleStatus_VEHICLE_VTOL_STATE_FW               uint8 = 4
)

type VtolVehicleStatus struct {
	Timestamp              uint64 `yaml:"timestamp"`                 // time since system start (microseconds)
	VehicleVtolState       uint8  `yaml:"vehicle_vtol_state"`        // current state of the vtol, see VEHICLE_VTOL_STATE
	FixedWingSystemFailure bool   `yaml:"fixed_wing_system_failure"` // vehicle in fixed-wing system failure failsafe mode (after quad-chute)
}

// NewVtolVehicleStatus creates a new VtolVehicleStatus with default values.
func NewVtolVehicleStatus() *VtolVehicleStatus {
	self := VtolVehicleStatus{}
	self.SetDefaults()
	return &self
}

func (t *VtolVehicleStatus) Clone() *VtolVehicleStatus {
	c := &VtolVehicleStatus{}
	c.Timestamp = t.Timestamp
	c.VehicleVtolState = t.VehicleVtolState
	c.FixedWingSystemFailure = t.FixedWingSystemFailure
	return c
}

func (t *VtolVehicleStatus) CloneMsg() types.Message {
	return t.Clone()
}

func (t *VtolVehicleStatus) SetDefaults() {
	t.Timestamp = 0
	t.VehicleVtolState = 0
	t.FixedWingSystemFailure = false
}

func (t *VtolVehicleStatus) GetTypeSupport() types.MessageTypeSupport {
	return VtolVehicleStatusTypeSupport
}

// VtolVehicleStatusPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type VtolVehicleStatusPublisher struct {
	*rclgo.Publisher
}

// NewVtolVehicleStatusPublisher creates and returns a new publisher for the
// VtolVehicleStatus
func NewVtolVehicleStatusPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*VtolVehicleStatusPublisher, error) {
	pub, err := node.NewPublisher(topic_name, VtolVehicleStatusTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &VtolVehicleStatusPublisher{pub}, nil
}

func (p *VtolVehicleStatusPublisher) Publish(msg *VtolVehicleStatus) error {
	return p.Publisher.Publish(msg)
}

// VtolVehicleStatusSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type VtolVehicleStatusSubscription struct {
	*rclgo.Subscription
}

// VtolVehicleStatusSubscriptionCallback type is used to provide a subscription
// handler function for a VtolVehicleStatusSubscription.
type VtolVehicleStatusSubscriptionCallback func(msg *VtolVehicleStatus, info *rclgo.MessageInfo, err error)

// NewVtolVehicleStatusSubscription creates and returns a new subscription for the
// VtolVehicleStatus
func NewVtolVehicleStatusSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback VtolVehicleStatusSubscriptionCallback) (*VtolVehicleStatusSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg VtolVehicleStatus
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, VtolVehicleStatusTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &VtolVehicleStatusSubscription{sub}, nil
}

func (s *VtolVehicleStatusSubscription) TakeMessage(out *VtolVehicleStatus) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneVtolVehicleStatusSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneVtolVehicleStatusSlice(dst, src []VtolVehicleStatus) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var VtolVehicleStatusTypeSupport types.MessageTypeSupport = _VtolVehicleStatusTypeSupport{}

type _VtolVehicleStatusTypeSupport struct{}

func (t _VtolVehicleStatusTypeSupport) New() types.Message {
	return NewVtolVehicleStatus()
}

func (t _VtolVehicleStatusTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__VtolVehicleStatus
	return (unsafe.Pointer)(C.px4_msgs__msg__VtolVehicleStatus__create())
}

func (t _VtolVehicleStatusTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__VtolVehicleStatus__destroy((*C.px4_msgs__msg__VtolVehicleStatus)(pointer_to_free))
}

func (t _VtolVehicleStatusTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*VtolVehicleStatus)
	mem := (*C.px4_msgs__msg__VtolVehicleStatus)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.vehicle_vtol_state = C.uint8_t(m.VehicleVtolState)
	mem.fixed_wing_system_failure = C.bool(m.FixedWingSystemFailure)
}

func (t _VtolVehicleStatusTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*VtolVehicleStatus)
	mem := (*C.px4_msgs__msg__VtolVehicleStatus)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.VehicleVtolState = uint8(mem.vehicle_vtol_state)
	m.FixedWingSystemFailure = bool(mem.fixed_wing_system_failure)
}

func (t _VtolVehicleStatusTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__VtolVehicleStatus())
}

type CVtolVehicleStatus = C.px4_msgs__msg__VtolVehicleStatus
type CVtolVehicleStatus__Sequence = C.px4_msgs__msg__VtolVehicleStatus__Sequence

func VtolVehicleStatus__Sequence_to_Go(goSlice *[]VtolVehicleStatus, cSlice CVtolVehicleStatus__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]VtolVehicleStatus, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		VtolVehicleStatusTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func VtolVehicleStatus__Sequence_to_C(cSlice *CVtolVehicleStatus__Sequence, goSlice []VtolVehicleStatus) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__VtolVehicleStatus)(C.malloc(C.sizeof_struct_px4_msgs__msg__VtolVehicleStatus * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		VtolVehicleStatusTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func VtolVehicleStatus__Array_to_Go(goSlice []VtolVehicleStatus, cSlice []CVtolVehicleStatus) {
	for i := 0; i < len(cSlice); i++ {
		VtolVehicleStatusTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func VtolVehicleStatus__Array_to_C(cSlice []CVtolVehicleStatus, goSlice []VtolVehicleStatus) {
	for i := 0; i < len(goSlice); i++ {
		VtolVehicleStatusTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
