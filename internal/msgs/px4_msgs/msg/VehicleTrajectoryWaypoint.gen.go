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

#include <px4_msgs/msg/vehicle_trajectory_waypoint.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/VehicleTrajectoryWaypoint", VehicleTrajectoryWaypointTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/VehicleTrajectoryWaypoint", VehicleTrajectoryWaypointTypeSupport)
}

const (
	VehicleTrajectoryWaypoint_MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS uint8 = 0
	VehicleTrajectoryWaypoint_POINT_0                                 uint8 = 0
	VehicleTrajectoryWaypoint_POINT_1                                 uint8 = 1
	VehicleTrajectoryWaypoint_POINT_2                                 uint8 = 2
	VehicleTrajectoryWaypoint_POINT_3                                 uint8 = 3
	VehicleTrajectoryWaypoint_POINT_4                                 uint8 = 4
	VehicleTrajectoryWaypoint_NUMBER_POINTS                           uint8 = 5
)

type VehicleTrajectoryWaypoint struct {
	Timestamp uint64                `yaml:"timestamp"` // time since system start (microseconds)
	Type      uint8                 `yaml:"type"`      // Type from MAV_TRAJECTORY_REPRESENTATION enum.
	Waypoints [5]TrajectoryWaypoint `yaml:"waypoints"`
}

// NewVehicleTrajectoryWaypoint creates a new VehicleTrajectoryWaypoint with default values.
func NewVehicleTrajectoryWaypoint() *VehicleTrajectoryWaypoint {
	self := VehicleTrajectoryWaypoint{}
	self.SetDefaults()
	return &self
}

func (t *VehicleTrajectoryWaypoint) Clone() *VehicleTrajectoryWaypoint {
	c := &VehicleTrajectoryWaypoint{}
	c.Timestamp = t.Timestamp
	c.Type = t.Type
	CloneTrajectoryWaypointSlice(c.Waypoints[:], t.Waypoints[:])
	return c
}

func (t *VehicleTrajectoryWaypoint) CloneMsg() types.Message {
	return t.Clone()
}

func (t *VehicleTrajectoryWaypoint) SetDefaults() {
	t.Timestamp = 0
	t.Type = 0
	for i := range t.Waypoints {
		t.Waypoints[i].SetDefaults()
	}
}

func (t *VehicleTrajectoryWaypoint) GetTypeSupport() types.MessageTypeSupport {
	return VehicleTrajectoryWaypointTypeSupport
}

// VehicleTrajectoryWaypointPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type VehicleTrajectoryWaypointPublisher struct {
	*rclgo.Publisher
}

// NewVehicleTrajectoryWaypointPublisher creates and returns a new publisher for the
// VehicleTrajectoryWaypoint
func NewVehicleTrajectoryWaypointPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*VehicleTrajectoryWaypointPublisher, error) {
	pub, err := node.NewPublisher(topic_name, VehicleTrajectoryWaypointTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &VehicleTrajectoryWaypointPublisher{pub}, nil
}

func (p *VehicleTrajectoryWaypointPublisher) Publish(msg *VehicleTrajectoryWaypoint) error {
	return p.Publisher.Publish(msg)
}

// VehicleTrajectoryWaypointSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type VehicleTrajectoryWaypointSubscription struct {
	*rclgo.Subscription
}

// VehicleTrajectoryWaypointSubscriptionCallback type is used to provide a subscription
// handler function for a VehicleTrajectoryWaypointSubscription.
type VehicleTrajectoryWaypointSubscriptionCallback func(msg *VehicleTrajectoryWaypoint, info *rclgo.MessageInfo, err error)

// NewVehicleTrajectoryWaypointSubscription creates and returns a new subscription for the
// VehicleTrajectoryWaypoint
func NewVehicleTrajectoryWaypointSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback VehicleTrajectoryWaypointSubscriptionCallback) (*VehicleTrajectoryWaypointSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg VehicleTrajectoryWaypoint
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, VehicleTrajectoryWaypointTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &VehicleTrajectoryWaypointSubscription{sub}, nil
}

func (s *VehicleTrajectoryWaypointSubscription) TakeMessage(out *VehicleTrajectoryWaypoint) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneVehicleTrajectoryWaypointSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneVehicleTrajectoryWaypointSlice(dst, src []VehicleTrajectoryWaypoint) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var VehicleTrajectoryWaypointTypeSupport types.MessageTypeSupport = _VehicleTrajectoryWaypointTypeSupport{}

type _VehicleTrajectoryWaypointTypeSupport struct{}

func (t _VehicleTrajectoryWaypointTypeSupport) New() types.Message {
	return NewVehicleTrajectoryWaypoint()
}

func (t _VehicleTrajectoryWaypointTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__VehicleTrajectoryWaypoint
	return (unsafe.Pointer)(C.px4_msgs__msg__VehicleTrajectoryWaypoint__create())
}

func (t _VehicleTrajectoryWaypointTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__VehicleTrajectoryWaypoint__destroy((*C.px4_msgs__msg__VehicleTrajectoryWaypoint)(pointer_to_free))
}

func (t _VehicleTrajectoryWaypointTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*VehicleTrajectoryWaypoint)
	mem := (*C.px4_msgs__msg__VehicleTrajectoryWaypoint)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem._type = C.uint8_t(m.Type)
	TrajectoryWaypoint__Array_to_C(mem.waypoints[:], m.Waypoints[:])
}

func (t _VehicleTrajectoryWaypointTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*VehicleTrajectoryWaypoint)
	mem := (*C.px4_msgs__msg__VehicleTrajectoryWaypoint)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.Type = uint8(mem._type)
	TrajectoryWaypoint__Array_to_Go(m.Waypoints[:], mem.waypoints[:])
}

func (t _VehicleTrajectoryWaypointTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__VehicleTrajectoryWaypoint())
}

type CVehicleTrajectoryWaypoint = C.px4_msgs__msg__VehicleTrajectoryWaypoint
type CVehicleTrajectoryWaypoint__Sequence = C.px4_msgs__msg__VehicleTrajectoryWaypoint__Sequence

func VehicleTrajectoryWaypoint__Sequence_to_Go(goSlice *[]VehicleTrajectoryWaypoint, cSlice CVehicleTrajectoryWaypoint__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]VehicleTrajectoryWaypoint, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		VehicleTrajectoryWaypointTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func VehicleTrajectoryWaypoint__Sequence_to_C(cSlice *CVehicleTrajectoryWaypoint__Sequence, goSlice []VehicleTrajectoryWaypoint) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__VehicleTrajectoryWaypoint)(C.malloc(C.sizeof_struct_px4_msgs__msg__VehicleTrajectoryWaypoint * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		VehicleTrajectoryWaypointTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func VehicleTrajectoryWaypoint__Array_to_Go(goSlice []VehicleTrajectoryWaypoint, cSlice []CVehicleTrajectoryWaypoint) {
	for i := 0; i < len(cSlice); i++ {
		VehicleTrajectoryWaypointTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func VehicleTrajectoryWaypoint__Array_to_C(cSlice []CVehicleTrajectoryWaypoint, goSlice []VehicleTrajectoryWaypoint) {
	for i := 0; i < len(goSlice); i++ {
		VehicleTrajectoryWaypointTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
