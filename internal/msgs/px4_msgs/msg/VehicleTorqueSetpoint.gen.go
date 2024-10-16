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

#include <px4_msgs/msg/vehicle_torque_setpoint.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/VehicleTorqueSetpoint", VehicleTorqueSetpointTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/VehicleTorqueSetpoint", VehicleTorqueSetpointTypeSupport)
}

type VehicleTorqueSetpoint struct {
	Timestamp       uint64     `yaml:"timestamp"`        // time since system start (microseconds)
	TimestampSample uint64     `yaml:"timestamp_sample"` // timestamp of the data sample on which this message is based (microseconds)
	Xyz             [3]float32 `yaml:"xyz"`              // torque setpoint about X, Y, Z body axis (normalized)
}

// NewVehicleTorqueSetpoint creates a new VehicleTorqueSetpoint with default values.
func NewVehicleTorqueSetpoint() *VehicleTorqueSetpoint {
	self := VehicleTorqueSetpoint{}
	self.SetDefaults()
	return &self
}

func (t *VehicleTorqueSetpoint) Clone() *VehicleTorqueSetpoint {
	c := &VehicleTorqueSetpoint{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.Xyz = t.Xyz
	return c
}

func (t *VehicleTorqueSetpoint) CloneMsg() types.Message {
	return t.Clone()
}

func (t *VehicleTorqueSetpoint) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.Xyz = [3]float32{}
}

func (t *VehicleTorqueSetpoint) GetTypeSupport() types.MessageTypeSupport {
	return VehicleTorqueSetpointTypeSupport
}

// VehicleTorqueSetpointPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type VehicleTorqueSetpointPublisher struct {
	*rclgo.Publisher
}

// NewVehicleTorqueSetpointPublisher creates and returns a new publisher for the
// VehicleTorqueSetpoint
func NewVehicleTorqueSetpointPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*VehicleTorqueSetpointPublisher, error) {
	pub, err := node.NewPublisher(topic_name, VehicleTorqueSetpointTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &VehicleTorqueSetpointPublisher{pub}, nil
}

func (p *VehicleTorqueSetpointPublisher) Publish(msg *VehicleTorqueSetpoint) error {
	return p.Publisher.Publish(msg)
}

// VehicleTorqueSetpointSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type VehicleTorqueSetpointSubscription struct {
	*rclgo.Subscription
}

// VehicleTorqueSetpointSubscriptionCallback type is used to provide a subscription
// handler function for a VehicleTorqueSetpointSubscription.
type VehicleTorqueSetpointSubscriptionCallback func(msg *VehicleTorqueSetpoint, info *rclgo.MessageInfo, err error)

// NewVehicleTorqueSetpointSubscription creates and returns a new subscription for the
// VehicleTorqueSetpoint
func NewVehicleTorqueSetpointSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback VehicleTorqueSetpointSubscriptionCallback) (*VehicleTorqueSetpointSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg VehicleTorqueSetpoint
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, VehicleTorqueSetpointTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &VehicleTorqueSetpointSubscription{sub}, nil
}

func (s *VehicleTorqueSetpointSubscription) TakeMessage(out *VehicleTorqueSetpoint) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneVehicleTorqueSetpointSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneVehicleTorqueSetpointSlice(dst, src []VehicleTorqueSetpoint) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var VehicleTorqueSetpointTypeSupport types.MessageTypeSupport = _VehicleTorqueSetpointTypeSupport{}

type _VehicleTorqueSetpointTypeSupport struct{}

func (t _VehicleTorqueSetpointTypeSupport) New() types.Message {
	return NewVehicleTorqueSetpoint()
}

func (t _VehicleTorqueSetpointTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__VehicleTorqueSetpoint
	return (unsafe.Pointer)(C.px4_msgs__msg__VehicleTorqueSetpoint__create())
}

func (t _VehicleTorqueSetpointTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__VehicleTorqueSetpoint__destroy((*C.px4_msgs__msg__VehicleTorqueSetpoint)(pointer_to_free))
}

func (t _VehicleTorqueSetpointTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*VehicleTorqueSetpoint)
	mem := (*C.px4_msgs__msg__VehicleTorqueSetpoint)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	cSlice_xyz := mem.xyz[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_xyz)), m.Xyz[:])
}

func (t _VehicleTorqueSetpointTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*VehicleTorqueSetpoint)
	mem := (*C.px4_msgs__msg__VehicleTorqueSetpoint)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	cSlice_xyz := mem.xyz[:]
	primitives.Float32__Array_to_Go(m.Xyz[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_xyz)))
}

func (t _VehicleTorqueSetpointTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__VehicleTorqueSetpoint())
}

type CVehicleTorqueSetpoint = C.px4_msgs__msg__VehicleTorqueSetpoint
type CVehicleTorqueSetpoint__Sequence = C.px4_msgs__msg__VehicleTorqueSetpoint__Sequence

func VehicleTorqueSetpoint__Sequence_to_Go(goSlice *[]VehicleTorqueSetpoint, cSlice CVehicleTorqueSetpoint__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]VehicleTorqueSetpoint, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		VehicleTorqueSetpointTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func VehicleTorqueSetpoint__Sequence_to_C(cSlice *CVehicleTorqueSetpoint__Sequence, goSlice []VehicleTorqueSetpoint) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__VehicleTorqueSetpoint)(C.malloc(C.sizeof_struct_px4_msgs__msg__VehicleTorqueSetpoint * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		VehicleTorqueSetpointTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func VehicleTorqueSetpoint__Array_to_Go(goSlice []VehicleTorqueSetpoint, cSlice []CVehicleTorqueSetpoint) {
	for i := 0; i < len(cSlice); i++ {
		VehicleTorqueSetpointTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func VehicleTorqueSetpoint__Array_to_C(cSlice []CVehicleTorqueSetpoint, goSlice []VehicleTorqueSetpoint) {
	for i := 0; i < len(goSlice); i++ {
		VehicleTorqueSetpointTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
