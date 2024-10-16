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

#include <px4_msgs/msg/vehicle_angular_velocity.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/VehicleAngularVelocity", VehicleAngularVelocityTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/VehicleAngularVelocity", VehicleAngularVelocityTypeSupport)
}

type VehicleAngularVelocity struct {
	Timestamp       uint64     `yaml:"timestamp"`        // time since system start (microseconds)
	TimestampSample uint64     `yaml:"timestamp_sample"` // timestamp of the data sample on which this message is based (microseconds)
	Xyz             [3]float32 `yaml:"xyz"`              // Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s
	XyzDerivative   [3]float32 `yaml:"xyz_derivative"`   // angular acceleration about the FRD body frame XYZ-axis in rad/s^2
}

// NewVehicleAngularVelocity creates a new VehicleAngularVelocity with default values.
func NewVehicleAngularVelocity() *VehicleAngularVelocity {
	self := VehicleAngularVelocity{}
	self.SetDefaults()
	return &self
}

func (t *VehicleAngularVelocity) Clone() *VehicleAngularVelocity {
	c := &VehicleAngularVelocity{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.Xyz = t.Xyz
	c.XyzDerivative = t.XyzDerivative
	return c
}

func (t *VehicleAngularVelocity) CloneMsg() types.Message {
	return t.Clone()
}

func (t *VehicleAngularVelocity) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.Xyz = [3]float32{}
	t.XyzDerivative = [3]float32{}
}

func (t *VehicleAngularVelocity) GetTypeSupport() types.MessageTypeSupport {
	return VehicleAngularVelocityTypeSupport
}

// VehicleAngularVelocityPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type VehicleAngularVelocityPublisher struct {
	*rclgo.Publisher
}

// NewVehicleAngularVelocityPublisher creates and returns a new publisher for the
// VehicleAngularVelocity
func NewVehicleAngularVelocityPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*VehicleAngularVelocityPublisher, error) {
	pub, err := node.NewPublisher(topic_name, VehicleAngularVelocityTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &VehicleAngularVelocityPublisher{pub}, nil
}

func (p *VehicleAngularVelocityPublisher) Publish(msg *VehicleAngularVelocity) error {
	return p.Publisher.Publish(msg)
}

// VehicleAngularVelocitySubscription wraps rclgo.Subscription to provide type safe helper
// functions
type VehicleAngularVelocitySubscription struct {
	*rclgo.Subscription
}

// VehicleAngularVelocitySubscriptionCallback type is used to provide a subscription
// handler function for a VehicleAngularVelocitySubscription.
type VehicleAngularVelocitySubscriptionCallback func(msg *VehicleAngularVelocity, info *rclgo.MessageInfo, err error)

// NewVehicleAngularVelocitySubscription creates and returns a new subscription for the
// VehicleAngularVelocity
func NewVehicleAngularVelocitySubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback VehicleAngularVelocitySubscriptionCallback) (*VehicleAngularVelocitySubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg VehicleAngularVelocity
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, VehicleAngularVelocityTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &VehicleAngularVelocitySubscription{sub}, nil
}

func (s *VehicleAngularVelocitySubscription) TakeMessage(out *VehicleAngularVelocity) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneVehicleAngularVelocitySlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneVehicleAngularVelocitySlice(dst, src []VehicleAngularVelocity) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var VehicleAngularVelocityTypeSupport types.MessageTypeSupport = _VehicleAngularVelocityTypeSupport{}

type _VehicleAngularVelocityTypeSupport struct{}

func (t _VehicleAngularVelocityTypeSupport) New() types.Message {
	return NewVehicleAngularVelocity()
}

func (t _VehicleAngularVelocityTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__VehicleAngularVelocity
	return (unsafe.Pointer)(C.px4_msgs__msg__VehicleAngularVelocity__create())
}

func (t _VehicleAngularVelocityTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__VehicleAngularVelocity__destroy((*C.px4_msgs__msg__VehicleAngularVelocity)(pointer_to_free))
}

func (t _VehicleAngularVelocityTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*VehicleAngularVelocity)
	mem := (*C.px4_msgs__msg__VehicleAngularVelocity)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	cSlice_xyz := mem.xyz[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_xyz)), m.Xyz[:])
	cSlice_xyz_derivative := mem.xyz_derivative[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_xyz_derivative)), m.XyzDerivative[:])
}

func (t _VehicleAngularVelocityTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*VehicleAngularVelocity)
	mem := (*C.px4_msgs__msg__VehicleAngularVelocity)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	cSlice_xyz := mem.xyz[:]
	primitives.Float32__Array_to_Go(m.Xyz[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_xyz)))
	cSlice_xyz_derivative := mem.xyz_derivative[:]
	primitives.Float32__Array_to_Go(m.XyzDerivative[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_xyz_derivative)))
}

func (t _VehicleAngularVelocityTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__VehicleAngularVelocity())
}

type CVehicleAngularVelocity = C.px4_msgs__msg__VehicleAngularVelocity
type CVehicleAngularVelocity__Sequence = C.px4_msgs__msg__VehicleAngularVelocity__Sequence

func VehicleAngularVelocity__Sequence_to_Go(goSlice *[]VehicleAngularVelocity, cSlice CVehicleAngularVelocity__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]VehicleAngularVelocity, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		VehicleAngularVelocityTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func VehicleAngularVelocity__Sequence_to_C(cSlice *CVehicleAngularVelocity__Sequence, goSlice []VehicleAngularVelocity) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__VehicleAngularVelocity)(C.malloc(C.sizeof_struct_px4_msgs__msg__VehicleAngularVelocity * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		VehicleAngularVelocityTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func VehicleAngularVelocity__Array_to_Go(goSlice []VehicleAngularVelocity, cSlice []CVehicleAngularVelocity) {
	for i := 0; i < len(cSlice); i++ {
		VehicleAngularVelocityTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func VehicleAngularVelocity__Array_to_C(cSlice []CVehicleAngularVelocity, goSlice []VehicleAngularVelocity) {
	for i := 0; i < len(goSlice); i++ {
		VehicleAngularVelocityTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
