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

#include <px4_msgs/msg/estimator_bias3d.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/EstimatorBias3d", EstimatorBias3dTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/EstimatorBias3d", EstimatorBias3dTypeSupport)
}

type EstimatorBias3d struct {
	Timestamp       uint64     `yaml:"timestamp"`        // time since system start (microseconds)
	TimestampSample uint64     `yaml:"timestamp_sample"` // the timestamp of the raw data (microseconds)
	DeviceId        uint32     `yaml:"device_id"`        // unique device ID for the sensor that does not change between power cycles
	Bias            [3]float32 `yaml:"bias"`             // estimated barometric altitude bias (m)
	BiasVar         [3]float32 `yaml:"bias_var"`         // estimated barometric altitude bias variance (m^2)
	Innov           [3]float32 `yaml:"innov"`            // innovation of the last measurement fusion (m)
	InnovVar        [3]float32 `yaml:"innov_var"`        // innovation variance of the last measurement fusion (m^2)
	InnovTestRatio  [3]float32 `yaml:"innov_test_ratio"` // normalized innovation squared test ratio
}

// NewEstimatorBias3d creates a new EstimatorBias3d with default values.
func NewEstimatorBias3d() *EstimatorBias3d {
	self := EstimatorBias3d{}
	self.SetDefaults()
	return &self
}

func (t *EstimatorBias3d) Clone() *EstimatorBias3d {
	c := &EstimatorBias3d{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.DeviceId = t.DeviceId
	c.Bias = t.Bias
	c.BiasVar = t.BiasVar
	c.Innov = t.Innov
	c.InnovVar = t.InnovVar
	c.InnovTestRatio = t.InnovTestRatio
	return c
}

func (t *EstimatorBias3d) CloneMsg() types.Message {
	return t.Clone()
}

func (t *EstimatorBias3d) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.DeviceId = 0
	t.Bias = [3]float32{}
	t.BiasVar = [3]float32{}
	t.Innov = [3]float32{}
	t.InnovVar = [3]float32{}
	t.InnovTestRatio = [3]float32{}
}

func (t *EstimatorBias3d) GetTypeSupport() types.MessageTypeSupport {
	return EstimatorBias3dTypeSupport
}

// EstimatorBias3dPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type EstimatorBias3dPublisher struct {
	*rclgo.Publisher
}

// NewEstimatorBias3dPublisher creates and returns a new publisher for the
// EstimatorBias3d
func NewEstimatorBias3dPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*EstimatorBias3dPublisher, error) {
	pub, err := node.NewPublisher(topic_name, EstimatorBias3dTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &EstimatorBias3dPublisher{pub}, nil
}

func (p *EstimatorBias3dPublisher) Publish(msg *EstimatorBias3d) error {
	return p.Publisher.Publish(msg)
}

// EstimatorBias3dSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type EstimatorBias3dSubscription struct {
	*rclgo.Subscription
}

// EstimatorBias3dSubscriptionCallback type is used to provide a subscription
// handler function for a EstimatorBias3dSubscription.
type EstimatorBias3dSubscriptionCallback func(msg *EstimatorBias3d, info *rclgo.MessageInfo, err error)

// NewEstimatorBias3dSubscription creates and returns a new subscription for the
// EstimatorBias3d
func NewEstimatorBias3dSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback EstimatorBias3dSubscriptionCallback) (*EstimatorBias3dSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg EstimatorBias3d
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, EstimatorBias3dTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &EstimatorBias3dSubscription{sub}, nil
}

func (s *EstimatorBias3dSubscription) TakeMessage(out *EstimatorBias3d) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneEstimatorBias3dSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneEstimatorBias3dSlice(dst, src []EstimatorBias3d) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var EstimatorBias3dTypeSupport types.MessageTypeSupport = _EstimatorBias3dTypeSupport{}

type _EstimatorBias3dTypeSupport struct{}

func (t _EstimatorBias3dTypeSupport) New() types.Message {
	return NewEstimatorBias3d()
}

func (t _EstimatorBias3dTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__EstimatorBias3d
	return (unsafe.Pointer)(C.px4_msgs__msg__EstimatorBias3d__create())
}

func (t _EstimatorBias3dTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__EstimatorBias3d__destroy((*C.px4_msgs__msg__EstimatorBias3d)(pointer_to_free))
}

func (t _EstimatorBias3dTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*EstimatorBias3d)
	mem := (*C.px4_msgs__msg__EstimatorBias3d)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	mem.device_id = C.uint32_t(m.DeviceId)
	cSlice_bias := mem.bias[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_bias)), m.Bias[:])
	cSlice_bias_var := mem.bias_var[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_bias_var)), m.BiasVar[:])
	cSlice_innov := mem.innov[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov)), m.Innov[:])
	cSlice_innov_var := mem.innov_var[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov_var)), m.InnovVar[:])
	cSlice_innov_test_ratio := mem.innov_test_ratio[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov_test_ratio)), m.InnovTestRatio[:])
}

func (t _EstimatorBias3dTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*EstimatorBias3d)
	mem := (*C.px4_msgs__msg__EstimatorBias3d)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	m.DeviceId = uint32(mem.device_id)
	cSlice_bias := mem.bias[:]
	primitives.Float32__Array_to_Go(m.Bias[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_bias)))
	cSlice_bias_var := mem.bias_var[:]
	primitives.Float32__Array_to_Go(m.BiasVar[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_bias_var)))
	cSlice_innov := mem.innov[:]
	primitives.Float32__Array_to_Go(m.Innov[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov)))
	cSlice_innov_var := mem.innov_var[:]
	primitives.Float32__Array_to_Go(m.InnovVar[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov_var)))
	cSlice_innov_test_ratio := mem.innov_test_ratio[:]
	primitives.Float32__Array_to_Go(m.InnovTestRatio[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov_test_ratio)))
}

func (t _EstimatorBias3dTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__EstimatorBias3d())
}

type CEstimatorBias3d = C.px4_msgs__msg__EstimatorBias3d
type CEstimatorBias3d__Sequence = C.px4_msgs__msg__EstimatorBias3d__Sequence

func EstimatorBias3d__Sequence_to_Go(goSlice *[]EstimatorBias3d, cSlice CEstimatorBias3d__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]EstimatorBias3d, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		EstimatorBias3dTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func EstimatorBias3d__Sequence_to_C(cSlice *CEstimatorBias3d__Sequence, goSlice []EstimatorBias3d) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__EstimatorBias3d)(C.malloc(C.sizeof_struct_px4_msgs__msg__EstimatorBias3d * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		EstimatorBias3dTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func EstimatorBias3d__Array_to_Go(goSlice []EstimatorBias3d, cSlice []CEstimatorBias3d) {
	for i := 0; i < len(cSlice); i++ {
		EstimatorBias3dTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func EstimatorBias3d__Array_to_C(cSlice []CEstimatorBias3d, goSlice []EstimatorBias3d) {
	for i := 0; i < len(goSlice); i++ {
		EstimatorBias3dTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
