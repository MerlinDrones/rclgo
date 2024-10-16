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

#include <px4_msgs/msg/yaw_estimator_status.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/YawEstimatorStatus", YawEstimatorStatusTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/YawEstimatorStatus", YawEstimatorStatusTypeSupport)
}

type YawEstimatorStatus struct {
	Timestamp         uint64     `yaml:"timestamp"`        // time since system start (microseconds)
	TimestampSample   uint64     `yaml:"timestamp_sample"` // the timestamp of the raw data (microseconds)
	YawComposite      float32    `yaml:"yaw_composite"`    // composite yaw from GSF (rad)
	YawVariance       float32    `yaml:"yaw_variance"`     // composite yaw variance from GSF (rad^2)
	YawCompositeValid bool       `yaml:"yaw_composite_valid"`
	Yaw               [5]float32 `yaml:"yaw"`      // yaw estimate for each model in the filter bank (rad)
	InnovVn           [5]float32 `yaml:"innov_vn"` // North velocity innovation for each model in the filter bank (m/s)
	InnovVe           [5]float32 `yaml:"innov_ve"` // East velocity innovation for each model in the filter bank (m/s)
	Weight            [5]float32 `yaml:"weight"`   // weighting for each model in the filter bank
}

// NewYawEstimatorStatus creates a new YawEstimatorStatus with default values.
func NewYawEstimatorStatus() *YawEstimatorStatus {
	self := YawEstimatorStatus{}
	self.SetDefaults()
	return &self
}

func (t *YawEstimatorStatus) Clone() *YawEstimatorStatus {
	c := &YawEstimatorStatus{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.YawComposite = t.YawComposite
	c.YawVariance = t.YawVariance
	c.YawCompositeValid = t.YawCompositeValid
	c.Yaw = t.Yaw
	c.InnovVn = t.InnovVn
	c.InnovVe = t.InnovVe
	c.Weight = t.Weight
	return c
}

func (t *YawEstimatorStatus) CloneMsg() types.Message {
	return t.Clone()
}

func (t *YawEstimatorStatus) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.YawComposite = 0
	t.YawVariance = 0
	t.YawCompositeValid = false
	t.Yaw = [5]float32{}
	t.InnovVn = [5]float32{}
	t.InnovVe = [5]float32{}
	t.Weight = [5]float32{}
}

func (t *YawEstimatorStatus) GetTypeSupport() types.MessageTypeSupport {
	return YawEstimatorStatusTypeSupport
}

// YawEstimatorStatusPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type YawEstimatorStatusPublisher struct {
	*rclgo.Publisher
}

// NewYawEstimatorStatusPublisher creates and returns a new publisher for the
// YawEstimatorStatus
func NewYawEstimatorStatusPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*YawEstimatorStatusPublisher, error) {
	pub, err := node.NewPublisher(topic_name, YawEstimatorStatusTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &YawEstimatorStatusPublisher{pub}, nil
}

func (p *YawEstimatorStatusPublisher) Publish(msg *YawEstimatorStatus) error {
	return p.Publisher.Publish(msg)
}

// YawEstimatorStatusSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type YawEstimatorStatusSubscription struct {
	*rclgo.Subscription
}

// YawEstimatorStatusSubscriptionCallback type is used to provide a subscription
// handler function for a YawEstimatorStatusSubscription.
type YawEstimatorStatusSubscriptionCallback func(msg *YawEstimatorStatus, info *rclgo.MessageInfo, err error)

// NewYawEstimatorStatusSubscription creates and returns a new subscription for the
// YawEstimatorStatus
func NewYawEstimatorStatusSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback YawEstimatorStatusSubscriptionCallback) (*YawEstimatorStatusSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg YawEstimatorStatus
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, YawEstimatorStatusTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &YawEstimatorStatusSubscription{sub}, nil
}

func (s *YawEstimatorStatusSubscription) TakeMessage(out *YawEstimatorStatus) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneYawEstimatorStatusSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneYawEstimatorStatusSlice(dst, src []YawEstimatorStatus) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var YawEstimatorStatusTypeSupport types.MessageTypeSupport = _YawEstimatorStatusTypeSupport{}

type _YawEstimatorStatusTypeSupport struct{}

func (t _YawEstimatorStatusTypeSupport) New() types.Message {
	return NewYawEstimatorStatus()
}

func (t _YawEstimatorStatusTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__YawEstimatorStatus
	return (unsafe.Pointer)(C.px4_msgs__msg__YawEstimatorStatus__create())
}

func (t _YawEstimatorStatusTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__YawEstimatorStatus__destroy((*C.px4_msgs__msg__YawEstimatorStatus)(pointer_to_free))
}

func (t _YawEstimatorStatusTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*YawEstimatorStatus)
	mem := (*C.px4_msgs__msg__YawEstimatorStatus)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	mem.yaw_composite = C.float(m.YawComposite)
	mem.yaw_variance = C.float(m.YawVariance)
	mem.yaw_composite_valid = C.bool(m.YawCompositeValid)
	cSlice_yaw := mem.yaw[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_yaw)), m.Yaw[:])
	cSlice_innov_vn := mem.innov_vn[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov_vn)), m.InnovVn[:])
	cSlice_innov_ve := mem.innov_ve[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov_ve)), m.InnovVe[:])
	cSlice_weight := mem.weight[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_weight)), m.Weight[:])
}

func (t _YawEstimatorStatusTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*YawEstimatorStatus)
	mem := (*C.px4_msgs__msg__YawEstimatorStatus)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	m.YawComposite = float32(mem.yaw_composite)
	m.YawVariance = float32(mem.yaw_variance)
	m.YawCompositeValid = bool(mem.yaw_composite_valid)
	cSlice_yaw := mem.yaw[:]
	primitives.Float32__Array_to_Go(m.Yaw[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_yaw)))
	cSlice_innov_vn := mem.innov_vn[:]
	primitives.Float32__Array_to_Go(m.InnovVn[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov_vn)))
	cSlice_innov_ve := mem.innov_ve[:]
	primitives.Float32__Array_to_Go(m.InnovVe[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_innov_ve)))
	cSlice_weight := mem.weight[:]
	primitives.Float32__Array_to_Go(m.Weight[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_weight)))
}

func (t _YawEstimatorStatusTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__YawEstimatorStatus())
}

type CYawEstimatorStatus = C.px4_msgs__msg__YawEstimatorStatus
type CYawEstimatorStatus__Sequence = C.px4_msgs__msg__YawEstimatorStatus__Sequence

func YawEstimatorStatus__Sequence_to_Go(goSlice *[]YawEstimatorStatus, cSlice CYawEstimatorStatus__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]YawEstimatorStatus, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		YawEstimatorStatusTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func YawEstimatorStatus__Sequence_to_C(cSlice *CYawEstimatorStatus__Sequence, goSlice []YawEstimatorStatus) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__YawEstimatorStatus)(C.malloc(C.sizeof_struct_px4_msgs__msg__YawEstimatorStatus * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		YawEstimatorStatusTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func YawEstimatorStatus__Array_to_Go(goSlice []YawEstimatorStatus, cSlice []CYawEstimatorStatus) {
	for i := 0; i < len(cSlice); i++ {
		YawEstimatorStatusTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func YawEstimatorStatus__Array_to_C(cSlice []CYawEstimatorStatus, goSlice []YawEstimatorStatus) {
	for i := 0; i < len(goSlice); i++ {
		YawEstimatorStatusTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
