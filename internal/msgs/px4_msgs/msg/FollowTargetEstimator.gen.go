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

#include <px4_msgs/msg/follow_target_estimator.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/FollowTargetEstimator", FollowTargetEstimatorTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/FollowTargetEstimator", FollowTargetEstimatorTypeSupport)
}

type FollowTargetEstimator struct {
	Timestamp                uint64     `yaml:"timestamp"`                   // time since system start (microseconds)
	LastFilterResetTimestamp uint64     `yaml:"last_filter_reset_timestamp"` // time of last filter reset (microseconds)
	Valid                    bool       `yaml:"valid"`                       // True if estimator states are okay to be used
	Stale                    bool       `yaml:"stale"`                       // True if estimator stopped receiving follow_target messages for some time. The estimate can still be valid, though it might be inaccurate.
	LatEst                   float64    `yaml:"lat_est"`                     // Estimated target latitude
	LonEst                   float64    `yaml:"lon_est"`                     // Estimated target longitude
	AltEst                   float32    `yaml:"alt_est"`                     // Estimated target altitude
	PosEst                   [3]float32 `yaml:"pos_est"`                     // Estimated target NED position (m)
	VelEst                   [3]float32 `yaml:"vel_est"`                     // Estimated target NED velocity (m/s)
	AccEst                   [3]float32 `yaml:"acc_est"`                     // Estimated target NED acceleration (m^2/s)
	PredictionCount          uint64     `yaml:"prediction_count"`
	FusionCount              uint64     `yaml:"fusion_count"`
}

// NewFollowTargetEstimator creates a new FollowTargetEstimator with default values.
func NewFollowTargetEstimator() *FollowTargetEstimator {
	self := FollowTargetEstimator{}
	self.SetDefaults()
	return &self
}

func (t *FollowTargetEstimator) Clone() *FollowTargetEstimator {
	c := &FollowTargetEstimator{}
	c.Timestamp = t.Timestamp
	c.LastFilterResetTimestamp = t.LastFilterResetTimestamp
	c.Valid = t.Valid
	c.Stale = t.Stale
	c.LatEst = t.LatEst
	c.LonEst = t.LonEst
	c.AltEst = t.AltEst
	c.PosEst = t.PosEst
	c.VelEst = t.VelEst
	c.AccEst = t.AccEst
	c.PredictionCount = t.PredictionCount
	c.FusionCount = t.FusionCount
	return c
}

func (t *FollowTargetEstimator) CloneMsg() types.Message {
	return t.Clone()
}

func (t *FollowTargetEstimator) SetDefaults() {
	t.Timestamp = 0
	t.LastFilterResetTimestamp = 0
	t.Valid = false
	t.Stale = false
	t.LatEst = 0
	t.LonEst = 0
	t.AltEst = 0
	t.PosEst = [3]float32{}
	t.VelEst = [3]float32{}
	t.AccEst = [3]float32{}
	t.PredictionCount = 0
	t.FusionCount = 0
}

func (t *FollowTargetEstimator) GetTypeSupport() types.MessageTypeSupport {
	return FollowTargetEstimatorTypeSupport
}

// FollowTargetEstimatorPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type FollowTargetEstimatorPublisher struct {
	*rclgo.Publisher
}

// NewFollowTargetEstimatorPublisher creates and returns a new publisher for the
// FollowTargetEstimator
func NewFollowTargetEstimatorPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*FollowTargetEstimatorPublisher, error) {
	pub, err := node.NewPublisher(topic_name, FollowTargetEstimatorTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &FollowTargetEstimatorPublisher{pub}, nil
}

func (p *FollowTargetEstimatorPublisher) Publish(msg *FollowTargetEstimator) error {
	return p.Publisher.Publish(msg)
}

// FollowTargetEstimatorSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type FollowTargetEstimatorSubscription struct {
	*rclgo.Subscription
}

// FollowTargetEstimatorSubscriptionCallback type is used to provide a subscription
// handler function for a FollowTargetEstimatorSubscription.
type FollowTargetEstimatorSubscriptionCallback func(msg *FollowTargetEstimator, info *rclgo.MessageInfo, err error)

// NewFollowTargetEstimatorSubscription creates and returns a new subscription for the
// FollowTargetEstimator
func NewFollowTargetEstimatorSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback FollowTargetEstimatorSubscriptionCallback) (*FollowTargetEstimatorSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg FollowTargetEstimator
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, FollowTargetEstimatorTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &FollowTargetEstimatorSubscription{sub}, nil
}

func (s *FollowTargetEstimatorSubscription) TakeMessage(out *FollowTargetEstimator) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneFollowTargetEstimatorSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneFollowTargetEstimatorSlice(dst, src []FollowTargetEstimator) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var FollowTargetEstimatorTypeSupport types.MessageTypeSupport = _FollowTargetEstimatorTypeSupport{}

type _FollowTargetEstimatorTypeSupport struct{}

func (t _FollowTargetEstimatorTypeSupport) New() types.Message {
	return NewFollowTargetEstimator()
}

func (t _FollowTargetEstimatorTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__FollowTargetEstimator
	return (unsafe.Pointer)(C.px4_msgs__msg__FollowTargetEstimator__create())
}

func (t _FollowTargetEstimatorTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__FollowTargetEstimator__destroy((*C.px4_msgs__msg__FollowTargetEstimator)(pointer_to_free))
}

func (t _FollowTargetEstimatorTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*FollowTargetEstimator)
	mem := (*C.px4_msgs__msg__FollowTargetEstimator)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.last_filter_reset_timestamp = C.uint64_t(m.LastFilterResetTimestamp)
	mem.valid = C.bool(m.Valid)
	mem.stale = C.bool(m.Stale)
	mem.lat_est = C.double(m.LatEst)
	mem.lon_est = C.double(m.LonEst)
	mem.alt_est = C.float(m.AltEst)
	cSlice_pos_est := mem.pos_est[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_pos_est)), m.PosEst[:])
	cSlice_vel_est := mem.vel_est[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_vel_est)), m.VelEst[:])
	cSlice_acc_est := mem.acc_est[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_acc_est)), m.AccEst[:])
	mem.prediction_count = C.uint64_t(m.PredictionCount)
	mem.fusion_count = C.uint64_t(m.FusionCount)
}

func (t _FollowTargetEstimatorTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*FollowTargetEstimator)
	mem := (*C.px4_msgs__msg__FollowTargetEstimator)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.LastFilterResetTimestamp = uint64(mem.last_filter_reset_timestamp)
	m.Valid = bool(mem.valid)
	m.Stale = bool(mem.stale)
	m.LatEst = float64(mem.lat_est)
	m.LonEst = float64(mem.lon_est)
	m.AltEst = float32(mem.alt_est)
	cSlice_pos_est := mem.pos_est[:]
	primitives.Float32__Array_to_Go(m.PosEst[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_pos_est)))
	cSlice_vel_est := mem.vel_est[:]
	primitives.Float32__Array_to_Go(m.VelEst[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_vel_est)))
	cSlice_acc_est := mem.acc_est[:]
	primitives.Float32__Array_to_Go(m.AccEst[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_acc_est)))
	m.PredictionCount = uint64(mem.prediction_count)
	m.FusionCount = uint64(mem.fusion_count)
}

func (t _FollowTargetEstimatorTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__FollowTargetEstimator())
}

type CFollowTargetEstimator = C.px4_msgs__msg__FollowTargetEstimator
type CFollowTargetEstimator__Sequence = C.px4_msgs__msg__FollowTargetEstimator__Sequence

func FollowTargetEstimator__Sequence_to_Go(goSlice *[]FollowTargetEstimator, cSlice CFollowTargetEstimator__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]FollowTargetEstimator, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		FollowTargetEstimatorTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func FollowTargetEstimator__Sequence_to_C(cSlice *CFollowTargetEstimator__Sequence, goSlice []FollowTargetEstimator) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__FollowTargetEstimator)(C.malloc(C.sizeof_struct_px4_msgs__msg__FollowTargetEstimator * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		FollowTargetEstimatorTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func FollowTargetEstimator__Array_to_Go(goSlice []FollowTargetEstimator, cSlice []CFollowTargetEstimator) {
	for i := 0; i < len(cSlice); i++ {
		FollowTargetEstimatorTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func FollowTargetEstimator__Array_to_C(cSlice []CFollowTargetEstimator, goSlice []FollowTargetEstimator) {
	for i := 0; i < len(goSlice); i++ {
		FollowTargetEstimatorTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
