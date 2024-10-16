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

#include <px4_msgs/msg/collision_report.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/CollisionReport", CollisionReportTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/CollisionReport", CollisionReportTypeSupport)
}

type CollisionReport struct {
	Timestamp              uint64  `yaml:"timestamp"` // time since system start (microseconds)
	Src                    uint8   `yaml:"src"`
	Id                     uint32  `yaml:"id"`
	Action                 uint8   `yaml:"action"`
	ThreatLevel            uint8   `yaml:"threat_level"`
	TimeToMinimumDelta     float32 `yaml:"time_to_minimum_delta"`
	AltitudeMinimumDelta   float32 `yaml:"altitude_minimum_delta"`
	HorizontalMinimumDelta float32 `yaml:"horizontal_minimum_delta"`
}

// NewCollisionReport creates a new CollisionReport with default values.
func NewCollisionReport() *CollisionReport {
	self := CollisionReport{}
	self.SetDefaults()
	return &self
}

func (t *CollisionReport) Clone() *CollisionReport {
	c := &CollisionReport{}
	c.Timestamp = t.Timestamp
	c.Src = t.Src
	c.Id = t.Id
	c.Action = t.Action
	c.ThreatLevel = t.ThreatLevel
	c.TimeToMinimumDelta = t.TimeToMinimumDelta
	c.AltitudeMinimumDelta = t.AltitudeMinimumDelta
	c.HorizontalMinimumDelta = t.HorizontalMinimumDelta
	return c
}

func (t *CollisionReport) CloneMsg() types.Message {
	return t.Clone()
}

func (t *CollisionReport) SetDefaults() {
	t.Timestamp = 0
	t.Src = 0
	t.Id = 0
	t.Action = 0
	t.ThreatLevel = 0
	t.TimeToMinimumDelta = 0
	t.AltitudeMinimumDelta = 0
	t.HorizontalMinimumDelta = 0
}

func (t *CollisionReport) GetTypeSupport() types.MessageTypeSupport {
	return CollisionReportTypeSupport
}

// CollisionReportPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type CollisionReportPublisher struct {
	*rclgo.Publisher
}

// NewCollisionReportPublisher creates and returns a new publisher for the
// CollisionReport
func NewCollisionReportPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*CollisionReportPublisher, error) {
	pub, err := node.NewPublisher(topic_name, CollisionReportTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &CollisionReportPublisher{pub}, nil
}

func (p *CollisionReportPublisher) Publish(msg *CollisionReport) error {
	return p.Publisher.Publish(msg)
}

// CollisionReportSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type CollisionReportSubscription struct {
	*rclgo.Subscription
}

// CollisionReportSubscriptionCallback type is used to provide a subscription
// handler function for a CollisionReportSubscription.
type CollisionReportSubscriptionCallback func(msg *CollisionReport, info *rclgo.MessageInfo, err error)

// NewCollisionReportSubscription creates and returns a new subscription for the
// CollisionReport
func NewCollisionReportSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback CollisionReportSubscriptionCallback) (*CollisionReportSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg CollisionReport
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, CollisionReportTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &CollisionReportSubscription{sub}, nil
}

func (s *CollisionReportSubscription) TakeMessage(out *CollisionReport) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneCollisionReportSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneCollisionReportSlice(dst, src []CollisionReport) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var CollisionReportTypeSupport types.MessageTypeSupport = _CollisionReportTypeSupport{}

type _CollisionReportTypeSupport struct{}

func (t _CollisionReportTypeSupport) New() types.Message {
	return NewCollisionReport()
}

func (t _CollisionReportTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__CollisionReport
	return (unsafe.Pointer)(C.px4_msgs__msg__CollisionReport__create())
}

func (t _CollisionReportTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__CollisionReport__destroy((*C.px4_msgs__msg__CollisionReport)(pointer_to_free))
}

func (t _CollisionReportTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*CollisionReport)
	mem := (*C.px4_msgs__msg__CollisionReport)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.src = C.uint8_t(m.Src)
	mem.id = C.uint32_t(m.Id)
	mem.action = C.uint8_t(m.Action)
	mem.threat_level = C.uint8_t(m.ThreatLevel)
	mem.time_to_minimum_delta = C.float(m.TimeToMinimumDelta)
	mem.altitude_minimum_delta = C.float(m.AltitudeMinimumDelta)
	mem.horizontal_minimum_delta = C.float(m.HorizontalMinimumDelta)
}

func (t _CollisionReportTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*CollisionReport)
	mem := (*C.px4_msgs__msg__CollisionReport)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.Src = uint8(mem.src)
	m.Id = uint32(mem.id)
	m.Action = uint8(mem.action)
	m.ThreatLevel = uint8(mem.threat_level)
	m.TimeToMinimumDelta = float32(mem.time_to_minimum_delta)
	m.AltitudeMinimumDelta = float32(mem.altitude_minimum_delta)
	m.HorizontalMinimumDelta = float32(mem.horizontal_minimum_delta)
}

func (t _CollisionReportTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__CollisionReport())
}

type CCollisionReport = C.px4_msgs__msg__CollisionReport
type CCollisionReport__Sequence = C.px4_msgs__msg__CollisionReport__Sequence

func CollisionReport__Sequence_to_Go(goSlice *[]CollisionReport, cSlice CCollisionReport__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]CollisionReport, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		CollisionReportTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func CollisionReport__Sequence_to_C(cSlice *CCollisionReport__Sequence, goSlice []CollisionReport) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__CollisionReport)(C.malloc(C.sizeof_struct_px4_msgs__msg__CollisionReport * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		CollisionReportTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func CollisionReport__Array_to_Go(goSlice []CollisionReport, cSlice []CCollisionReport) {
	for i := 0; i < len(cSlice); i++ {
		CollisionReportTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func CollisionReport__Array_to_C(cSlice []CCollisionReport, goSlice []CollisionReport) {
	for i := 0; i < len(goSlice); i++ {
		CollisionReportTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
