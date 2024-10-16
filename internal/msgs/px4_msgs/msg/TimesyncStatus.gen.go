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

#include <px4_msgs/msg/timesync_status.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/TimesyncStatus", TimesyncStatusTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/TimesyncStatus", TimesyncStatusTypeSupport)
}

const (
	TimesyncStatus_SOURCE_PROTOCOL_UNKNOWN uint8 = 0
	TimesyncStatus_SOURCE_PROTOCOL_MAVLINK uint8 = 1
	TimesyncStatus_SOURCE_PROTOCOL_DDS     uint8 = 2
)

type TimesyncStatus struct {
	Timestamp       uint64 `yaml:"timestamp"`        // time since system start (microseconds)
	SourceProtocol  uint8  `yaml:"source_protocol"`  // timesync source
	RemoteTimestamp uint64 `yaml:"remote_timestamp"` // remote system timestamp (microseconds)
	ObservedOffset  int64  `yaml:"observed_offset"`  // raw time offset directly observed from this timesync packet (microseconds)
	EstimatedOffset int64  `yaml:"estimated_offset"` // smoothed time offset between companion system and PX4 (microseconds)
	RoundTripTime   uint32 `yaml:"round_trip_time"`  // round trip time of this timesync packet (microseconds)
}

// NewTimesyncStatus creates a new TimesyncStatus with default values.
func NewTimesyncStatus() *TimesyncStatus {
	self := TimesyncStatus{}
	self.SetDefaults()
	return &self
}

func (t *TimesyncStatus) Clone() *TimesyncStatus {
	c := &TimesyncStatus{}
	c.Timestamp = t.Timestamp
	c.SourceProtocol = t.SourceProtocol
	c.RemoteTimestamp = t.RemoteTimestamp
	c.ObservedOffset = t.ObservedOffset
	c.EstimatedOffset = t.EstimatedOffset
	c.RoundTripTime = t.RoundTripTime
	return c
}

func (t *TimesyncStatus) CloneMsg() types.Message {
	return t.Clone()
}

func (t *TimesyncStatus) SetDefaults() {
	t.Timestamp = 0
	t.SourceProtocol = 0
	t.RemoteTimestamp = 0
	t.ObservedOffset = 0
	t.EstimatedOffset = 0
	t.RoundTripTime = 0
}

func (t *TimesyncStatus) GetTypeSupport() types.MessageTypeSupport {
	return TimesyncStatusTypeSupport
}

// TimesyncStatusPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type TimesyncStatusPublisher struct {
	*rclgo.Publisher
}

// NewTimesyncStatusPublisher creates and returns a new publisher for the
// TimesyncStatus
func NewTimesyncStatusPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*TimesyncStatusPublisher, error) {
	pub, err := node.NewPublisher(topic_name, TimesyncStatusTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &TimesyncStatusPublisher{pub}, nil
}

func (p *TimesyncStatusPublisher) Publish(msg *TimesyncStatus) error {
	return p.Publisher.Publish(msg)
}

// TimesyncStatusSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type TimesyncStatusSubscription struct {
	*rclgo.Subscription
}

// TimesyncStatusSubscriptionCallback type is used to provide a subscription
// handler function for a TimesyncStatusSubscription.
type TimesyncStatusSubscriptionCallback func(msg *TimesyncStatus, info *rclgo.MessageInfo, err error)

// NewTimesyncStatusSubscription creates and returns a new subscription for the
// TimesyncStatus
func NewTimesyncStatusSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback TimesyncStatusSubscriptionCallback) (*TimesyncStatusSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg TimesyncStatus
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, TimesyncStatusTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &TimesyncStatusSubscription{sub}, nil
}

func (s *TimesyncStatusSubscription) TakeMessage(out *TimesyncStatus) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneTimesyncStatusSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneTimesyncStatusSlice(dst, src []TimesyncStatus) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var TimesyncStatusTypeSupport types.MessageTypeSupport = _TimesyncStatusTypeSupport{}

type _TimesyncStatusTypeSupport struct{}

func (t _TimesyncStatusTypeSupport) New() types.Message {
	return NewTimesyncStatus()
}

func (t _TimesyncStatusTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__TimesyncStatus
	return (unsafe.Pointer)(C.px4_msgs__msg__TimesyncStatus__create())
}

func (t _TimesyncStatusTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__TimesyncStatus__destroy((*C.px4_msgs__msg__TimesyncStatus)(pointer_to_free))
}

func (t _TimesyncStatusTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*TimesyncStatus)
	mem := (*C.px4_msgs__msg__TimesyncStatus)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.source_protocol = C.uint8_t(m.SourceProtocol)
	mem.remote_timestamp = C.uint64_t(m.RemoteTimestamp)
	mem.observed_offset = C.int64_t(m.ObservedOffset)
	mem.estimated_offset = C.int64_t(m.EstimatedOffset)
	mem.round_trip_time = C.uint32_t(m.RoundTripTime)
}

func (t _TimesyncStatusTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*TimesyncStatus)
	mem := (*C.px4_msgs__msg__TimesyncStatus)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.SourceProtocol = uint8(mem.source_protocol)
	m.RemoteTimestamp = uint64(mem.remote_timestamp)
	m.ObservedOffset = int64(mem.observed_offset)
	m.EstimatedOffset = int64(mem.estimated_offset)
	m.RoundTripTime = uint32(mem.round_trip_time)
}

func (t _TimesyncStatusTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__TimesyncStatus())
}

type CTimesyncStatus = C.px4_msgs__msg__TimesyncStatus
type CTimesyncStatus__Sequence = C.px4_msgs__msg__TimesyncStatus__Sequence

func TimesyncStatus__Sequence_to_Go(goSlice *[]TimesyncStatus, cSlice CTimesyncStatus__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]TimesyncStatus, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		TimesyncStatusTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func TimesyncStatus__Sequence_to_C(cSlice *CTimesyncStatus__Sequence, goSlice []TimesyncStatus) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__TimesyncStatus)(C.malloc(C.sizeof_struct_px4_msgs__msg__TimesyncStatus * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		TimesyncStatusTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func TimesyncStatus__Array_to_Go(goSlice []TimesyncStatus, cSlice []CTimesyncStatus) {
	for i := 0; i < len(cSlice); i++ {
		TimesyncStatusTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func TimesyncStatus__Array_to_C(cSlice []CTimesyncStatus, goSlice []TimesyncStatus) {
	for i := 0; i < len(goSlice); i++ {
		TimesyncStatusTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
