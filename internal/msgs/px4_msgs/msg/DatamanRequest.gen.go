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

#include <px4_msgs/msg/dataman_request.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/DatamanRequest", DatamanRequestTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/DatamanRequest", DatamanRequestTypeSupport)
}

type DatamanRequest struct {
	Timestamp   uint64    `yaml:"timestamp"` // time since system start (microseconds)
	ClientId    uint8     `yaml:"client_id"`
	RequestType uint8     `yaml:"request_type"` // id/read/write/clear
	Item        uint8     `yaml:"item"`         // dm_item_t
	Index       uint32    `yaml:"index"`
	Data        [56]uint8 `yaml:"data"`
	DataLength  uint32    `yaml:"data_length"`
}

// NewDatamanRequest creates a new DatamanRequest with default values.
func NewDatamanRequest() *DatamanRequest {
	self := DatamanRequest{}
	self.SetDefaults()
	return &self
}

func (t *DatamanRequest) Clone() *DatamanRequest {
	c := &DatamanRequest{}
	c.Timestamp = t.Timestamp
	c.ClientId = t.ClientId
	c.RequestType = t.RequestType
	c.Item = t.Item
	c.Index = t.Index
	c.Data = t.Data
	c.DataLength = t.DataLength
	return c
}

func (t *DatamanRequest) CloneMsg() types.Message {
	return t.Clone()
}

func (t *DatamanRequest) SetDefaults() {
	t.Timestamp = 0
	t.ClientId = 0
	t.RequestType = 0
	t.Item = 0
	t.Index = 0
	t.Data = [56]uint8{}
	t.DataLength = 0
}

func (t *DatamanRequest) GetTypeSupport() types.MessageTypeSupport {
	return DatamanRequestTypeSupport
}

// DatamanRequestPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type DatamanRequestPublisher struct {
	*rclgo.Publisher
}

// NewDatamanRequestPublisher creates and returns a new publisher for the
// DatamanRequest
func NewDatamanRequestPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*DatamanRequestPublisher, error) {
	pub, err := node.NewPublisher(topic_name, DatamanRequestTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &DatamanRequestPublisher{pub}, nil
}

func (p *DatamanRequestPublisher) Publish(msg *DatamanRequest) error {
	return p.Publisher.Publish(msg)
}

// DatamanRequestSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type DatamanRequestSubscription struct {
	*rclgo.Subscription
}

// DatamanRequestSubscriptionCallback type is used to provide a subscription
// handler function for a DatamanRequestSubscription.
type DatamanRequestSubscriptionCallback func(msg *DatamanRequest, info *rclgo.MessageInfo, err error)

// NewDatamanRequestSubscription creates and returns a new subscription for the
// DatamanRequest
func NewDatamanRequestSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback DatamanRequestSubscriptionCallback) (*DatamanRequestSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg DatamanRequest
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, DatamanRequestTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &DatamanRequestSubscription{sub}, nil
}

func (s *DatamanRequestSubscription) TakeMessage(out *DatamanRequest) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneDatamanRequestSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneDatamanRequestSlice(dst, src []DatamanRequest) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var DatamanRequestTypeSupport types.MessageTypeSupport = _DatamanRequestTypeSupport{}

type _DatamanRequestTypeSupport struct{}

func (t _DatamanRequestTypeSupport) New() types.Message {
	return NewDatamanRequest()
}

func (t _DatamanRequestTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__DatamanRequest
	return (unsafe.Pointer)(C.px4_msgs__msg__DatamanRequest__create())
}

func (t _DatamanRequestTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__DatamanRequest__destroy((*C.px4_msgs__msg__DatamanRequest)(pointer_to_free))
}

func (t _DatamanRequestTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*DatamanRequest)
	mem := (*C.px4_msgs__msg__DatamanRequest)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.client_id = C.uint8_t(m.ClientId)
	mem.request_type = C.uint8_t(m.RequestType)
	mem.item = C.uint8_t(m.Item)
	mem.index = C.uint32_t(m.Index)
	cSlice_data := mem.data[:]
	primitives.Uint8__Array_to_C(*(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_data)), m.Data[:])
	mem.data_length = C.uint32_t(m.DataLength)
}

func (t _DatamanRequestTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*DatamanRequest)
	mem := (*C.px4_msgs__msg__DatamanRequest)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.ClientId = uint8(mem.client_id)
	m.RequestType = uint8(mem.request_type)
	m.Item = uint8(mem.item)
	m.Index = uint32(mem.index)
	cSlice_data := mem.data[:]
	primitives.Uint8__Array_to_Go(m.Data[:], *(*[]primitives.CUint8)(unsafe.Pointer(&cSlice_data)))
	m.DataLength = uint32(mem.data_length)
}

func (t _DatamanRequestTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__DatamanRequest())
}

type CDatamanRequest = C.px4_msgs__msg__DatamanRequest
type CDatamanRequest__Sequence = C.px4_msgs__msg__DatamanRequest__Sequence

func DatamanRequest__Sequence_to_Go(goSlice *[]DatamanRequest, cSlice CDatamanRequest__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]DatamanRequest, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		DatamanRequestTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func DatamanRequest__Sequence_to_C(cSlice *CDatamanRequest__Sequence, goSlice []DatamanRequest) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__DatamanRequest)(C.malloc(C.sizeof_struct_px4_msgs__msg__DatamanRequest * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		DatamanRequestTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func DatamanRequest__Array_to_Go(goSlice []DatamanRequest, cSlice []CDatamanRequest) {
	for i := 0; i < len(cSlice); i++ {
		DatamanRequestTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func DatamanRequest__Array_to_C(cSlice []CDatamanRequest, goSlice []DatamanRequest) {
	for i := 0; i < len(goSlice); i++ {
		DatamanRequestTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
