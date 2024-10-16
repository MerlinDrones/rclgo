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

#include <px4_msgs/msg/camera_capture.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/CameraCapture", CameraCaptureTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/CameraCapture", CameraCaptureTypeSupport)
}

type CameraCapture struct {
	Timestamp      uint64     `yaml:"timestamp"`       // time since system start (microseconds)
	TimestampUtc   uint64     `yaml:"timestamp_utc"`   // Capture time in UTC / GPS time
	Seq            uint32     `yaml:"seq"`             // Image sequence number
	Lat            float64    `yaml:"lat"`             // Latitude in degrees (WGS84)
	Lon            float64    `yaml:"lon"`             // Longitude in degrees (WGS84)
	Alt            float32    `yaml:"alt"`             // Altitude (AMSL)
	GroundDistance float32    `yaml:"ground_distance"` // Altitude above ground (meters)
	Q              [4]float32 `yaml:"q"`               // Attitude of the camera relative to NED earth-fixed frame when using a gimbal, otherwise vehicle attitude
	Result         int8       `yaml:"result"`          // 1 for success, 0 for failure, -1 if camera does not provide feedback
}

// NewCameraCapture creates a new CameraCapture with default values.
func NewCameraCapture() *CameraCapture {
	self := CameraCapture{}
	self.SetDefaults()
	return &self
}

func (t *CameraCapture) Clone() *CameraCapture {
	c := &CameraCapture{}
	c.Timestamp = t.Timestamp
	c.TimestampUtc = t.TimestampUtc
	c.Seq = t.Seq
	c.Lat = t.Lat
	c.Lon = t.Lon
	c.Alt = t.Alt
	c.GroundDistance = t.GroundDistance
	c.Q = t.Q
	c.Result = t.Result
	return c
}

func (t *CameraCapture) CloneMsg() types.Message {
	return t.Clone()
}

func (t *CameraCapture) SetDefaults() {
	t.Timestamp = 0
	t.TimestampUtc = 0
	t.Seq = 0
	t.Lat = 0
	t.Lon = 0
	t.Alt = 0
	t.GroundDistance = 0
	t.Q = [4]float32{}
	t.Result = 0
}

func (t *CameraCapture) GetTypeSupport() types.MessageTypeSupport {
	return CameraCaptureTypeSupport
}

// CameraCapturePublisher wraps rclgo.Publisher to provide type safe helper
// functions
type CameraCapturePublisher struct {
	*rclgo.Publisher
}

// NewCameraCapturePublisher creates and returns a new publisher for the
// CameraCapture
func NewCameraCapturePublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*CameraCapturePublisher, error) {
	pub, err := node.NewPublisher(topic_name, CameraCaptureTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &CameraCapturePublisher{pub}, nil
}

func (p *CameraCapturePublisher) Publish(msg *CameraCapture) error {
	return p.Publisher.Publish(msg)
}

// CameraCaptureSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type CameraCaptureSubscription struct {
	*rclgo.Subscription
}

// CameraCaptureSubscriptionCallback type is used to provide a subscription
// handler function for a CameraCaptureSubscription.
type CameraCaptureSubscriptionCallback func(msg *CameraCapture, info *rclgo.MessageInfo, err error)

// NewCameraCaptureSubscription creates and returns a new subscription for the
// CameraCapture
func NewCameraCaptureSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback CameraCaptureSubscriptionCallback) (*CameraCaptureSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg CameraCapture
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, CameraCaptureTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &CameraCaptureSubscription{sub}, nil
}

func (s *CameraCaptureSubscription) TakeMessage(out *CameraCapture) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneCameraCaptureSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneCameraCaptureSlice(dst, src []CameraCapture) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var CameraCaptureTypeSupport types.MessageTypeSupport = _CameraCaptureTypeSupport{}

type _CameraCaptureTypeSupport struct{}

func (t _CameraCaptureTypeSupport) New() types.Message {
	return NewCameraCapture()
}

func (t _CameraCaptureTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__CameraCapture
	return (unsafe.Pointer)(C.px4_msgs__msg__CameraCapture__create())
}

func (t _CameraCaptureTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__CameraCapture__destroy((*C.px4_msgs__msg__CameraCapture)(pointer_to_free))
}

func (t _CameraCaptureTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*CameraCapture)
	mem := (*C.px4_msgs__msg__CameraCapture)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_utc = C.uint64_t(m.TimestampUtc)
	mem.seq = C.uint32_t(m.Seq)
	mem.lat = C.double(m.Lat)
	mem.lon = C.double(m.Lon)
	mem.alt = C.float(m.Alt)
	mem.ground_distance = C.float(m.GroundDistance)
	cSlice_q := mem.q[:]
	primitives.Float32__Array_to_C(*(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_q)), m.Q[:])
	mem.result = C.int8_t(m.Result)
}

func (t _CameraCaptureTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*CameraCapture)
	mem := (*C.px4_msgs__msg__CameraCapture)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampUtc = uint64(mem.timestamp_utc)
	m.Seq = uint32(mem.seq)
	m.Lat = float64(mem.lat)
	m.Lon = float64(mem.lon)
	m.Alt = float32(mem.alt)
	m.GroundDistance = float32(mem.ground_distance)
	cSlice_q := mem.q[:]
	primitives.Float32__Array_to_Go(m.Q[:], *(*[]primitives.CFloat32)(unsafe.Pointer(&cSlice_q)))
	m.Result = int8(mem.result)
}

func (t _CameraCaptureTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__CameraCapture())
}

type CCameraCapture = C.px4_msgs__msg__CameraCapture
type CCameraCapture__Sequence = C.px4_msgs__msg__CameraCapture__Sequence

func CameraCapture__Sequence_to_Go(goSlice *[]CameraCapture, cSlice CCameraCapture__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]CameraCapture, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		CameraCaptureTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func CameraCapture__Sequence_to_C(cSlice *CCameraCapture__Sequence, goSlice []CameraCapture) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__CameraCapture)(C.malloc(C.sizeof_struct_px4_msgs__msg__CameraCapture * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		CameraCaptureTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func CameraCapture__Array_to_Go(goSlice []CameraCapture, cSlice []CCameraCapture) {
	for i := 0; i < len(cSlice); i++ {
		CameraCaptureTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func CameraCapture__Array_to_C(cSlice []CCameraCapture, goSlice []CameraCapture) {
	for i := 0; i < len(goSlice); i++ {
		CameraCaptureTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
