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

#include <px4_msgs/msg/airspeed_wind.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/AirspeedWind", AirspeedWindTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/AirspeedWind", AirspeedWindTypeSupport)
}

const (
	AirspeedWind_SOURCE_AS_BETA_ONLY uint8 = 0 // wind estimate only based on synthetic sideslip fusion
	AirspeedWind_SOURCE_AS_SENSOR_1  uint8 = 1 // combined synthetic sideslip and airspeed fusion (data from first airspeed sensor)
	AirspeedWind_SOURCE_AS_SENSOR_2  uint8 = 2 // combined synthetic sideslip and airspeed fusion (data from second airspeed sensor)
	AirspeedWind_SOURCE_AS_SENSOR_3  uint8 = 3 // combined synthetic sideslip and airspeed fusion (data from third airspeed sensor)
)

type AirspeedWind struct {
	Timestamp         uint64  `yaml:"timestamp"`           // time since system start (microseconds)
	TimestampSample   uint64  `yaml:"timestamp_sample"`    // the timestamp of the raw data (microseconds)
	WindspeedNorth    float32 `yaml:"windspeed_north"`     // Wind component in north / X direction (m/sec)
	WindspeedEast     float32 `yaml:"windspeed_east"`      // Wind component in east / Y direction (m/sec)
	VarianceNorth     float32 `yaml:"variance_north"`      // Wind estimate error variance in north / X direction (m/sec)**2 - set to zero (no uncertainty) if not estimated
	VarianceEast      float32 `yaml:"variance_east"`       // Wind estimate error variance in east / Y direction (m/sec)**2 - set to zero (no uncertainty) if not estimated
	TasInnov          float32 `yaml:"tas_innov"`           // True airspeed innovation
	TasInnovVar       float32 `yaml:"tas_innov_var"`       // True airspeed innovation variance
	TasScaleRaw       float32 `yaml:"tas_scale_raw"`       // Estimated true airspeed scale factor (not validated)
	TasScaleRawVar    float32 `yaml:"tas_scale_raw_var"`   // True airspeed scale factor variance
	TasScaleValidated float32 `yaml:"tas_scale_validated"` // Estimated true airspeed scale factor after validation
	BetaInnov         float32 `yaml:"beta_innov"`          // Sideslip measurement innovation
	BetaInnovVar      float32 `yaml:"beta_innov_var"`      // Sideslip measurement innovation variance
	Source            uint8   `yaml:"source"`              // source of wind estimate
}

// NewAirspeedWind creates a new AirspeedWind with default values.
func NewAirspeedWind() *AirspeedWind {
	self := AirspeedWind{}
	self.SetDefaults()
	return &self
}

func (t *AirspeedWind) Clone() *AirspeedWind {
	c := &AirspeedWind{}
	c.Timestamp = t.Timestamp
	c.TimestampSample = t.TimestampSample
	c.WindspeedNorth = t.WindspeedNorth
	c.WindspeedEast = t.WindspeedEast
	c.VarianceNorth = t.VarianceNorth
	c.VarianceEast = t.VarianceEast
	c.TasInnov = t.TasInnov
	c.TasInnovVar = t.TasInnovVar
	c.TasScaleRaw = t.TasScaleRaw
	c.TasScaleRawVar = t.TasScaleRawVar
	c.TasScaleValidated = t.TasScaleValidated
	c.BetaInnov = t.BetaInnov
	c.BetaInnovVar = t.BetaInnovVar
	c.Source = t.Source
	return c
}

func (t *AirspeedWind) CloneMsg() types.Message {
	return t.Clone()
}

func (t *AirspeedWind) SetDefaults() {
	t.Timestamp = 0
	t.TimestampSample = 0
	t.WindspeedNorth = 0
	t.WindspeedEast = 0
	t.VarianceNorth = 0
	t.VarianceEast = 0
	t.TasInnov = 0
	t.TasInnovVar = 0
	t.TasScaleRaw = 0
	t.TasScaleRawVar = 0
	t.TasScaleValidated = 0
	t.BetaInnov = 0
	t.BetaInnovVar = 0
	t.Source = 0
}

func (t *AirspeedWind) GetTypeSupport() types.MessageTypeSupport {
	return AirspeedWindTypeSupport
}

// AirspeedWindPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type AirspeedWindPublisher struct {
	*rclgo.Publisher
}

// NewAirspeedWindPublisher creates and returns a new publisher for the
// AirspeedWind
func NewAirspeedWindPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*AirspeedWindPublisher, error) {
	pub, err := node.NewPublisher(topic_name, AirspeedWindTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &AirspeedWindPublisher{pub}, nil
}

func (p *AirspeedWindPublisher) Publish(msg *AirspeedWind) error {
	return p.Publisher.Publish(msg)
}

// AirspeedWindSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type AirspeedWindSubscription struct {
	*rclgo.Subscription
}

// AirspeedWindSubscriptionCallback type is used to provide a subscription
// handler function for a AirspeedWindSubscription.
type AirspeedWindSubscriptionCallback func(msg *AirspeedWind, info *rclgo.MessageInfo, err error)

// NewAirspeedWindSubscription creates and returns a new subscription for the
// AirspeedWind
func NewAirspeedWindSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback AirspeedWindSubscriptionCallback) (*AirspeedWindSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg AirspeedWind
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, AirspeedWindTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &AirspeedWindSubscription{sub}, nil
}

func (s *AirspeedWindSubscription) TakeMessage(out *AirspeedWind) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneAirspeedWindSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneAirspeedWindSlice(dst, src []AirspeedWind) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var AirspeedWindTypeSupport types.MessageTypeSupport = _AirspeedWindTypeSupport{}

type _AirspeedWindTypeSupport struct{}

func (t _AirspeedWindTypeSupport) New() types.Message {
	return NewAirspeedWind()
}

func (t _AirspeedWindTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__AirspeedWind
	return (unsafe.Pointer)(C.px4_msgs__msg__AirspeedWind__create())
}

func (t _AirspeedWindTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__AirspeedWind__destroy((*C.px4_msgs__msg__AirspeedWind)(pointer_to_free))
}

func (t _AirspeedWindTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*AirspeedWind)
	mem := (*C.px4_msgs__msg__AirspeedWind)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.timestamp_sample = C.uint64_t(m.TimestampSample)
	mem.windspeed_north = C.float(m.WindspeedNorth)
	mem.windspeed_east = C.float(m.WindspeedEast)
	mem.variance_north = C.float(m.VarianceNorth)
	mem.variance_east = C.float(m.VarianceEast)
	mem.tas_innov = C.float(m.TasInnov)
	mem.tas_innov_var = C.float(m.TasInnovVar)
	mem.tas_scale_raw = C.float(m.TasScaleRaw)
	mem.tas_scale_raw_var = C.float(m.TasScaleRawVar)
	mem.tas_scale_validated = C.float(m.TasScaleValidated)
	mem.beta_innov = C.float(m.BetaInnov)
	mem.beta_innov_var = C.float(m.BetaInnovVar)
	mem.source = C.uint8_t(m.Source)
}

func (t _AirspeedWindTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*AirspeedWind)
	mem := (*C.px4_msgs__msg__AirspeedWind)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.TimestampSample = uint64(mem.timestamp_sample)
	m.WindspeedNorth = float32(mem.windspeed_north)
	m.WindspeedEast = float32(mem.windspeed_east)
	m.VarianceNorth = float32(mem.variance_north)
	m.VarianceEast = float32(mem.variance_east)
	m.TasInnov = float32(mem.tas_innov)
	m.TasInnovVar = float32(mem.tas_innov_var)
	m.TasScaleRaw = float32(mem.tas_scale_raw)
	m.TasScaleRawVar = float32(mem.tas_scale_raw_var)
	m.TasScaleValidated = float32(mem.tas_scale_validated)
	m.BetaInnov = float32(mem.beta_innov)
	m.BetaInnovVar = float32(mem.beta_innov_var)
	m.Source = uint8(mem.source)
}

func (t _AirspeedWindTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__AirspeedWind())
}

type CAirspeedWind = C.px4_msgs__msg__AirspeedWind
type CAirspeedWind__Sequence = C.px4_msgs__msg__AirspeedWind__Sequence

func AirspeedWind__Sequence_to_Go(goSlice *[]AirspeedWind, cSlice CAirspeedWind__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]AirspeedWind, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		AirspeedWindTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func AirspeedWind__Sequence_to_C(cSlice *CAirspeedWind__Sequence, goSlice []AirspeedWind) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__AirspeedWind)(C.malloc(C.sizeof_struct_px4_msgs__msg__AirspeedWind * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		AirspeedWindTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func AirspeedWind__Array_to_Go(goSlice []AirspeedWind, cSlice []CAirspeedWind) {
	for i := 0; i < len(cSlice); i++ {
		AirspeedWindTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func AirspeedWind__Array_to_C(cSlice []CAirspeedWind, goSlice []AirspeedWind) {
	for i := 0; i < len(goSlice); i++ {
		AirspeedWindTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
