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

#include <px4_msgs/msg/config_overrides.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/ConfigOverrides", ConfigOverridesTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/ConfigOverrides", ConfigOverridesTypeSupport)
}

const (
	ConfigOverrides_SOURCE_TYPE_MODE          int8  = 0
	ConfigOverrides_SOURCE_TYPE_MODE_EXECUTOR int8  = 1
	ConfigOverrides_ORB_QUEUE_LENGTH          uint8 = 4
)

type ConfigOverrides struct {
	Timestamp              uint64 `yaml:"timestamp"`                 // time since system start (microseconds). Configurable overrides by (external) modes or mode executors
	DisableAutoDisarm      bool   `yaml:"disable_auto_disarm"`       // Prevent the drone from automatically disarming after landing (if configured)
	DeferFailsafes         bool   `yaml:"defer_failsafes"`           // Defer all failsafes that can be deferred (until the flag is cleared)
	DeferFailsafesTimeoutS int16  `yaml:"defer_failsafes_timeout_s"` // Maximum time a failsafe can be deferred. 0 = system default, -1 = no timeout
	SourceType             int8   `yaml:"source_type"`
	SourceId               uint8  `yaml:"source_id"` // ID depending on source_type
}

// NewConfigOverrides creates a new ConfigOverrides with default values.
func NewConfigOverrides() *ConfigOverrides {
	self := ConfigOverrides{}
	self.SetDefaults()
	return &self
}

func (t *ConfigOverrides) Clone() *ConfigOverrides {
	c := &ConfigOverrides{}
	c.Timestamp = t.Timestamp
	c.DisableAutoDisarm = t.DisableAutoDisarm
	c.DeferFailsafes = t.DeferFailsafes
	c.DeferFailsafesTimeoutS = t.DeferFailsafesTimeoutS
	c.SourceType = t.SourceType
	c.SourceId = t.SourceId
	return c
}

func (t *ConfigOverrides) CloneMsg() types.Message {
	return t.Clone()
}

func (t *ConfigOverrides) SetDefaults() {
	t.Timestamp = 0
	t.DisableAutoDisarm = false
	t.DeferFailsafes = false
	t.DeferFailsafesTimeoutS = 0
	t.SourceType = 0
	t.SourceId = 0
}

func (t *ConfigOverrides) GetTypeSupport() types.MessageTypeSupport {
	return ConfigOverridesTypeSupport
}

// ConfigOverridesPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type ConfigOverridesPublisher struct {
	*rclgo.Publisher
}

// NewConfigOverridesPublisher creates and returns a new publisher for the
// ConfigOverrides
func NewConfigOverridesPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*ConfigOverridesPublisher, error) {
	pub, err := node.NewPublisher(topic_name, ConfigOverridesTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &ConfigOverridesPublisher{pub}, nil
}

func (p *ConfigOverridesPublisher) Publish(msg *ConfigOverrides) error {
	return p.Publisher.Publish(msg)
}

// ConfigOverridesSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type ConfigOverridesSubscription struct {
	*rclgo.Subscription
}

// ConfigOverridesSubscriptionCallback type is used to provide a subscription
// handler function for a ConfigOverridesSubscription.
type ConfigOverridesSubscriptionCallback func(msg *ConfigOverrides, info *rclgo.MessageInfo, err error)

// NewConfigOverridesSubscription creates and returns a new subscription for the
// ConfigOverrides
func NewConfigOverridesSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback ConfigOverridesSubscriptionCallback) (*ConfigOverridesSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg ConfigOverrides
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, ConfigOverridesTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &ConfigOverridesSubscription{sub}, nil
}

func (s *ConfigOverridesSubscription) TakeMessage(out *ConfigOverrides) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneConfigOverridesSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneConfigOverridesSlice(dst, src []ConfigOverrides) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var ConfigOverridesTypeSupport types.MessageTypeSupport = _ConfigOverridesTypeSupport{}

type _ConfigOverridesTypeSupport struct{}

func (t _ConfigOverridesTypeSupport) New() types.Message {
	return NewConfigOverrides()
}

func (t _ConfigOverridesTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__ConfigOverrides
	return (unsafe.Pointer)(C.px4_msgs__msg__ConfigOverrides__create())
}

func (t _ConfigOverridesTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__ConfigOverrides__destroy((*C.px4_msgs__msg__ConfigOverrides)(pointer_to_free))
}

func (t _ConfigOverridesTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*ConfigOverrides)
	mem := (*C.px4_msgs__msg__ConfigOverrides)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.disable_auto_disarm = C.bool(m.DisableAutoDisarm)
	mem.defer_failsafes = C.bool(m.DeferFailsafes)
	mem.defer_failsafes_timeout_s = C.int16_t(m.DeferFailsafesTimeoutS)
	mem.source_type = C.int8_t(m.SourceType)
	mem.source_id = C.uint8_t(m.SourceId)
}

func (t _ConfigOverridesTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*ConfigOverrides)
	mem := (*C.px4_msgs__msg__ConfigOverrides)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.DisableAutoDisarm = bool(mem.disable_auto_disarm)
	m.DeferFailsafes = bool(mem.defer_failsafes)
	m.DeferFailsafesTimeoutS = int16(mem.defer_failsafes_timeout_s)
	m.SourceType = int8(mem.source_type)
	m.SourceId = uint8(mem.source_id)
}

func (t _ConfigOverridesTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__ConfigOverrides())
}

type CConfigOverrides = C.px4_msgs__msg__ConfigOverrides
type CConfigOverrides__Sequence = C.px4_msgs__msg__ConfigOverrides__Sequence

func ConfigOverrides__Sequence_to_Go(goSlice *[]ConfigOverrides, cSlice CConfigOverrides__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]ConfigOverrides, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		ConfigOverridesTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func ConfigOverrides__Sequence_to_C(cSlice *CConfigOverrides__Sequence, goSlice []ConfigOverrides) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__ConfigOverrides)(C.malloc(C.sizeof_struct_px4_msgs__msg__ConfigOverrides * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		ConfigOverridesTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func ConfigOverrides__Array_to_Go(goSlice []ConfigOverrides, cSlice []CConfigOverrides) {
	for i := 0; i < len(cSlice); i++ {
		ConfigOverridesTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func ConfigOverrides__Array_to_C(cSlice []CConfigOverrides, goSlice []ConfigOverrides) {
	for i := 0; i < len(goSlice); i++ {
		ConfigOverridesTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
