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

#include <px4_msgs/msg/esc_report.h>

*/
import "C"

func init() {
	typemap.RegisterMessage("px4_msgs/EscReport", EscReportTypeSupport)
	typemap.RegisterMessage("px4_msgs/msg/EscReport", EscReportTypeSupport)
}

const (
	EscReport_FAILURE_OVER_CURRENT           uint8 = 0  // (1 << 0)
	EscReport_FAILURE_OVER_VOLTAGE           uint8 = 1  // (1 << 1)
	EscReport_FAILURE_MOTOR_OVER_TEMPERATURE uint8 = 2  // (1 << 2)
	EscReport_FAILURE_OVER_RPM               uint8 = 3  // (1 << 3)
	EscReport_FAILURE_INCONSISTENT_CMD       uint8 = 4  // (1 << 4)  Set if ESC received an inconsistent command (i.e out of boundaries)
	EscReport_FAILURE_MOTOR_STUCK            uint8 = 5  // (1 << 5)
	EscReport_FAILURE_GENERIC                uint8 = 6  // (1 << 6)
	EscReport_FAILURE_MOTOR_WARN_TEMPERATURE uint8 = 7  // (1 << 7)
	EscReport_FAILURE_WARN_ESC_TEMPERATURE   uint8 = 8  // (1 << 8)
	EscReport_FAILURE_OVER_ESC_TEMPERATURE   uint8 = 9  // (1 << 9)
	EscReport_ESC_FAILURE_COUNT              uint8 = 10 // Counter - keep it as last element!
)

type EscReport struct {
	Timestamp        uint64  `yaml:"timestamp"`         // time since system start (microseconds)
	EscErrorcount    uint32  `yaml:"esc_errorcount"`    // Number of reported errors by ESC - if supported
	EscRpm           int32   `yaml:"esc_rpm"`           // Motor RPM, negative for reverse rotation [RPM] - if supported
	EscVoltage       float32 `yaml:"esc_voltage"`       // Voltage measured from current ESC [V] - if supported
	EscCurrent       float32 `yaml:"esc_current"`       // Current measured from current ESC [A] - if supported
	EscTemperature   float32 `yaml:"esc_temperature"`   // Temperature measured from current ESC [degC] - if supported
	EscAddress       uint8   `yaml:"esc_address"`       // Address of current ESC (in most cases 1-8 / must be set by driver)
	EscCmdcount      uint8   `yaml:"esc_cmdcount"`      // Counter of number of commands
	EscState         uint8   `yaml:"esc_state"`         // State of ESC - depend on Vendor
	ActuatorFunction uint8   `yaml:"actuator_function"` // actuator output function (one of Motor1...MotorN)
	Failures         uint16  `yaml:"failures"`          // Bitmask to indicate the internal ESC faults
	EscPower         int8    `yaml:"esc_power"`         // Applied power 0-100 in % (negative values reserved)
}

// NewEscReport creates a new EscReport with default values.
func NewEscReport() *EscReport {
	self := EscReport{}
	self.SetDefaults()
	return &self
}

func (t *EscReport) Clone() *EscReport {
	c := &EscReport{}
	c.Timestamp = t.Timestamp
	c.EscErrorcount = t.EscErrorcount
	c.EscRpm = t.EscRpm
	c.EscVoltage = t.EscVoltage
	c.EscCurrent = t.EscCurrent
	c.EscTemperature = t.EscTemperature
	c.EscAddress = t.EscAddress
	c.EscCmdcount = t.EscCmdcount
	c.EscState = t.EscState
	c.ActuatorFunction = t.ActuatorFunction
	c.Failures = t.Failures
	c.EscPower = t.EscPower
	return c
}

func (t *EscReport) CloneMsg() types.Message {
	return t.Clone()
}

func (t *EscReport) SetDefaults() {
	t.Timestamp = 0
	t.EscErrorcount = 0
	t.EscRpm = 0
	t.EscVoltage = 0
	t.EscCurrent = 0
	t.EscTemperature = 0
	t.EscAddress = 0
	t.EscCmdcount = 0
	t.EscState = 0
	t.ActuatorFunction = 0
	t.Failures = 0
	t.EscPower = 0
}

func (t *EscReport) GetTypeSupport() types.MessageTypeSupport {
	return EscReportTypeSupport
}

// EscReportPublisher wraps rclgo.Publisher to provide type safe helper
// functions
type EscReportPublisher struct {
	*rclgo.Publisher
}

// NewEscReportPublisher creates and returns a new publisher for the
// EscReport
func NewEscReportPublisher(node *rclgo.Node, topic_name string, options *rclgo.PublisherOptions) (*EscReportPublisher, error) {
	pub, err := node.NewPublisher(topic_name, EscReportTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &EscReportPublisher{pub}, nil
}

func (p *EscReportPublisher) Publish(msg *EscReport) error {
	return p.Publisher.Publish(msg)
}

// EscReportSubscription wraps rclgo.Subscription to provide type safe helper
// functions
type EscReportSubscription struct {
	*rclgo.Subscription
}

// EscReportSubscriptionCallback type is used to provide a subscription
// handler function for a EscReportSubscription.
type EscReportSubscriptionCallback func(msg *EscReport, info *rclgo.MessageInfo, err error)

// NewEscReportSubscription creates and returns a new subscription for the
// EscReport
func NewEscReportSubscription(node *rclgo.Node, topic_name string, opts *rclgo.SubscriptionOptions, subscriptionCallback EscReportSubscriptionCallback) (*EscReportSubscription, error) {
	callback := func(s *rclgo.Subscription) {
		var msg EscReport
		info, err := s.TakeMessage(&msg)
		subscriptionCallback(&msg, info, err)
	}
	sub, err := node.NewSubscription(topic_name, EscReportTypeSupport, opts, callback)
	if err != nil {
		return nil, err
	}
	return &EscReportSubscription{sub}, nil
}

func (s *EscReportSubscription) TakeMessage(out *EscReport) (*rclgo.MessageInfo, error) {
	return s.Subscription.TakeMessage(out)
}

// CloneEscReportSlice clones src to dst by calling Clone for each element in
// src. Panics if len(dst) < len(src).
func CloneEscReportSlice(dst, src []EscReport) {
	for i := range src {
		dst[i] = *src[i].Clone()
	}
}

// Modifying this variable is undefined behavior.
var EscReportTypeSupport types.MessageTypeSupport = _EscReportTypeSupport{}

type _EscReportTypeSupport struct{}

func (t _EscReportTypeSupport) New() types.Message {
	return NewEscReport()
}

func (t _EscReportTypeSupport) PrepareMemory() unsafe.Pointer { //returns *C.px4_msgs__msg__EscReport
	return (unsafe.Pointer)(C.px4_msgs__msg__EscReport__create())
}

func (t _EscReportTypeSupport) ReleaseMemory(pointer_to_free unsafe.Pointer) {
	C.px4_msgs__msg__EscReport__destroy((*C.px4_msgs__msg__EscReport)(pointer_to_free))
}

func (t _EscReportTypeSupport) AsCStruct(dst unsafe.Pointer, msg types.Message) {
	m := msg.(*EscReport)
	mem := (*C.px4_msgs__msg__EscReport)(dst)
	mem.timestamp = C.uint64_t(m.Timestamp)
	mem.esc_errorcount = C.uint32_t(m.EscErrorcount)
	mem.esc_rpm = C.int32_t(m.EscRpm)
	mem.esc_voltage = C.float(m.EscVoltage)
	mem.esc_current = C.float(m.EscCurrent)
	mem.esc_temperature = C.float(m.EscTemperature)
	mem.esc_address = C.uint8_t(m.EscAddress)
	mem.esc_cmdcount = C.uint8_t(m.EscCmdcount)
	mem.esc_state = C.uint8_t(m.EscState)
	mem.actuator_function = C.uint8_t(m.ActuatorFunction)
	mem.failures = C.uint16_t(m.Failures)
	mem.esc_power = C.int8_t(m.EscPower)
}

func (t _EscReportTypeSupport) AsGoStruct(msg types.Message, ros2_message_buffer unsafe.Pointer) {
	m := msg.(*EscReport)
	mem := (*C.px4_msgs__msg__EscReport)(ros2_message_buffer)
	m.Timestamp = uint64(mem.timestamp)
	m.EscErrorcount = uint32(mem.esc_errorcount)
	m.EscRpm = int32(mem.esc_rpm)
	m.EscVoltage = float32(mem.esc_voltage)
	m.EscCurrent = float32(mem.esc_current)
	m.EscTemperature = float32(mem.esc_temperature)
	m.EscAddress = uint8(mem.esc_address)
	m.EscCmdcount = uint8(mem.esc_cmdcount)
	m.EscState = uint8(mem.esc_state)
	m.ActuatorFunction = uint8(mem.actuator_function)
	m.Failures = uint16(mem.failures)
	m.EscPower = int8(mem.esc_power)
}

func (t _EscReportTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__EscReport())
}

type CEscReport = C.px4_msgs__msg__EscReport
type CEscReport__Sequence = C.px4_msgs__msg__EscReport__Sequence

func EscReport__Sequence_to_Go(goSlice *[]EscReport, cSlice CEscReport__Sequence) {
	if cSlice.size == 0 {
		return
	}
	*goSlice = make([]EscReport, cSlice.size)
	src := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range src {
		EscReportTypeSupport.AsGoStruct(&(*goSlice)[i], unsafe.Pointer(&src[i]))
	}
}
func EscReport__Sequence_to_C(cSlice *CEscReport__Sequence, goSlice []EscReport) {
	if len(goSlice) == 0 {
		cSlice.data = nil
		cSlice.capacity = 0
		cSlice.size = 0
		return
	}
	cSlice.data = (*C.px4_msgs__msg__EscReport)(C.malloc(C.sizeof_struct_px4_msgs__msg__EscReport * C.size_t(len(goSlice))))
	cSlice.capacity = C.size_t(len(goSlice))
	cSlice.size = cSlice.capacity
	dst := unsafe.Slice(cSlice.data, cSlice.size)
	for i := range goSlice {
		EscReportTypeSupport.AsCStruct(unsafe.Pointer(&dst[i]), &goSlice[i])
	}
}
func EscReport__Array_to_Go(goSlice []EscReport, cSlice []CEscReport) {
	for i := 0; i < len(cSlice); i++ {
		EscReportTypeSupport.AsGoStruct(&goSlice[i], unsafe.Pointer(&cSlice[i]))
	}
}
func EscReport__Array_to_C(cSlice []CEscReport, goSlice []EscReport) {
	for i := 0; i < len(goSlice); i++ {
		EscReportTypeSupport.AsCStruct(unsafe.Pointer(&cSlice[i]), &goSlice[i])
	}
}
