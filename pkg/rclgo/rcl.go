/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo

/*
#cgo LDFLAGS: -lrcl_yaml_param_parser
#include <stdlib.h>
#include <string.h>

#include <rcutils/allocator.h>
#include <rcutils/types/string_array.h>
#include <rcl/rcl.h>
#include <rcl/client.h>
#include <rcl/service.h>
#include <rcl/timer.h>
#include <rcl/expand_topic_name.h>
#include <rcl/logging.h>
#include <rcl/logging_rosout.h>
#include <rcl_action/wait.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rmw/rmw.h>

// Cast enums on the C side to avoid cgo typedef mismatches in this package.
static inline void rclgo_qos_set_local(
  rmw_qos_profile_t* dst,
  uint32_t history,
  size_t depth,
  uint32_t reliability,
  uint32_t durability,
  uint64_t deadline_sec, uint64_t deadline_nsec,
  uint64_t lifespan_sec, uint64_t lifespan_nsec,
  uint32_t liveliness,
  uint64_t lease_sec, uint64_t lease_nsec,
  _Bool avoid_ns
) {
  dst->history = (rmw_qos_history_policy_t)history;
  dst->depth = depth;
  dst->reliability = (rmw_qos_reliability_policy_t)reliability;
  dst->durability = (rmw_qos_durability_policy_t)durability;
  dst->deadline.sec = deadline_sec;
  dst->deadline.nsec = deadline_nsec;
  dst->lifespan.sec = lifespan_sec;
  dst->lifespan.nsec = lifespan_nsec;
  dst->liveliness = (rmw_qos_liveliness_policy_t)liveliness;
  dst->liveliness_lease_duration.sec = lease_sec;
  dst->liveliness_lease_duration.nsec = lease_nsec;
  dst->avoid_ros_namespace_conventions = avoid_ns;
}
*/
import "C"

import (
	"context"
	"errors"
	"fmt"
	"reflect"
	"runtime"
	"sync"
	"time"
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

// pkg/rclgo/rcl.go

// pkg/rclgo/rcl.go
// ...
// qosToC writes a qos.Profile into a rmw_qos_profile_t.
// qosToC writes a qos.Profile into a rmw_qos_profile_t.
func qosToC(p qos.Profile, dst *C.rmw_qos_profile_t) {
	deadlineSec, deadlineNsec := qos.DurationToRMW(p.Deadline)
	lifespanSec, lifespanNsec := qos.DurationToRMW(p.Lifespan)
	leaseSec, leaseNsec := qos.DurationToRMW(p.LivelinessLeaseDuration)
	C.rclgo_qos_set_local(
		dst,
		C.uint32_t(p.History),
		C.size_t(p.Depth),
		C.uint32_t(p.Reliability),
		C.uint32_t(p.Durability),
		C.uint64_t(deadlineSec), C.uint64_t(deadlineNsec),
		C.uint64_t(lifespanSec), C.uint64_t(lifespanNsec),
		C.uint32_t(p.Liveliness),
		C.uint64_t(leaseSec), C.uint64_t(leaseNsec),
		C.bool(p.AvoidROSNamespaceConventions),
	)
}

// qosFromC creates a qos.Profile from a rmw_qos_profile_t.
func qosFromC(src *C.rmw_qos_profile_t) qos.Profile {
	return qos.Profile{
		History:                      qos.HistoryPolicy(src.history),
		Depth:                        int(src.depth),
		Reliability:                  qos.ReliabilityPolicy(src.reliability),
		Durability:                   qos.DurabilityPolicy(src.durability),
		Deadline:                     qos.DurationFromRMW(uint64(src.deadline.sec), uint64(src.deadline.nsec)),
		Lifespan:                     qos.DurationFromRMW(uint64(src.lifespan.sec), uint64(src.lifespan.nsec)),
		Liveliness:                   qos.LivelinessPolicy(src.liveliness),
		LivelinessLeaseDuration:      qos.DurationFromRMW(uint64(src.liveliness_lease_duration.sec), uint64(src.liveliness_lease_duration.nsec)),
		AvoidROSNamespaceConventions: bool(src.avoid_ros_namespace_conventions),
	}
}

type MessageInfo struct {
	SourceTimestamp   time.Time
	ReceivedTimestamp time.Time
	FromIntraProcess  bool
}

type ClockType uint32

const (
	ClockTypeUninitialized ClockType = 0
	ClockTypeROSTime       ClockType = 1
	ClockTypeSystemTime    ClockType = 2
	ClockTypeSteadyTime    ClockType = 3
)

// ROS2 is configured via CLI arguments, so merge them from different sources.
// See http://design.ros2.org/articles/ros_command_line_arguments.html for
// details.
type Args struct {
	unparsed []*C.char
	parsed   C.rcl_arguments_t
}

// ParseArgs parses ROS 2 command line arguments from the given slice. Returns
// the parsed ROS 2 arguments and the remaining non-ROS arguments.
//
// ParseArgs expects ROS 2 arguments to be wrapped between a pair of
// "--ros-args" and "--" arguments. See
// http://design.ros2.org/articles/ros_command_line_arguments.html for details.
func ParseArgs(args []string) (*Args, []string, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	rclArgs := &Args{
		unparsed: []*C.char{C.CString("--ros-args")},
		parsed:   C.rcl_get_zero_initialized_arguments(),
	}
	runtime.SetFinalizer(rclArgs, func(a *Args) {
		for _, arg := range a.unparsed {
			C.free(unsafe.Pointer(arg))
		}
		a.unparsed = nil
		C.rcl_arguments_fini(&a.parsed)
	})
	var restArgs []string
	isROSArg := false
	for _, arg := range args {
		if arg == "--ros-args" {
			isROSArg = true
		} else if isROSArg {
			if arg == "--" {
				isROSArg = false
			} else {
				rclArgs.unparsed = append(rclArgs.unparsed, C.CString(arg))
			}
		} else {
			restArgs = append(restArgs, arg)
		}
	}
	rc := C.rcl_parse_arguments(
		rclArgs.argc(),
		rclArgs.argv(),
		C.rcl_get_default_allocator(),
		&rclArgs.parsed,
	)
	if rc != C.RCL_RET_OK {
		return nil, restArgs, errorsCastC(rc, "rcl_parse_arguments")
	}
	return rclArgs, restArgs, nil
}

func (a *Args) argc() C.int {
	return C.int(len(a.unparsed))
}

func (a *Args) argv() **C.char {
	s := (*reflect.SliceHeader)(unsafe.Pointer(&a.unparsed))
	return (**C.char)(unsafe.Pointer(s.Data))
}

// Parsed returns a pointer to the parsed RCL arguments structure.
// This is primarily used internally and by the params package for extracting
// parameter overrides.
func (a *Args) Parsed() unsafe.Pointer {
	return unsafe.Pointer(&a.parsed)
}

// GetParamOverrides safely extracts parameter overrides from ROS arguments.
// This method performs the CGO call here (where Args is defined) to avoid
// CGO pointer safety issues when passing pointers across package boundaries.
// Returns an unsafe.Pointer to rcl_params_t that must be freed by the caller
// using rcl_yaml_node_struct_fini.
func (a *Args) GetParamOverrides() (unsafe.Pointer, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var cparams *C.rcl_params_t
	rc := C.rcl_arguments_get_param_overrides(&a.parsed, &cparams)
	runtime.KeepAlive(a)
	if rc != C.RCL_RET_OK {
		return nil, errorsCastC(rc, "rcl_arguments_get_param_overrides")
	}
	return unsafe.Pointer(cparams), nil
}

func (a *Args) String() string {
	var s []byte
	for i, p := range a.unparsed[1:] {
		if i > 0 {
			s = append(s, ' ')
		}
		for *p != 0 {
			s = append(s, byte(*p))
			p = (*C.char)(unsafe.Add(unsafe.Pointer(p), 1))
		}
	}
	runtime.KeepAlive(a)
	return string(s)
}

type serializedMessage C.rmw_serialized_message_t

func newSerializedMessage(size int) (serializedMessage, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	msg := C.rmw_get_zero_initialized_serialized_message()
	allocator := C.rcl_get_default_allocator()
	rc := C.rcutils_uint8_array_init(&msg, C.size_t(size), &allocator)
	if rc != C.RCL_RET_OK {
		return serializedMessage{}, errorsCastC(rc, "failed to initialize serialized message")
	}
	msg.buffer_length = msg.buffer_capacity
	return serializedMessage(msg), nil
}

func (m *serializedMessage) c() *C.rmw_serialized_message_t {
	return (*C.rmw_serialized_message_t)(m)
}

func (m *serializedMessage) Close() {
	C.rcutils_uint8_array_fini(m.c())
}

func (m *serializedMessage) AsSlice() []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(m.buffer)), m.buffer_length)
}

func (m *serializedMessage) ToSlice() []byte {
	buf := m.AsSlice()
	slice := make([]byte, len(buf))
	copy(slice, buf)
	return slice
}

// Serialize returns the serialized form of msg as a byte slice.
func Serialize(msg types.Message) (buf []byte, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	defer wrapErr("failed to serialize: %v", &err)
	ts := msg.GetTypeSupport()
	cmsg := ts.PrepareMemory()
	defer ts.ReleaseMemory(cmsg)
	ts.AsCStruct(cmsg, msg)
	serialized, err := newSerializedMessage(0)
	if err != nil {
		return nil, err
	}
	defer serialized.Close()
	rc := C.rmw_serialize(
		cmsg,
		(*C.rosidl_message_type_support_t)(ts.TypeSupport()),
		serialized.c(),
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}
	return serialized.ToSlice(), nil
}

// Deserialize deserializes buf to a message whose type support is ts. The
// contents of buf must match ts.
func Deserialize(buf []byte, ts types.MessageTypeSupport) (msg types.Message, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	defer wrapErr("failed to deserialize: %v", &err)
	// Every CDR buffer starts with a 4-byte encapsulation header. Some RMWs
	// (Cyclone DDS on Jazzy) crash on shorter input instead of failing.
	if len(buf) < 4 {
		return nil, fmt.Errorf("buffer of %d bytes is shorter than the 4-byte CDR header", len(buf))
	}
	serialized, err := newSerializedMessage(len(buf))
	if err != nil {
		return nil, err
	}
	defer serialized.Close()
	copy(serialized.AsSlice(), buf)
	cmsg := ts.PrepareMemory()
	defer ts.ReleaseMemory(cmsg)
	rc := C.rmw_deserialize(
		serialized.c(),
		(*C.rosidl_message_type_support_t)(ts.TypeSupport()),
		cmsg,
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}
	gomsg := ts.New()
	ts.AsGoStruct(gomsg, cmsg)
	return gomsg, nil
}

// ExpandTopicName returns inputTopicName expanded to a fully qualified topic
// name.
//
// substitutions may be nil, which is treated the same as an empty substitution
// map.
func ExpandTopicName(
	inputTopicName, nodeName, nodeNamespace string,
	substitutions map[string]string,
) (expanded string, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	csubstitutions := C.rcutils_get_zero_initialized_string_map()
	rc := C.rcutils_string_map_init(
		&csubstitutions,
		C.size_t(len(substitutions)),
		C.rcl_get_default_allocator(),
	)
	if rc != C.RCL_RET_OK {
		return "", errorsCastC(rc, "failed to initialize substitution map")
	}
	defer C.rcutils_string_map_fini(&csubstitutions)
	for key, value := range substitutions {
		ckey := C.CString(key)
		defer C.free(unsafe.Pointer(ckey))
		cvalue := C.CString(value)
		defer C.free(unsafe.Pointer(cvalue))
		rc = C.rcutils_string_map_set(&csubstitutions, ckey, cvalue)
		if rc != C.RCL_RET_OK {
			return "", errorsCastC(rc, "failed to set substitution pair")
		}
	}
	cinputTopicName := C.CString(inputTopicName)
	defer C.free(unsafe.Pointer(cinputTopicName))
	cnodeName := C.CString(nodeName)
	defer C.free(unsafe.Pointer(cnodeName))
	cnodeNamespace := C.CString(nodeNamespace)
	defer C.free(unsafe.Pointer(cnodeNamespace))
	var output *C.char
	rc = C.rcl_expand_topic_name(
		cinputTopicName,
		cnodeName,
		cnodeNamespace,
		&csubstitutions,
		C.rcl_get_default_allocator(),
		&output,
	)
	if rc != C.RCL_RET_OK {
		return "", errorsCastC(rc, "failed to expand topic name")
	}
	defer C.free(unsafe.Pointer(output))
	return C.GoString(output), nil
}

type Node struct {
	rosID
	rosResourceStore
	life               lifetime
	rcl_node_t         *C.rcl_node_t
	context            *Context
	name               string
	namespace          string
	fullyQualifiedName string
	logger             *Logger
	// rosoutInitialized is true when this node created a /rosout log
	// publisher that must be finalized when the node is closed.
	rosoutInitialized bool
}

func NewNode(nodeName, namespace string) (*Node, error) {
	if defaultContext == nil {
		return nil, errInitNotCalled
	}
	return defaultContext.NewNode(nodeName, namespace)
}

func (c *Context) NewNode(node_name, namespace string) (node *Node, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	node = &Node{
		rcl_node_t: (*C.rcl_node_t)(C.malloc(C.sizeof_rcl_node_t)),
		context:    c,
	}
	node.rosResourceStore.onChange = c.resourcesChanged
	*node.rcl_node_t = C.rcl_get_zero_initialized_node()
	defer onErr(&err, node.Close)

	cname := C.CString(node_name)
	defer C.free(unsafe.Pointer(cname))
	cnamespace := C.CString(namespace)
	defer C.free(unsafe.Pointer(cnamespace))
	rcl_node_options := C.rcl_node_get_default_options()
	rc := C.rcl_node_init(
		node.rcl_node_t,
		cname,
		cnamespace,
		c.rcl_context_t,
		&rcl_node_options,
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCastC(rc, "failed to create node:")
	}
	cstr := C.rcl_node_get_name(node.rcl_node_t)
	if cstr == nil {
		return nil, errors.New("unexpectedly invalid node")
	}
	node.name = C.GoString(cstr)
	cstr = C.rcl_node_get_namespace(node.rcl_node_t)
	if cstr == nil {
		return nil, errors.New("unexpectedly invalid node")
	}
	node.namespace = C.GoString(cstr)
	cstr = C.rcl_node_get_fully_qualified_name(node.rcl_node_t)
	if cstr == nil {
		return nil, errors.New("unexpectedly invalid node")
	}
	node.fullyQualifiedName = C.GoString(cstr)
	loggerName := C.rcl_node_get_logger_name(node.rcl_node_t)
	if loggerName == nil {
		return nil, errors.New("unexpectedly invalid node")
	}
	node.logger = GetLogger(C.GoString(loggerName))

	// Create the /rosout log publisher for this node so that its log messages
	// are visible to /rosout subscribers (e.g. rqt_console), matching the
	// behavior of rclcpp and rclpy nodes.
	//
	// Since rcl 9.x (Jazzy) rcl_node_init no longer creates this publisher
	// itself; the client library is responsible for it. rcl_node_options
	// enables rosout by default (enable_rosout = true), so the only remaining
	// gate is whether rosout logging is enabled globally.
	if bool(C.rcl_logging_rosout_enabled()) && bool(rcl_node_options.enable_rosout) {
		rc = C.rcl_logging_rosout_init_publisher_for_node(node.rcl_node_t)
		if rc != C.RCL_RET_OK {
			return nil, errorsCastC(rc, "failed to initialize rosout publisher")
		}
		node.rosoutInitialized = true
	}

	c.addResource(node)
	return node, nil
}

// Close closes every entity of the node and finalizes the node. Entities that a
// running wait set still uses are finalized when it lets go of them, and the
// node after them.
func (n *Node) Close() error {
	if !n.life.markClosed() {
		return closeErr("node")
	}
	n.context.removeResource(n)
	err := n.rosResourceStore.closeAll(nil)
	return errors.Join(err, n.life.finalize(&n.context.life, "node", n.fini))
}

func (n *Node) fini() (err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if n.rcl_node_t == nil {
		return nil
	}
	// Finalize the /rosout log publisher before finalizing the node, mirroring
	// the initialization done in NewNode.
	if n.rosoutInitialized {
		if rc := C.rcl_logging_rosout_fini_publisher_for_node(n.rcl_node_t); rc != C.RCL_RET_OK {
			err = errors.Join(err, errorsCastC(rc, "failed to finalize rosout publisher"))
		}
		n.rosoutInitialized = false
	}

	rc := C.rcl_node_fini(n.rcl_node_t)
	if rc != C.RCL_RET_OK {
		err = errors.Join(err, errorsCast(rc))
	}
	C.free(unsafe.Pointer(n.rcl_node_t))
	n.rcl_node_t = nil

	return err
}

// Context returns the context n belongs to.
func (n *Node) Context() *Context {
	return n.context
}

// Logger returns the logger associated with n.
func (n *Node) Logger() *Logger {
	return n.logger
}

// Name returns the name of n.
func (n *Node) Name() string {
	return n.name
}

// Namespace returns the namespace of n.
func (n *Node) Namespace() string {
	return n.namespace
}

// FullyQualifiedName returns the fully qualified name of n, which includes the
// namespace as well as the name.
func (n *Node) FullyQualifiedName() string {
	return n.fullyQualifiedName
}

func spinErr(spinner string, err error) error {
	if err == nil {
		return nil
	}
	return fmt.Errorf("failed to spin %s: %w", spinner, err)
}

// Spin starts and waits for all ROS resources in the node that need waiting
// such as subscriptions. Resources created on the node while Spin is running
// are picked up as well. Spin returns when an error occurs or ctx is canceled.
func (n *Node) Spin(ctx context.Context) error {
	return spinErr("node", n.context.spin(ctx, &n.rosResourceStore))
}

type PublisherOptions struct {
	Qos qos.Profile
}

func NewDefaultPublisherOptions() *PublisherOptions { return &PublisherOptions{Qos: qos.NewDefault()} }

type Publisher struct {
	rosID
	life            lifetime
	TopicName       string
	typeSupport     types.MessageTypeSupport
	node            *Node
	rcl_publisher_t *C.rcl_publisher_t
	topicName       *C.char
}

// NewPublisher creates a new publisher.
//
// options must not be modified after passing it to this function. If options is
// nil, default options are used.
func (n *Node) NewPublisher(
	topicName string,
	ros2msg types.MessageTypeSupport,
	options *PublisherOptions,
) (pub *Publisher, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if options == nil {
		options = NewDefaultPublisherOptions()
	}
	pub = &Publisher{
		TopicName:       topicName,
		typeSupport:     ros2msg,
		node:            n,
		rcl_publisher_t: (*C.rcl_publisher_t)(C.malloc(C.sizeof_rcl_publisher_t)),
		topicName:       C.CString(topicName),
	}
	*pub.rcl_publisher_t = C.rcl_get_zero_initialized_publisher()
	defer onErr(&err, pub.Close)
	rcl_publisher_options_t := C.rcl_publisher_get_default_options()
	rcl_publisher_options_t.allocator = *n.context.rcl_allocator_t
	qosToC(options.Qos, &rcl_publisher_options_t.qos)

	var rc C.rcl_ret_t = C.rcl_publisher_init(
		pub.rcl_publisher_t,
		n.rcl_node_t,
		(*C.rosidl_message_type_support_t)(ros2msg.TypeSupport()),
		pub.topicName,
		&rcl_publisher_options_t,
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}

	n.addResource(pub)
	return pub, nil
}

// Node returns the node p belongs to.
func (p *Publisher) Node() *Node {
	return p.node
}

func (p *Publisher) Publish(ros2msg types.Message) error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !p.life.acquire() {
		return closedErr("publisher")
	}
	defer p.life.release()
	var rc C.rcl_ret_t

	ptr := p.typeSupport.PrepareMemory()
	defer p.typeSupport.ReleaseMemory(ptr)
	p.typeSupport.AsCStruct(ptr, ros2msg)
	rc = C.rcl_publish(p.rcl_publisher_t, ptr, nil)
	if rc != C.RCL_RET_OK {
		return errorsCastC(rc, fmt.Sprintf("rcl_publish() failed for publisher on %q", p.TopicName))
	}
	return nil
}

// PublishSerialized publishes a message that has already been serialized.
func (p *Publisher) PublishSerialized(msg []byte) error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !p.life.acquire() {
		return closedErr("publisher")
	}
	defer p.life.release()
	rclMsg, err := newSerializedMessage(len(msg))
	if err != nil {
		return fmt.Errorf("failed to publish serialized message: %v", err)
	}
	defer rclMsg.Close()
	copy(rclMsg.AsSlice(), msg)
	rc := C.rcl_publish_serialized_message(p.rcl_publisher_t, rclMsg.c(), nil)
	if rc != C.RCL_RET_OK {
		return errorsCastC(rc, "failed to publish serialized message")
	}
	return nil
}

// GetSubscriptionCount returns the number of subscriptions matched to p.
func (p *Publisher) GetSubscriptionCount() (int, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !p.life.acquire() {
		return 0, closedErr("publisher")
	}
	defer p.life.release()
	var count C.size_t
	rc := C.rcl_publisher_get_subscription_count(p.rcl_publisher_t, &count)
	if rc != C.RCL_RET_OK {
		return 0, errorsCastC(rc, "failed to get subscription count")
	}
	return int(count), nil
}

/*
Close frees the allocated memory
*/
func (p *Publisher) Close() error {
	return closeEntity(&p.life, "publisher", func() { p.node.removeResource(p) }, &p.node.life, p.fini)
}

func (p *Publisher) fini() (err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if p.rcl_publisher_t == nil {
		return nil
	}
	rc := C.rcl_publisher_fini(p.rcl_publisher_t, p.node.rcl_node_t)
	if rc != C.RCL_RET_OK {
		err = errors.Join(err, errorsCast(rc))
	}
	C.free(unsafe.Pointer(p.rcl_publisher_t))
	p.rcl_publisher_t = nil
	C.free(unsafe.Pointer(p.topicName))
	return err
}

type Clock struct {
	rosID
	rcl_clock_t *C.rcl_clock_t
	context     *Context

	// jumpMu serializes changes to the clock's jump callbacks. rcl keeps them
	// in an unsynchronized array that timers (including the expire timer of
	// action servers) grow and shrink on init and fini, so timers using one
	// clock must not be created or finalized concurrently.
	jumpMu sync.Mutex
}

func NewClock(clockType ClockType) (*Clock, error) {
	if defaultContext == nil {
		return nil, errInitNotCalled
	}
	return defaultContext.NewClock(clockType)
}

func (c *Context) NewClock(clockType ClockType) (clock *Clock, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if clockType == ClockTypeUninitialized {
		clockType = ClockTypeROSTime
	}
	clock = &Clock{
		context: c,
	}
	cclock := (*C.rcl_clock_t)(C.calloc(1, C.sizeof_rcl_clock_t))
	clock.rcl_clock_t = cclock
	defer func() {
		// Only the clock is released on failure; the context stays usable.
		if err != nil {
			C.free(unsafe.Pointer(cclock))
		}
	}()
	rc := C.rcl_clock_init(
		C.rcl_clock_type_t(clockType),
		clock.rcl_clock_t,
		c.rcl_allocator_t,
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}
	c.addResource(clock)
	return clock, nil
}

/*
Close frees the allocated memory
*/
func (c *Clock) Close() (err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if c.rcl_clock_t == nil {
		return closeErr("clock")
	}
	c.context.removeResource(c)
	rc := C.rcl_clock_fini(c.rcl_clock_t)
	if rc != C.RCL_RET_OK {
		err = errors.Join(err, errorsCast(rc))
	}
	C.free(unsafe.Pointer(c.rcl_clock_t))
	c.rcl_clock_t = nil
	return err
}

// Context returns the context c belongs to.
func (c *Clock) Context() *Context {
	return c.context
}

func (c *Clock) now() (time.Duration, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var t C.rcl_time_point_value_t
	rc := C.rcl_clock_get_now(c.rcl_clock_t, &t)
	if rc != C.RCL_RET_OK {
		return 0, errorsCastC(rc, "failed to get current time")
	}
	return time.Duration(t), nil
}

type Timer struct {
	rosID
	life        lifetime
	rcl_timer_t *C.rcl_timer_t
	Callback    func(*Timer)
	context     *Context
	clock       *Clock
}

func NewTimer(timeout time.Duration, timerCallback func(*Timer)) (*Timer, error) {
	if defaultContext == nil {
		return nil, errInitNotCalled
	}
	return defaultContext.NewTimer(timeout, timerCallback)
}

func (c *Context) NewTimer(timeout time.Duration, timer_callback func(*Timer)) (timer *Timer, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if timeout == 0 {
		timeout = 1000 * time.Millisecond
	}
	timer = &Timer{
		rcl_timer_t: (*C.rcl_timer_t)(C.malloc(C.sizeof_rcl_timer_t)),
		Callback:    timer_callback,
		context:     c,
		clock:       c.Clock(),
	}
	*timer.rcl_timer_t = C.rcl_get_zero_initialized_timer()
	defer onErr(&err, timer.Close)

	timer.clock.jumpMu.Lock()
	defer timer.clock.jumpMu.Unlock()
	rc := C.rcl_timer_init2(
		timer.rcl_timer_t,
		timer.clock.rcl_clock_t,
		c.rcl_context_t,
		C.int64_t(timeout),
		nil,
		*c.rcl_allocator_t,
		C.bool(true),
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}

	c.addResource(timer)
	return timer, nil
}

// Context returns the context t belongs to.
func (t *Timer) Context() *Context {
	return t.context
}

func (t *Timer) GetTimeUntilNextCall() (int64, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !t.life.acquire() {
		return 0, closedErr("timer")
	}
	defer t.life.release()
	var time_until_next_call C.int64_t
	rc := C.rcl_timer_get_time_until_next_call(t.rcl_timer_t, &time_until_next_call)
	if rc != C.RCL_RET_OK {
		return 0, errorsCast(rc)
	}
	return int64(time_until_next_call), nil
}

// call marks the timer as called: its next expiry moves forward by exactly one
// period from the previous expiry (skipping missed periods), so the timer
// keeps its period regardless of when it is serviced. Reset would instead
// restart the period from now, adding the wake-up latency to every period.
func (t *Timer) call() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	rc := C.rcl_timer_call(t.rcl_timer_t)
	if rc != C.RCL_RET_OK {
		return errorsCast(rc)
	}
	return nil
}

func (t *Timer) Reset() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !t.life.acquire() {
		return closedErr("timer")
	}
	defer t.life.release()
	rc := C.rcl_timer_reset(t.rcl_timer_t)
	if rc != C.RCL_RET_OK {
		return errorsCast(rc)
	}
	return nil
}

/*
Close frees the allocated memory
*/
func (t *Timer) Close() error {
	return closeEntity(&t.life, "timer", func() { t.context.removeResource(t) }, &t.context.life, t.fini)
}

func (t *Timer) fini() (err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if t.rcl_timer_t == nil {
		return nil
	}
	t.clock.jumpMu.Lock()
	rc := C.rcl_timer_fini(t.rcl_timer_t)
	t.clock.jumpMu.Unlock()
	if rc != C.RCL_RET_OK {
		err = errors.Join(err, errorsCast(rc))
	}
	C.free(unsafe.Pointer(t.rcl_timer_t))
	t.rcl_timer_t = nil
	return err
}

type SubscriptionOptions struct {
	Qos qos.Profile
}

func NewDefaultSubscriptionOptions() *SubscriptionOptions {
	return &SubscriptionOptions{Qos: qos.NewDefault()}
}

type SubscriptionCallback func(*Subscription)

type Subscription struct {
	rosID
	life               lifetime
	TopicName          string
	Ros2MsgType        types.MessageTypeSupport
	Callback           SubscriptionCallback
	node               *Node
	rcl_subscription_t *C.rcl_subscription_t
	topicName          *C.char
}

// NewSubscription creates a new subscription.
//
// options must not be modified after passing it to this function. If options is
// nil, default options are used.
func (n *Node) NewSubscription(
	topicName string,
	ros2msg types.MessageTypeSupport,
	options *SubscriptionOptions,
	subscriptionCallback SubscriptionCallback,
) (sub *Subscription, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if options == nil {
		options = NewDefaultSubscriptionOptions()
	}
	sub = &Subscription{
		TopicName:          topicName,
		Ros2MsgType:        ros2msg,
		Callback:           subscriptionCallback,
		node:               n,
		rcl_subscription_t: (*C.rcl_subscription_t)(C.malloc(C.sizeof_rcl_subscription_t)),
		topicName:          C.CString(topicName),
	}
	*sub.rcl_subscription_t = C.rcl_get_zero_initialized_subscription()
	defer onErr(&err, sub.Close)
	rclOpts := C.rcl_subscription_get_default_options()
	rclOpts.allocator = *n.context.rcl_allocator_t
	qosToC(options.Qos, &rclOpts.qos)

	rc := C.rcl_subscription_init(
		sub.rcl_subscription_t,
		n.rcl_node_t,
		(*C.rosidl_message_type_support_t)(ros2msg.TypeSupport()),
		sub.topicName,
		&rclOpts,
	)
	if rc != C.RCL_RET_OK {
		return sub, errorsCastC(rc, fmt.Sprintf("Topic name '%s'", topicName))
	}

	n.addResource(sub)
	return sub, nil
}

// Node returns the node s belongs to.
func (s *Subscription) Node() *Node {
	return s.node
}

func (s *Subscription) TakeMessage(out types.Message) (*MessageInfo, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !s.life.acquire() {
		return nil, closedErr("subscription")
	}
	defer s.life.release()
	rmw_message_info := C.rmw_get_zero_initialized_message_info()

	ros2_msg_receive_buffer := s.Ros2MsgType.PrepareMemory()
	defer s.Ros2MsgType.ReleaseMemory(ros2_msg_receive_buffer)

	rc := C.rcl_take(s.rcl_subscription_t, ros2_msg_receive_buffer, &rmw_message_info, nil)
	if rc != C.RCL_RET_OK {
		return nil, errorsCastC(rc, fmt.Sprintf("rcl_take() failed for subscription to %q", s.TopicName))
	}
	s.Ros2MsgType.AsGoStruct(out, ros2_msg_receive_buffer)
	return &MessageInfo{
		SourceTimestamp:   time.Unix(0, int64(rmw_message_info.source_timestamp)),
		ReceivedTimestamp: time.Unix(0, int64(rmw_message_info.received_timestamp)),
		FromIntraProcess:  bool(rmw_message_info.from_intra_process),
	}, nil
}

// TakeSerializedMessage takes a message without deserializing it and returns it
// as a byte slice.
func (s *Subscription) TakeSerializedMessage() ([]byte, *MessageInfo, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !s.life.acquire() {
		return nil, nil, closedErr("subscription")
	}
	defer s.life.release()
	info := C.rmw_get_zero_initialized_message_info()
	msg, err := newSerializedMessage(0)
	if err != nil {
		return nil, nil, err
	}
	defer msg.Close()
	rc := C.rcl_take_serialized_message(s.rcl_subscription_t, msg.c(), &info, nil)
	if rc != C.RCL_RET_OK {
		return nil, nil, errorsCastC(rc, fmt.Sprintf("rcl_take_serialized_message() failed for subscription to %q", s.TopicName))
	}
	return msg.ToSlice(), &MessageInfo{
		SourceTimestamp:   time.Unix(0, int64(info.source_timestamp)),
		ReceivedTimestamp: time.Unix(0, int64(info.received_timestamp)),
		FromIntraProcess:  bool(info.from_intra_process),
	}, nil
}

// GetPublisherCount returns the number of publishers matched to s.
func (s *Subscription) GetPublisherCount() (int, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !s.life.acquire() {
		return 0, closedErr("subscription")
	}
	defer s.life.release()
	var count C.size_t
	rc := C.rcl_subscription_get_publisher_count(s.rcl_subscription_t, &count)
	if rc != C.RCL_RET_OK {
		return 0, errorsCastC(rc, "failed to get publisher count")
	}
	return int(count), nil
}

/*
Close frees the allocated memory
*/
func (s *Subscription) Close() error {
	return closeEntity(&s.life, "subscription", func() { s.node.removeResource(s) }, &s.node.life, s.fini)
}

func (s *Subscription) fini() (err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if s.rcl_subscription_t == nil {
		return nil
	}
	rc := C.rcl_subscription_fini(s.rcl_subscription_t, s.node.rcl_node_t)
	if rc != C.RCL_RET_OK {
		err = errors.Join(err, errorsCast(rc))
	}
	C.free(unsafe.Pointer(s.rcl_subscription_t))
	s.rcl_subscription_t = nil
	C.free(unsafe.Pointer(s.topicName))
	return err
}

type guardCondition struct {
	rosID
	life              lifetime
	rclGuardCondition *C.rcl_guard_condition_t
	context           *Context
}

func (c *Context) newGuardCondition() (g *guardCondition, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	g = &guardCondition{
		rclGuardCondition: (*C.rcl_guard_condition_t)(C.malloc(C.sizeof_rcl_guard_condition_t)),
		context:           c,
	}
	*g.rclGuardCondition = C.rcl_get_zero_initialized_guard_condition()
	defer onErr(&err, g.Close)
	rc := C.rcl_guard_condition_init(
		g.rclGuardCondition,
		c.rcl_context_t,
		C.rcl_guard_condition_get_default_options(),
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}
	c.addResource(g)
	return g, nil
}

func (c *guardCondition) Close() error {
	return closeEntity(&c.life, "guard condition", func() { c.context.removeResource(c) }, &c.context.life, c.fini)
}

func (c *guardCondition) fini() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if c.rclGuardCondition == nil {
		return nil
	}
	rc := C.rcl_guard_condition_fini(c.rclGuardCondition)
	C.free(unsafe.Pointer(c.rclGuardCondition))
	c.rclGuardCondition = nil
	if rc == C.RCL_RET_OK {
		return nil
	}
	return errorsCast(rc)
}

func (c *guardCondition) Trigger() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	rc := C.rcl_trigger_guard_condition(c.rclGuardCondition)
	if rc != C.RCL_RET_OK {
		return errorsCast(rc)
	}
	return nil
}

type RequestID struct {
	WriterGUID     [16]byte
	SequenceNumber int64
}

func newRequestID(reqID *C.rmw_request_id_t) RequestID {
	return RequestID{
		WriterGUID:     *(*[16]byte)(unsafe.Pointer(&reqID.writer_guid)),
		SequenceNumber: int64(reqID.sequence_number),
	}
}

type ServiceInfo struct {
	SourceTimestamp   time.Time
	ReceivedTimestamp time.Time
	RequestID         RequestID
}

type ServiceOptions struct {
	Qos qos.Profile
}

func NewDefaultServiceOptions() *ServiceOptions { return &ServiceOptions{Qos: qos.NewDefault()} }

type ServiceResponseSender interface {
	SendResponse(resp types.Message) error
}

type serviceResponseSender func(resp types.Message) error

func (s serviceResponseSender) SendResponse(resp types.Message) error {
	return s(resp)
}

type ServiceRequestHandler func(*ServiceInfo, types.Message, ServiceResponseSender)

type Service struct {
	rosID
	life                lifetime
	node                *Node
	rclService          *C.rcl_service_t
	name                *C.char
	handler             ServiceRequestHandler
	requestTypeSupport  types.MessageTypeSupport
	responseTypeSupport types.MessageTypeSupport
}

// NewService creates a new service.
//
// options must not be modified after passing it to this function. If options is
// nil, default options are used.
func (n *Node) NewService(
	name string,
	typeSupport types.ServiceTypeSupport,
	options *ServiceOptions,
	handler ServiceRequestHandler,
) (s *Service, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if options == nil {
		options = NewDefaultServiceOptions()
	}
	s = &Service{
		requestTypeSupport:  typeSupport.Request(),
		responseTypeSupport: typeSupport.Response(),
		node:                n,
		rclService:          (*C.rcl_service_t)(C.malloc(C.sizeof_rcl_service_t)),
		name:                C.CString(name),
		handler:             handler,
	}
	*s.rclService = C.rcl_get_zero_initialized_service()
	defer onErr(&err, s.Close)
	opts := C.rcl_service_options_t{allocator: *n.context.rcl_allocator_t}
	qosToC(options.Qos, &opts.qos)
	retCode := C.rcl_service_init(
		s.rclService,
		n.rcl_node_t,
		(*C.struct_rosidl_service_type_support_t)(typeSupport.TypeSupport()),
		s.name,
		&opts,
	)
	if retCode != C.RCL_RET_OK {
		return nil, errorsCastC(retCode, "failed to create service")
	}
	n.addResource(s)
	return s, nil
}

func (s *Service) Close() error {
	return closeEntity(&s.life, "service", func() { s.node.removeResource(s) }, &s.node.life, s.fini)
}

func (s *Service) fini() (err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if s.name == nil {
		return nil
	}
	rc := C.rcl_service_fini(s.rclService, s.node.rcl_node_t)
	if rc != C.RCL_RET_OK {
		err = errorsCastC(rc, "failed to finalize service")
	}
	C.free(unsafe.Pointer(s.rclService))
	C.free(unsafe.Pointer(s.name))
	s.name = nil
	return err
}

// Node returns the node s belongs to.
func (s *Service) Node() *Node {
	return s.node
}

func (s *Service) handleRequest() {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var reqHeader C.rmw_service_info_t
	reqBuffer := s.requestTypeSupport.PrepareMemory()
	defer s.requestTypeSupport.ReleaseMemory(reqBuffer)
	switch rc := C.rcl_take_request_with_info(s.rclService, &reqHeader, reqBuffer); rc {
	case C.RCL_RET_OK:
		info := ServiceInfo{
			SourceTimestamp:   time.Unix(0, int64(reqHeader.source_timestamp)),
			ReceivedTimestamp: time.Unix(0, int64(reqHeader.received_timestamp)),
			RequestID:         newRequestID(&reqHeader.request_id),
		}
		req := s.requestTypeSupport.New()
		s.requestTypeSupport.AsGoStruct(req, reqBuffer)
		s.handler(
			&info,
			req,
			serviceResponseSender(func(resp types.Message) error {
				runtime.LockOSThread() // rcl error state is per OS thread; see errorString
				defer runtime.UnlockOSThread()
				// The handler may answer later, from another goroutine,
				// possibly after the service was closed.
				if !s.life.acquire() {
					return closedErr("service")
				}
				defer s.life.release()
				respBuffer := s.responseTypeSupport.PrepareMemory()
				defer s.responseTypeSupport.ReleaseMemory(respBuffer)
				s.responseTypeSupport.AsCStruct(respBuffer, resp)
				rc := C.rcl_send_response(s.rclService, &reqHeader.request_id, respBuffer)
				if rc != C.RCL_RET_OK {
					return errorsCastC(rc, "failed to send response")
				}
				return nil
			}),
		)
	case C.RCL_RET_SERVICE_TAKE_FAILED:
	default:
		s.node.Logger().Debug(errorsCastC(rc, "failed to take request"))
	}
}

type ClientOptions struct {
	Qos qos.Profile
}

func NewDefaultClientOptions() *ClientOptions { return &ClientOptions{Qos: qos.NewDefault()} }

// Client is used to send requests to and receive responses from a service.
//
// Calling Send and Close is thread-safe. Creating clients is not thread-safe.
type Client struct {
	rosID
	life      lifetime
	node      *Node
	rclClient *C.rcl_client_t
	sender    requestSender
}

// NewClient creates a new client.
//
// options must not be modified after passing it to this function. If options is
// nil, default options are used.
func (n *Node) NewClient(
	serviceName string,
	typeSupport types.ServiceTypeSupport,
	options *ClientOptions,
) (c *Client, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if options == nil {
		options = NewDefaultClientOptions()
	}
	c = &Client{
		node:      n,
		rclClient: (*C.rcl_client_t)(C.malloc(C.sizeof_rcl_client_t)),
	}
	c.sender = newRequestSender(requestSenderTransport{
		SendRequest:  c.sendRequest,
		TakeResponse: c.takeResponse,
		TypeSupport:  typeSupport,
		Logger:       n.Logger(),
	})
	*c.rclClient = C.rcl_get_zero_initialized_client()
	defer onErr(&err, c.Close)
	opts := C.rcl_client_options_t{allocator: *n.context.rcl_allocator_t}
	qosToC(options.Qos, &opts.qos)
	cserviceName := C.CString(serviceName)
	defer C.free(unsafe.Pointer(cserviceName))
	rc := C.rcl_client_init(
		c.rclClient,
		n.rcl_node_t,
		(*C.struct_rosidl_service_type_support_t)(typeSupport.TypeSupport()),
		cserviceName,
		&opts,
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCastC(rc, "failed to create client")
	}
	n.addResource(c)
	return c, nil
}

func (c *Client) Close() error {
	return closeEntity(&c.life, "client", func() {
		c.node.removeResource(c)
		// Wake callers waiting for a response right away.
		c.sender.Close() //nolint:errcheck
	}, &c.node.life, c.fini)
}

func (c *Client) fini() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if c.rclClient == nil {
		return nil
	}
	var err error
	rc := C.rcl_client_fini(c.rclClient, c.node.rcl_node_t)
	if rc != C.RCL_RET_OK {
		err = errors.Join(err, errorsCastC(rc, "failed to finalize client"))
	}
	C.free(unsafe.Pointer(c.rclClient))
	c.rclClient = nil

	return err
}

// Node returns the node c belongs to.
func (c *Client) Node() *Node {
	return c.node
}

func (c *Client) Send(ctx context.Context, req types.Message) (types.Message, *ServiceInfo, error) {
	if !c.life.acquire() {
		return nil, nil, closedErr("client")
	}
	defer c.life.release()
	resp, info, err := c.sender.Send(ctx, req)
	if rmwInfo, ok := info.(*ServiceInfo); ok {
		return resp, rmwInfo, err
	}

	return resp, nil, err
}

func (c *Client) sendRequest(req unsafe.Pointer) (C.int64_t, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var seqNum C.int64_t
	rc := C.rcl_send_request(c.rclClient, req, &seqNum)
	if rc != C.RCL_RET_OK {
		return 0, errorsCastC(rc, "failed to send request")
	}
	return seqNum, nil
}

func (c *Client) takeResponse(resp unsafe.Pointer) (C.int64_t, interface{}, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var header C.rmw_service_info_t
	switch rc := C.rcl_take_response_with_info(c.rclClient, &header, resp); rc {
	case C.RCL_RET_OK:
		return header.request_id.sequence_number, &ServiceInfo{
			SourceTimestamp:   time.Unix(0, int64(header.source_timestamp)),
			ReceivedTimestamp: time.Unix(0, int64(header.received_timestamp)),
			RequestID:         newRequestID(&header.request_id),
		}, nil
	case C.RCL_RET_CLIENT_TAKE_FAILED:
		return 0, nil, errTakeFailed
	default:
		return 0, nil, errorsCastC(rc, "failed to take response")
	}
}

var errTakeFailed = errors.New("take failed")

type sendResult struct {
	resp      types.Message
	otherData interface{}
}

type requestSenderTransport struct {
	SendRequest  func(unsafe.Pointer) (C.int64_t, error)
	TakeResponse func(unsafe.Pointer) (C.int64_t, interface{}, error)
	TypeSupport  types.ServiceTypeSupport
	Logger       *Logger
}

type requestSender struct {
	transport       requestSenderTransport
	pendingRequests map[C.int64_t]chan *sendResult
	mutex           sync.Mutex
}

func newRequestSender(transport requestSenderTransport) requestSender {
	return requestSender{
		transport:       transport,
		pendingRequests: make(map[C.int64_t]chan *sendResult),
	}
}

func (s *requestSender) Close() error {
	s.mutex.Lock()
	defer s.mutex.Unlock()
	for seqNum, ch := range s.pendingRequests {
		delete(s.pendingRequests, seqNum)
		close(ch)
	}
	return nil
}

func (s *requestSender) Send(ctx context.Context, req types.Message) (types.Message, interface{}, error) {
	resultChan, seqNum, err := s.addPendingRequest(req)
	if err != nil {
		return nil, nil, err
	}
	defer func() {
		s.mutex.Lock()
		defer s.mutex.Unlock()
		delete(s.pendingRequests, seqNum)
	}()
	select {
	case <-ctx.Done():
		return nil, nil, ctx.Err()
	case result := <-resultChan:
		if result == nil {
			return nil, nil, errors.New("sender was closed before a response was received")
		}
		return result.resp, result.otherData, nil
	}
}

func (s *requestSender) addPendingRequest(req types.Message) (<-chan *sendResult, C.int64_t, error) {
	ts := s.transport.TypeSupport.Request()
	buf := ts.PrepareMemory()
	defer ts.ReleaseMemory(buf)
	ts.AsCStruct(buf, req)
	s.mutex.Lock()
	defer s.mutex.Unlock()
	seqNum, err := s.transport.SendRequest(buf)
	if err != nil {
		return nil, 0, err
	}
	resultChan := make(chan *sendResult, 1)
	s.pendingRequests[seqNum] = resultChan
	return resultChan, seqNum, nil
}

func (s *requestSender) HandleResponse() {
	ts := s.transport.TypeSupport.Response()
	buf := ts.PrepareMemory()
	defer ts.ReleaseMemory(buf)
	respChan, otherData := func() (chan *sendResult, interface{}) {
		s.mutex.Lock()
		defer s.mutex.Unlock()
		seqNum, otherData, err := s.transport.TakeResponse(buf)
		if err != nil {
			if !errors.Is(err, errTakeFailed) {
				s.transport.Logger.Error(err)
			}
			return nil, nil
		}
		ch := s.pendingRequests[seqNum]
		delete(s.pendingRequests, seqNum)
		return ch, otherData
	}()
	if respChan == nil {
		return
	}
	defer close(respChan)
	result := &sendResult{
		resp:      ts.New(),
		otherData: otherData,
	}
	ts.AsGoStruct(result.resp, buf)
	respChan <- result
}
