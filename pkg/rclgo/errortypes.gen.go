/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package rclgo

/*
#include <rcl/types.h>
#include <rmw/ret_types.h>
#include <rcl_action/types.h>
*/
import "C"
import (
	"runtime"
)

func errorsCastC(rcl_ret_t C.rcl_ret_t, context string) error {
	stackTraceBuffer := make([]byte, 2048)
	runtime.Stack(stackTraceBuffer, true) // Get stack trace of the current running thread only

	// https://stackoverflow.com/questions/9928221/table-of-functions-vs-switch-in-golang
	// switch-case is faster thanks to compiler optimization than a dispatcher?
	switch rcl_ret_t {
	case C.RCL_RET_ALREADY_INIT:
		return &AlreadyInit{rclError: rclError{rclRetCode: 100, trace: string(stackTraceBuffer), context: errorsBuildContext(&AlreadyInit{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_NOT_INIT:
		return &NotInit{rclError: rclError{rclRetCode: 101, trace: string(stackTraceBuffer), context: errorsBuildContext(&NotInit{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_MISMATCHED_RMW_ID:
		return &MismatchedRmwId{rclError: rclError{rclRetCode: 102, trace: string(stackTraceBuffer), context: errorsBuildContext(&MismatchedRmwId{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_TOPIC_NAME_INVALID:
		return &TopicNameInvalid{rclError: rclError{rclRetCode: 103, trace: string(stackTraceBuffer), context: errorsBuildContext(&TopicNameInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_SERVICE_NAME_INVALID:
		return &ServiceNameInvalid{rclError: rclError{rclRetCode: 104, trace: string(stackTraceBuffer), context: errorsBuildContext(&ServiceNameInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_UNKNOWN_SUBSTITUTION:
		return &UnknownSubstitution{rclError: rclError{rclRetCode: 105, trace: string(stackTraceBuffer), context: errorsBuildContext(&UnknownSubstitution{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_ALREADY_SHUTDOWN:
		return &AlreadyShutdown{rclError: rclError{rclRetCode: 106, trace: string(stackTraceBuffer), context: errorsBuildContext(&AlreadyShutdown{}, context, string(stackTraceBuffer))}}
	//case C.RCL_RET_NOT_FOUND:
	//return &NotFound{rclError: rclError{rclRetCode: 107, trace: string(stackTraceBuffer), context: errorsBuildContext(&NotFound{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_NODE_INVALID:
		return &NodeInvalid{rclError: rclError{rclRetCode: 200, trace: string(stackTraceBuffer), context: errorsBuildContext(&NodeInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_NODE_INVALID_NAME:
		return &NodeInvalidName{rclError: rclError{rclRetCode: 201, trace: string(stackTraceBuffer), context: errorsBuildContext(&NodeInvalidName{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_NODE_INVALID_NAMESPACE:
		return &NodeInvalidNamespace{rclError: rclError{rclRetCode: 202, trace: string(stackTraceBuffer), context: errorsBuildContext(&NodeInvalidNamespace{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_NODE_NAME_NON_EXISTENT:
		return &NodeNameNonExistent{rclError: rclError{rclRetCode: 203, trace: string(stackTraceBuffer), context: errorsBuildContext(&NodeNameNonExistent{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_PUBLISHER_INVALID:
		return &PublisherInvalid{rclError: rclError{rclRetCode: 300, trace: string(stackTraceBuffer), context: errorsBuildContext(&PublisherInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_SUBSCRIPTION_INVALID:
		return &SubscriptionInvalid{rclError: rclError{rclRetCode: 400, trace: string(stackTraceBuffer), context: errorsBuildContext(&SubscriptionInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_SUBSCRIPTION_TAKE_FAILED:
		return &SubscriptionTakeFailed{rclError: rclError{rclRetCode: 401, trace: string(stackTraceBuffer), context: errorsBuildContext(&SubscriptionTakeFailed{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_CLIENT_INVALID:
		return &ClientInvalid{rclError: rclError{rclRetCode: 500, trace: string(stackTraceBuffer), context: errorsBuildContext(&ClientInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_CLIENT_TAKE_FAILED:
		return &ClientTakeFailed{rclError: rclError{rclRetCode: 501, trace: string(stackTraceBuffer), context: errorsBuildContext(&ClientTakeFailed{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_SERVICE_INVALID:
		return &ServiceInvalid{rclError: rclError{rclRetCode: 600, trace: string(stackTraceBuffer), context: errorsBuildContext(&ServiceInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_SERVICE_TAKE_FAILED:
		return &ServiceTakeFailed{rclError: rclError{rclRetCode: 601, trace: string(stackTraceBuffer), context: errorsBuildContext(&ServiceTakeFailed{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_TIMER_INVALID:
		return &TimerInvalid{rclError: rclError{rclRetCode: 800, trace: string(stackTraceBuffer), context: errorsBuildContext(&TimerInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_TIMER_CANCELED:
		return &TimerCanceled{rclError: rclError{rclRetCode: 801, trace: string(stackTraceBuffer), context: errorsBuildContext(&TimerCanceled{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_WAIT_SET_INVALID:
		return &WaitSetInvalid{rclError: rclError{rclRetCode: 900, trace: string(stackTraceBuffer), context: errorsBuildContext(&WaitSetInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_WAIT_SET_EMPTY:
		return &WaitSetEmpty{rclError: rclError{rclRetCode: 901, trace: string(stackTraceBuffer), context: errorsBuildContext(&WaitSetEmpty{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_WAIT_SET_FULL:
		return &WaitSetFull{rclError: rclError{rclRetCode: 902, trace: string(stackTraceBuffer), context: errorsBuildContext(&WaitSetFull{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_INVALID_REMAP_RULE:
		return &InvalidRemapRule{rclError: rclError{rclRetCode: 1001, trace: string(stackTraceBuffer), context: errorsBuildContext(&InvalidRemapRule{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_WRONG_LEXEME:
		return &WrongLexeme{rclError: rclError{rclRetCode: 1002, trace: string(stackTraceBuffer), context: errorsBuildContext(&WrongLexeme{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_INVALID_ROS_ARGS:
		return &InvalidRosArgs{rclError: rclError{rclRetCode: 1003, trace: string(stackTraceBuffer), context: errorsBuildContext(&InvalidRosArgs{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_INVALID_PARAM_RULE:
		return &InvalidParamRule{rclError: rclError{rclRetCode: 1010, trace: string(stackTraceBuffer), context: errorsBuildContext(&InvalidParamRule{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_INVALID_LOG_LEVEL_RULE:
		return &InvalidLogLevelRule{rclError: rclError{rclRetCode: 1020, trace: string(stackTraceBuffer), context: errorsBuildContext(&InvalidLogLevelRule{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_EVENT_INVALID:
		return &EventInvalid{rclError: rclError{rclRetCode: 2000, trace: string(stackTraceBuffer), context: errorsBuildContext(&EventInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_EVENT_TAKE_FAILED:
		return &EventTakeFailed{rclError: rclError{rclRetCode: 2001, trace: string(stackTraceBuffer), context: errorsBuildContext(&EventTakeFailed{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_LIFECYCLE_STATE_REGISTERED:
		return &LifecycleStateRegistered{rclError: rclError{rclRetCode: 3000, trace: string(stackTraceBuffer), context: errorsBuildContext(&LifecycleStateRegistered{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_LIFECYCLE_STATE_NOT_REGISTERED:
		return &LifecycleStateNotRegistered{rclError: rclError{rclRetCode: 3001, trace: string(stackTraceBuffer), context: errorsBuildContext(&LifecycleStateNotRegistered{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_ACTION_GOAL_ACCEPTED:
		return &ActionGoalAccepted{rclError: rclError{rclRetCode: 2100, trace: string(stackTraceBuffer), context: errorsBuildContext(&ActionGoalAccepted{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_ACTION_GOAL_REJECTED:
		return &ActionGoalRejected{rclError: rclError{rclRetCode: 2101, trace: string(stackTraceBuffer), context: errorsBuildContext(&ActionGoalRejected{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_ACTION_CLIENT_INVALID:
		return &ActionClientInvalid{rclError: rclError{rclRetCode: 2102, trace: string(stackTraceBuffer), context: errorsBuildContext(&ActionClientInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_ACTION_CLIENT_TAKE_FAILED:
		return &ActionClientTakeFailed{rclError: rclError{rclRetCode: 2103, trace: string(stackTraceBuffer), context: errorsBuildContext(&ActionClientTakeFailed{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_ACTION_SERVER_INVALID:
		return &ActionServerInvalid{rclError: rclError{rclRetCode: 2200, trace: string(stackTraceBuffer), context: errorsBuildContext(&ActionServerInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_ACTION_SERVER_TAKE_FAILED:
		return &ActionServerTakeFailed{rclError: rclError{rclRetCode: 2201, trace: string(stackTraceBuffer), context: errorsBuildContext(&ActionServerTakeFailed{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_ACTION_GOAL_HANDLE_INVALID:
		return &ActionGoalHandleInvalid{rclError: rclError{rclRetCode: 2300, trace: string(stackTraceBuffer), context: errorsBuildContext(&ActionGoalHandleInvalid{}, context, string(stackTraceBuffer))}}
	case C.RCL_RET_ACTION_GOAL_EVENT_INVALID:
		return &ActionGoalEventInvalid{rclError: rclError{rclRetCode: 2301, trace: string(stackTraceBuffer), context: errorsBuildContext(&ActionGoalEventInvalid{}, context, string(stackTraceBuffer))}}
	case C.RMW_RET_OK:
		return &RmwOk{rclError: rclError{rclRetCode: 0, trace: string(stackTraceBuffer), context: errorsBuildContext(&RmwOk{}, context, string(stackTraceBuffer))}}
	case C.RMW_RET_ERROR:
		return &RmwError{rclError: rclError{rclRetCode: 1, trace: string(stackTraceBuffer), context: errorsBuildContext(&RmwError{}, context, string(stackTraceBuffer))}}
	case C.RMW_RET_TIMEOUT:
		return &RmwTimeout{rclError: rclError{rclRetCode: 2, trace: string(stackTraceBuffer), context: errorsBuildContext(&RmwTimeout{}, context, string(stackTraceBuffer))}}
	case C.RMW_RET_UNSUPPORTED:
		return &RmwUnsupported{rclError: rclError{rclRetCode: 3, trace: string(stackTraceBuffer), context: errorsBuildContext(&RmwUnsupported{}, context, string(stackTraceBuffer))}}
	case C.RMW_RET_BAD_ALLOC:
		return &RmwBadAlloc{rclError: rclError{rclRetCode: 10, trace: string(stackTraceBuffer), context: errorsBuildContext(&RmwBadAlloc{}, context, string(stackTraceBuffer))}}
	case C.RMW_RET_INVALID_ARGUMENT:
		return &RmwInvalidArgument{rclError: rclError{rclRetCode: 11, trace: string(stackTraceBuffer), context: errorsBuildContext(&RmwInvalidArgument{}, context, string(stackTraceBuffer))}}
	case C.RMW_RET_INCORRECT_RMW_IMPLEMENTATION:
		return &RmwIncorrectRmwImplementation{rclError: rclError{rclRetCode: 12, trace: string(stackTraceBuffer), context: errorsBuildContext(&RmwIncorrectRmwImplementation{}, context, string(stackTraceBuffer))}}

	default:
		return &UnknownReturnCode{rclError: rclError{rclRetCode: int(rcl_ret_t), context: context}}
	}
}

type UnknownReturnCode struct {
	rclError
}

// AlreadyInit rcl specific ret codes start at 100rcl_init() already called return code.
type AlreadyInit struct {
	rclError
}

// NotInit rcl_init() not yet called return code.
type NotInit struct {
	rclError
}

// MismatchedRmwId Mismatched rmw identifier return code.
type MismatchedRmwId struct {
	rclError
}

// TopicNameInvalid Topic name does not pass validation.
type TopicNameInvalid struct {
	rclError
}

// ServiceNameInvalid Service name (same as topic name) does not pass validation.
type ServiceNameInvalid struct {
	rclError
}

// UnknownSubstitution Topic name substitution is unknown.
type UnknownSubstitution struct {
	rclError
}

// AlreadyShutdown rcl_shutdown() already called return code.
type AlreadyShutdown struct {
	rclError
}

// NotFound Resource not found
type NotFound struct {
	rclError
}

// NodeInvalid rcl node specific ret codes in 2XXInvalid rcl_node_t given return code.
type NodeInvalid struct {
	rclError
}

// NodeInvalidName Invalid node name return code.
type NodeInvalidName struct {
	rclError
}

// NodeInvalidNamespace Invalid node namespace return code.
type NodeInvalidNamespace struct {
	rclError
}

// NodeNameNonExistent Failed to find node name
type NodeNameNonExistent struct {
	rclError
}

// PublisherInvalid rcl publisher specific ret codes in 3XXInvalid rcl_publisher_t given return code.
type PublisherInvalid struct {
	rclError
}

// SubscriptionInvalid rcl subscription specific ret codes in 4XXInvalid rcl_subscription_t given return code.
type SubscriptionInvalid struct {
	rclError
}

// SubscriptionTakeFailed Failed to take a message from the subscription return code.
type SubscriptionTakeFailed struct {
	rclError
}

// ClientInvalid rcl service client specific ret codes in 5XXInvalid rcl_client_t given return code.
type ClientInvalid struct {
	rclError
}

// ClientTakeFailed Failed to take a response from the client return code.
type ClientTakeFailed struct {
	rclError
}

// ServiceInvalid rcl service server specific ret codes in 6XXInvalid rcl_service_t given return code.
type ServiceInvalid struct {
	rclError
}

// ServiceTakeFailed Failed to take a request from the service return code.
type ServiceTakeFailed struct {
	rclError
}

// TimerInvalid rcl timer specific ret codes in 8XXInvalid rcl_timer_t given return code.
type TimerInvalid struct {
	rclError
}

// TimerCanceled Given timer was canceled return code.
type TimerCanceled struct {
	rclError
}

// WaitSetInvalid rcl wait and wait set specific ret codes in 9XXInvalid rcl_wait_set_t given return code.
type WaitSetInvalid struct {
	rclError
}

// WaitSetEmpty Given rcl_wait_set_t is empty return code.
type WaitSetEmpty struct {
	rclError
}

// WaitSetFull Given rcl_wait_set_t is full return code.
type WaitSetFull struct {
	rclError
}

// InvalidRemapRule rcl argument parsing specific ret codes in 1XXXArgument is not a valid remap rule
type InvalidRemapRule struct {
	rclError
}

// WrongLexeme Expected one type of lexeme but got another
type WrongLexeme struct {
	rclError
}

// InvalidRosArgs Found invalid ros argument while parsing
type InvalidRosArgs struct {
	rclError
}

// InvalidParamRule Argument is not a valid parameter rule
type InvalidParamRule struct {
	rclError
}

// InvalidLogLevelRule Argument is not a valid log level rule
type InvalidLogLevelRule struct {
	rclError
}

// EventInvalid rcl event specific ret codes in 20XXInvalid rcl_event_t given return code.
type EventInvalid struct {
	rclError
}

// EventTakeFailed Failed to take an event from the event handle
type EventTakeFailed struct {
	rclError
}

// LifecycleStateRegistered rcl_lifecycle state register ret codes in 30XXrcl_lifecycle state registered
type LifecycleStateRegistered struct {
	rclError
}

// LifecycleStateNotRegistered rcl_lifecycle state not registered
type LifecycleStateNotRegistered struct {
	rclError
}

// ActionNameInvalid rcl action specific ret codes in 2XXXAction name does not pass validation return code.
type ActionNameInvalid struct {
	rclError
}

// ActionGoalAccepted Action goal accepted return code.
type ActionGoalAccepted struct {
	rclError
}

// ActionGoalRejected Action goal rejected return code.
type ActionGoalRejected struct {
	rclError
}

// ActionClientInvalid Action client is invalid return code.
type ActionClientInvalid struct {
	rclError
}

// ActionClientTakeFailed Action client failed to take response return code.
type ActionClientTakeFailed struct {
	rclError
}

// ActionServerInvalid Action server is invalid return code.
type ActionServerInvalid struct {
	rclError
}

// ActionServerTakeFailed Action server failed to take request return code.
type ActionServerTakeFailed struct {
	rclError
}

// ActionGoalHandleInvalid Action goal handle invalid return code.
type ActionGoalHandleInvalid struct {
	rclError
}

// ActionGoalEventInvalid Action invalid event return code.
type ActionGoalEventInvalid struct {
	rclError
}

// RmwOk Return code for rmw functionsThe operation ran as expected
type RmwOk struct {
	rclError
}

// RmwError Generic error to indicate operation could not complete successfully
type RmwError struct {
	rclError
}

// RmwTimeout The operation was halted early because it exceeded its timeout critera
type RmwTimeout struct {
	rclError
}

// RmwUnsupported The operation or event handling is not supported.
type RmwUnsupported struct {
	rclError
}

// RmwBadAlloc Failed to allocate memory
type RmwBadAlloc struct {
	rclError
}

// RmwInvalidArgument Argument to function was invalid
type RmwInvalidArgument struct {
	rclError
}

// RmwIncorrectRmwImplementation Incorrect rmw implementation.
type RmwIncorrectRmwImplementation struct {
	rclError
}

// RmwNodeNameNonExistent rmw node specific ret codes in 2XXFailed to find node nameUsing same return code than in rcl
type RmwNodeNameNonExistent struct {
	rclError
}

// Ok Success return code.
type Ok = RmwOk

// Error Unspecified error return code.
type Error = RmwError

// Timeout Timeout occurred return code.
type Timeout = RmwTimeout

// BadAlloc Failed to allocate memory return code.
type BadAlloc = RmwBadAlloc

// InvalidArgument Invalid argument return code.
type InvalidArgument = RmwInvalidArgument

// Unsupported Unsupported return code.
type Unsupported = RmwUnsupported
