/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package action_msgs_srv

/*
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <action_msgs/srv/cancel_goal.h>
*/
import "C"

import (
	"context"
	"errors"
	"unsafe"

	"github.com/PolibaX/rclgo/pkg/rclgo"
	"github.com/PolibaX/rclgo/pkg/rclgo/typemap"
	"github.com/PolibaX/rclgo/pkg/rclgo/types"
)

func init() {
	typemap.RegisterService("action_msgs/CancelGoal", CancelGoalTypeSupport)
	typemap.RegisterService("action_msgs/srv/CancelGoal", CancelGoalTypeSupport)
}

type _CancelGoalTypeSupport struct {}

func (s _CancelGoalTypeSupport) Request() types.MessageTypeSupport {
	return CancelGoal_RequestTypeSupport
}

func (s _CancelGoalTypeSupport) Response() types.MessageTypeSupport {
	return CancelGoal_ResponseTypeSupport
}

func (s _CancelGoalTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_service_type_support_handle__action_msgs__srv__CancelGoal())
}

// Modifying this variable is undefined behavior.
var CancelGoalTypeSupport types.ServiceTypeSupport = _CancelGoalTypeSupport{}

// CancelGoalClient wraps rclgo.Client to provide type safe helper
// functions
type CancelGoalClient struct {
	*rclgo.Client
}

// NewCancelGoalClient creates and returns a new client for the
// CancelGoal
func NewCancelGoalClient(node *rclgo.Node, serviceName string, options *rclgo.ClientOptions) (*CancelGoalClient, error) {
	client, err := node.NewClient(serviceName, CancelGoalTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &CancelGoalClient{client}, nil
}

func (s *CancelGoalClient) Send(ctx context.Context, req *CancelGoal_Request) (*CancelGoal_Response, *rclgo.ServiceInfo, error) {
	msg, rmw, err := s.Client.Send(ctx, req)
	if err != nil {
		return nil, rmw, err
	}
	typedMessage, ok := msg.(*CancelGoal_Response)
	if !ok {
		return nil, rmw, errors.New("invalid message type returned")
	}
	return typedMessage, rmw, err
}

type CancelGoalServiceResponseSender struct {
	sender rclgo.ServiceResponseSender
}

func (s CancelGoalServiceResponseSender) SendResponse(resp *CancelGoal_Response) error {
	return s.sender.SendResponse(resp)
}

type CancelGoalServiceRequestHandler func(*rclgo.ServiceInfo, *CancelGoal_Request, CancelGoalServiceResponseSender)

// CancelGoalService wraps rclgo.Service to provide type safe helper
// functions
type CancelGoalService struct {
	*rclgo.Service
}

// NewCancelGoalService creates and returns a new service for the
// CancelGoal
func NewCancelGoalService(node *rclgo.Node, name string, options *rclgo.ServiceOptions, handler CancelGoalServiceRequestHandler) (*CancelGoalService, error) {
	h := func(rmw *rclgo.ServiceInfo, msg types.Message, rs rclgo.ServiceResponseSender) {
		m := msg.(*CancelGoal_Request)
		responseSender := CancelGoalServiceResponseSender{sender: rs} 
		handler(rmw, m, responseSender)
	}
	service, err := node.NewService(name, CancelGoalTypeSupport, options, h)
	if err != nil {
		return nil, err
	}
	return &CancelGoalService{service}, nil
}