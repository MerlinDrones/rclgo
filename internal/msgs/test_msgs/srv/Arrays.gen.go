/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
	http://www.apache.org/licenses/LICENSE-2.0
*/

// Code generated by rclgo-gen. DO NOT EDIT.

package test_msgs_srv

/*
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <test_msgs/srv/arrays.h>
*/
import "C"

import (
	"context"
	"errors"
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

func init() {
	typemap.RegisterService("test_msgs/Arrays", ArraysTypeSupport)
	typemap.RegisterService("test_msgs/srv/Arrays", ArraysTypeSupport)
}

type _ArraysTypeSupport struct{}

func (s _ArraysTypeSupport) Request() types.MessageTypeSupport {
	return Arrays_RequestTypeSupport
}

func (s _ArraysTypeSupport) Response() types.MessageTypeSupport {
	return Arrays_ResponseTypeSupport
}

func (s _ArraysTypeSupport) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__Arrays())
}

// Modifying this variable is undefined behavior.
var ArraysTypeSupport types.ServiceTypeSupport = _ArraysTypeSupport{}

// ArraysClient wraps rclgo.Client to provide type safe helper
// functions
type ArraysClient struct {
	*rclgo.Client
}

// NewArraysClient creates and returns a new client for the
// Arrays
func NewArraysClient(node *rclgo.Node, serviceName string, options *rclgo.ClientOptions) (*ArraysClient, error) {
	client, err := node.NewClient(serviceName, ArraysTypeSupport, options)
	if err != nil {
		return nil, err
	}
	return &ArraysClient{client}, nil
}

func (s *ArraysClient) Send(ctx context.Context, req *Arrays_Request) (*Arrays_Response, *rclgo.ServiceInfo, error) {
	msg, rmw, err := s.Client.Send(ctx, req)
	if err != nil {
		return nil, rmw, err
	}
	typedMessage, ok := msg.(*Arrays_Response)
	if !ok {
		return nil, rmw, errors.New("invalid message type returned")
	}
	return typedMessage, rmw, err
}

type ArraysServiceResponseSender struct {
	sender rclgo.ServiceResponseSender
}

func (s ArraysServiceResponseSender) SendResponse(resp *Arrays_Response) error {
	return s.sender.SendResponse(resp)
}

type ArraysServiceRequestHandler func(*rclgo.ServiceInfo, *Arrays_Request, ArraysServiceResponseSender)

// ArraysService wraps rclgo.Service to provide type safe helper
// functions
type ArraysService struct {
	*rclgo.Service
}

// NewArraysService creates and returns a new service for the
// Arrays
func NewArraysService(node *rclgo.Node, name string, options *rclgo.ServiceOptions, handler ArraysServiceRequestHandler) (*ArraysService, error) {
	h := func(rmw *rclgo.ServiceInfo, msg types.Message, rs rclgo.ServiceResponseSender) {
		m := msg.(*Arrays_Request)
		responseSender := ArraysServiceResponseSender{sender: rs}
		handler(rmw, m, responseSender)
	}
	service, err := node.NewService(name, ArraysTypeSupport, options, h)
	if err != nil {
		return nil, err
	}
	return &ArraysService{service}, nil
}
