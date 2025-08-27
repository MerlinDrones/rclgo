/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"context"
	"errors"
	"testing"

	"github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	"github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/msgs/std_srvs/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	. "github.com/smartystreets/goconvey/convey" //nolint:revive
)

type noOpAction struct{}

func (a *noOpAction) ExecuteGoal(_ context.Context, _ *rclgo.GoalHandle) (types.Message, error) {
	return nil, errors.New("not implemented")
}

func (a *noOpAction) TypeSupport() types.ActionTypeSupport {
	return example_interfaces_action.FibonacciTypeSupport
}

func TestContextClose(t *testing.T) {
	var context *rclgo.Context
	defer func() {
		if context != nil {
			context.Close()
		}
	}()
	SetDefaultFailureMode(FailureContinues)
	Convey(
		"Scenario: Does Context handle close resources correctly",
		t,
		func() {
			Convey("Given a context with resources", func() {
				var err error
				context, err = newDefaultRCLContext()
				So(err, ShouldBeNil)
				_, err = context.NewNode("node1", "/test/context_close")
				So(err, ShouldBeNil)
				node2, err := context.NewNode("node2", "/test/context_close")
				So(err, ShouldBeNil)
				_, err = context.NewWaitSet()
				So(err, ShouldBeNil)
				_, err = node2.NewClient(
					"client2",
					std_srvs_srv.EmptyTypeSupport,
					nil,
				)
				So(err, ShouldBeNil)
				_, err = context.NewNode("node3", "/test/context_close")
				So(err, ShouldBeNil)
				_, err = node2.NewPublisher(
					"/test_topic",
					std_msgs_msg.StringTypeSupport,
					nil,
				)
				So(err, ShouldBeNil)
				_, err = node2.NewClient(
					"client1",
					std_srvs_srv.EmptyTypeSupport,
					nil,
				)
				So(err, ShouldBeNil)
				_, err = node2.NewSubscription(
					"/test_topic",
					std_msgs_msg.StringTypeSupport,
					nil,
					func(s *rclgo.Subscription) {},
				)
				So(err, ShouldBeNil)
				_, err = node2.NewPublisher(
					"/test_topic2",
					std_msgs_msg.ColorRGBATypeSupport,
					nil,
				)
				So(err, ShouldBeNil)
				_, err = node2.NewService(
					"service1",
					std_srvs_srv.EmptyTypeSupport,
					nil,
					func(rsi *rclgo.ServiceInfo, rm types.Message, srs rclgo.ServiceResponseSender) {},
				)
				So(err, ShouldBeNil)
				_, err = node2.NewService(
					"service2",
					std_srvs_srv.EmptyTypeSupport,
					nil,
					func(rsi *rclgo.ServiceInfo, rm types.Message, srs rclgo.ServiceResponseSender) {},
				)
				So(err, ShouldBeNil)
				_, err = node2.NewActionServer(
					"action1",
					&noOpAction{},
					nil,
				)
				So(err, ShouldBeNil)
				_, err = node2.NewActionClient(
					"action1",
					example_interfaces_action.FibonacciTypeSupport,
					nil,
				)
				So(err, ShouldBeNil)
			})
			Convey(
				"When the context is closed the first time, no errors occur",
				func() {
					So(context.Close(), ShouldBeNil)
				},
			)
			Convey(
				"When the context is closed the second time, an error occurs",
				func() {
					So(context.Close(), ShouldNotBeNil)
				},
			)
		},
	)
}
