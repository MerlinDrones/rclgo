/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"context"
	"encoding/hex"
	"fmt"
	"strings"
	"sync"
	"testing"
	"time"

	action_tutorials "github.com/merlindrones/rclgo/pkg/msgs/action_tutorials_interfaces/action"
	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Interoperability with rclcpp and rclpy, after system_tests'
// test_communication: rclgo exchanges topics, services and actions with C++
// and Python nodes running as separate processes. Skipped with -short or
// without the ros2 tools.

const interopScripts = "testdata/interop/"

// Every test_msgs type, filled in every field, reaches an rclpy subscriber
// unchanged: what rclpy received, serialized by rclpy and deserialized by
// rclgo, equals what rclgo sent. (The raw bytes are not compared: CDR padding
// between fields is not initialized and differs.)
func TestInteropMessagesToPython(t *testing.T) {
	rclgotest.RequireROS2(t)
	for _, tc := range testMessageTypes {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			topic := "/interop/to_py_" + strings.ToLower(tc.name)
			echo := rclgotest.Python(t, interopScripts+"msg_echo.py", "test_msgs/msg/"+tc.name, topic)
			echo.WaitFor("READY")
			node := rclgotest.Node(t, "to_py_"+strings.ToLower(tc.name), "/interop")
			pub, err := node.NewPublisher(topic, tc.ts, nil)
			require.NoError(t, err)
			msg := filledMessage(tc.ts)
			rclgotest.Eventually(t, "message received by rclpy", func() error {
				if err := pub.Publish(msg); err != nil {
					return err
				}
				if !strings.Contains(echo.Output(), "HEX:") {
					return fmt.Errorf("not yet; output:\n%s", echo.Output())
				}
				return nil
			})
			got := strings.Fields(strings.SplitN(echo.Output(), "HEX:", 2)[1])[0]
			buf, err := hex.DecodeString(got)
			require.NoError(t, err)
			received, err := rclgo.Deserialize(buf, tc.ts)
			require.NoError(t, err)
			require.Equal(t, msg, received)
		})
	}
}

// Every test_msgs type published by rclpy arrives in rclgo unchanged.
func TestInteropMessagesFromPython(t *testing.T) {
	rclgotest.RequireROS2(t)
	for _, tc := range testMessageTypes {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			topic := "/interop/from_py_" + strings.ToLower(tc.name)
			want := filledMessage(tc.ts)
			buf, err := rclgo.Serialize(want)
			require.NoError(t, err)
			node := rclgotest.Node(t, "from_py_"+strings.ToLower(tc.name), "/interop")
			got := make(chan types.Message, 1)
			_, err = node.NewSubscription(topic, tc.ts, nil, func(s *rclgo.Subscription) {
				m := tc.ts.New()
				if _, err := s.TakeMessage(m); err == nil {
					select {
					case got <- m:
					default:
					}
				}
			})
			require.NoError(t, err)
			rclgotest.Spin(t, node.Context().Spin)
			rclgotest.Python(t, interopScripts+"msg_pub.py", "test_msgs/msg/"+tc.name, topic, hex.EncodeToString(buf))
			select {
			case m := <-got:
				require.Equal(t, want, m)
			case <-time.After(rclgotest.Timeout):
				t.Fatal("message from rclpy never arrived")
			}
		})
	}
}

// Topics with rclcpp: demo_nodes_cpp's talker reaches rclgo, and rclgo
// reaches its listener.
func TestInteropTopicsWithCpp(t *testing.T) {
	t.Run("talker to rclgo", func(t *testing.T) {
		t.Parallel()
		node := rclgotest.Node(t, "cpp_listener_go", "/interop")
		got := make(chan string, 1)
		_, err := node.NewSubscription("/interop/chatter_from_cpp", std_msgs_msg.StringTypeSupport, nil, func(s *rclgo.Subscription) {
			m := std_msgs_msg.NewString()
			if _, err := s.TakeMessage(m); err == nil {
				select {
				case got <- m.Data:
				default:
				}
			}
		})
		require.NoError(t, err)
		rclgotest.Spin(t, node.Context().Spin)
		rclgotest.RunROS2Node(t, "demo_nodes_cpp", "talker", "--ros-args", "-r", "chatter:=/interop/chatter_from_cpp")
		select {
		case data := <-got:
			require.True(t, strings.HasPrefix(data, "Hello World: "), "got %q", data)
		case <-time.After(rclgotest.Timeout):
			t.Fatal("no message from demo_nodes_cpp talker")
		}
	})
	t.Run("rclgo to listener", func(t *testing.T) {
		t.Parallel()
		listener := rclgotest.RunROS2Node(t, "demo_nodes_cpp", "listener", "--ros-args", "-r", "chatter:=/interop/chatter_to_cpp")
		node := rclgotest.Node(t, "cpp_talker_go", "/interop")
		pub, err := node.NewPublisher("/interop/chatter_to_cpp", std_msgs_msg.StringTypeSupport, nil)
		require.NoError(t, err)
		msg := std_msgs_msg.NewString()
		msg.Data = "hello from rclgo"
		rclgotest.Eventually(t, "demo_nodes_cpp listener hears rclgo", func() error {
			if err := pub.Publish(msg); err != nil {
				return err
			}
			if !strings.Contains(listener.Output(), "I heard: [hello from rclgo]") {
				return fmt.Errorf("output:\n%s", listener.Output())
			}
			return nil
		})
	})
}

func addTwoIntsService(t *testing.T, node *rclgo.Node, name string) {
	t.Helper()
	_, err := node.NewService(name, example_interfaces_srv.AddTwoIntsTypeSupport, nil,
		func(_ *rclgo.ServiceInfo, m types.Message, send rclgo.ServiceResponseSender) {
			req := m.(*example_interfaces_srv.AddTwoInts_Request)
			resp := example_interfaces_srv.NewAddTwoInts_Response()
			resp.Sum = req.A + req.B
			send.SendResponse(resp) //nolint:errcheck
		})
	require.NoError(t, err)
}

// Services with rclcpp and rclpy, both directions.
func TestInteropServices(t *testing.T) {
	for _, lang := range []string{"cpp", "py"} {
		t.Run("rclgo client to "+lang+" server", func(t *testing.T) {
			t.Parallel()
			name := "/interop/add_" + lang + "_server"
			rclgotest.RunROS2Node(t, "demo_nodes_"+lang, "add_two_ints_server", "--ros-args", "-r", "add_two_ints:="+name)
			node := rclgotest.Node(t, "add_client_"+lang, "/interop")
			client, err := node.NewClient(name, example_interfaces_srv.AddTwoIntsTypeSupport, nil)
			require.NoError(t, err)
			rclgotest.Spin(t, node.Context().Spin)
			req := example_interfaces_srv.NewAddTwoInts_Request()
			req.A, req.B = 40, 2
			resp := rclgotest.Call(t, client, req).(*example_interfaces_srv.AddTwoInts_Response)
			require.EqualValues(t, 42, resp.Sum)
		})
		t.Run(lang+" client to rclgo server", func(t *testing.T) {
			t.Parallel()
			name := "/interop/add_go_server_for_" + lang
			node := rclgotest.Node(t, "add_server_"+lang, "/interop")
			addTwoIntsService(t, node, name)
			rclgotest.Spin(t, node.Context().Spin)
			// The demo clients send 2 + 3 once the service is available.
			client := rclgotest.RunROS2Node(t, "demo_nodes_"+lang, "add_two_ints_client", "--ros-args", "-r", "add_two_ints:="+name)
			client.WaitFor("Result of add_two_ints")
			require.Regexp(t, `Result of add_two_ints: (for 2 \+ 3 = )?5\b`, client.Output())
		})
	}
	t.Run("ros2 service call to rclgo server", func(t *testing.T) {
		t.Parallel()
		rclgotest.RequireROS2(t)
		name := "/interop/add_go_server_for_cli"
		node := rclgotest.Node(t, "add_server_cli", "/interop")
		addTwoIntsService(t, node, name)
		rclgotest.Spin(t, node.Context().Spin)
		out, err := rclgotest.ROS2(t, "service", "call", name, "example_interfaces/srv/AddTwoInts", "{a: 4, b: 5}")
		require.NoError(t, err, out)
		require.Contains(t, out, "sum=9")
	})
}

// Actions with rclpy: rclgo's client against an rclpy server (feedback,
// result, cancellation), and `ros2 action send_goal` against rclgo's server.
func TestInteropActionsWithPython(t *testing.T) {
	rclgotest.RequireROS2(t)
	t.Run("rclgo client to rclpy server", func(t *testing.T) {
		t.Parallel()
		name := "/interop/fib_py_server"
		server := rclgotest.Python(t, interopScripts+"fibonacci_server.py", name)
		server.WaitFor("READY")
		node := rclgotest.Node(t, "fib_client", "/interop")
		client, err := node.NewActionClient(name, example_interfaces_action.FibonacciTypeSupport, nil)
		require.NoError(t, err)
		rclgotest.Spin(t, node.Context().Spin)

		var (
			mu       sync.Mutex
			feedback int
		)
		var result types.Message
		rclgotest.Eventually(t, "goal to rclpy server", func() error {
			ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
			defer cancel()
			var err error
			result, _, err = client.WatchGoal(ctx, fibGoal(6), func(context.Context, types.Message) {
				mu.Lock()
				feedback++
				mu.Unlock()
			})
			return err
		})
		res := result.(*example_interfaces_action.Fibonacci_GetResult_Response)
		require.EqualValues(t, rclgo.GoalSucceeded, res.Status)
		require.Equal(t, []int32{0, 1, 1, 2, 3, 5, 8}, res.Result.Sequence)
		mu.Lock()
		require.Positive(t, feedback, "no feedback from the rclpy server")
		mu.Unlock()
	})
	t.Run("ros2 action send_goal to rclgo server", func(t *testing.T) {
		t.Parallel()
		name := "/interop/fib_go_server"
		node := rclgotest.Node(t, "fib_server", "/interop")
		_, err := node.NewActionServer(name, rclgo.NewAction(example_interfaces_action.FibonacciTypeSupport,
			func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
				sender, err := goal.Accept()
				if err != nil {
					return nil, err
				}
				order := goal.Description.(*example_interfaces_action.Fibonacci_Goal).Order
				seq := []int32{0, 1}
				fb := example_interfaces_action.NewFibonacci_Feedback()
				for i := int32(1); i < order; i++ {
					seq = append(seq, seq[i]+seq[i-1])
					fb.Sequence = seq
					sender.Send(fb) //nolint:errcheck
					time.Sleep(20 * time.Millisecond)
				}
				return fibResult(seq...), nil
			}), nil)
		require.NoError(t, err)
		rclgotest.Spin(t, node.Context().Spin)
		out, err := rclgotest.ROS2(t, "action", "send_goal", "--feedback", name, "example_interfaces/action/Fibonacci", "{order: 5}")
		require.NoError(t, err, out)
		require.Contains(t, out, "Goal accepted")
		require.Contains(t, out, "Feedback:")
		require.Contains(t, out, "Goal finished with status: SUCCEEDED")
		require.Contains(t, out, "- 5")
	})
}

// Actions with rclcpp: rclgo's client against action_tutorials_cpp's
// fibonacci_action_server, and its fibonacci_action_client against an rclgo
// server.
func TestInteropActionsWithCpp(t *testing.T) {
	t.Run("rclgo client to rclcpp server", func(t *testing.T) {
		t.Parallel()
		name := "/interop/fib_cpp_server"
		rclgotest.RunROS2Node(t, "action_tutorials_cpp", "fibonacci_action_server", "--ros-args", "-r", "fibonacci:="+name)
		node := rclgotest.Node(t, "fib_client_cpp", "/interop")
		client, err := node.NewActionClient(name, action_tutorials.FibonacciTypeSupport, nil)
		require.NoError(t, err)
		rclgotest.Spin(t, node.Context().Spin)
		goal := action_tutorials.NewFibonacci_Goal()
		goal.Order = 4 // the tutorial server takes 1 s per element
		var (
			mu       sync.Mutex
			feedback int
			result   types.Message
		)
		rclgotest.Eventually(t, "goal to rclcpp server", func() error {
			ctx, cancel := context.WithTimeout(context.Background(), 8*time.Second)
			defer cancel()
			var err error
			result, _, err = client.WatchGoal(ctx, goal, func(context.Context, types.Message) {
				mu.Lock()
				feedback++
				mu.Unlock()
			})
			return err
		})
		res := result.(*action_tutorials.Fibonacci_GetResult_Response)
		require.EqualValues(t, rclgo.GoalSucceeded, res.Status)
		require.Equal(t, []int32{0, 1, 1, 2, 3}, res.Result.Sequence)
		mu.Lock()
		require.Positive(t, feedback, "no feedback from the rclcpp server")
		mu.Unlock()
	})
	t.Run("rclcpp client to rclgo server", func(t *testing.T) {
		t.Parallel()
		name := "/interop/fib_go_server_for_cpp"
		node := rclgotest.Node(t, "fib_server_cpp", "/interop")
		_, err := node.NewActionServer(name, rclgo.NewAction(action_tutorials.FibonacciTypeSupport,
			func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
				sender, err := goal.Accept()
				if err != nil {
					return nil, err
				}
				order := goal.Description.(*action_tutorials.Fibonacci_Goal).Order
				seq := []int32{0, 1}
				fb := action_tutorials.NewFibonacci_Feedback()
				for i := int32(1); i < order; i++ {
					seq = append(seq, seq[i]+seq[i-1])
					fb.PartialSequence = seq
					sender.Send(fb) //nolint:errcheck
					time.Sleep(10 * time.Millisecond)
				}
				r := action_tutorials.NewFibonacci_Result()
				r.Sequence = seq
				return r, nil
			}), nil)
		require.NoError(t, err)
		rclgotest.Spin(t, node.Context().Spin)
		// The tutorial client sends order 10 and logs the result.
		client := rclgotest.RunROS2Node(t, "action_tutorials_cpp", "fibonacci_action_client", "--ros-args", "-r", "fibonacci:="+name)
		client.WaitFor("Result received")
		require.Contains(t, client.Output(), "Goal accepted by server")
		require.Contains(t, client.Output(), "0 1 1 2 3 5 8 13 21 34 55")
	})
}

// QoS semantics across languages: a transient-local ("latched") message
// published by rclpy before rclgo subscribes reaches a transient-local rclgo
// subscriber, and a best-effort rclgo subscriber receives from a reliable
// rclpy publisher.
func TestInteropQoS(t *testing.T) {
	rclgotest.RequireROS2(t)
	receive := func(t *testing.T, node *rclgo.Node, topic string, profile qos.Profile) <-chan string {
		got := make(chan string, 1)
		_, err := node.NewSubscription(topic, std_msgs_msg.StringTypeSupport, &rclgo.SubscriptionOptions{Qos: profile},
			func(s *rclgo.Subscription) {
				m := std_msgs_msg.NewString()
				if _, err := s.TakeMessage(m); err == nil {
					select {
					case got <- m.Data:
					default:
					}
				}
			})
		require.NoError(t, err)
		return got
	}
	t.Run("transient local late joiner", func(t *testing.T) {
		t.Parallel()
		topic := "/interop/latched"
		// Publishes once, then keeps the publisher (and its history) alive.
		pub := rclgotest.Start(t, "ros2", "topic", "pub", "--once", "--keep-alive", "30", "--wait-matching-subscriptions", "0",
			"--qos-durability", "transient_local", "--qos-reliability", "reliable", topic, "std_msgs/msg/String", "{data: latched}")
		pub.WaitFor("publishing #1")
		time.Sleep(500 * time.Millisecond) // the message is published before rclgo exists
		node := rclgotest.Node(t, "latched_sub", "/interop")
		profile := qos.NewDefault()
		profile.Durability = qos.DurabilityTransientLocal
		got := receive(t, node, topic, profile)
		rclgotest.Spin(t, node.Context().Spin)
		select {
		case data := <-got:
			require.Equal(t, "latched", data)
		case <-time.After(rclgotest.Timeout):
			t.Fatal("the latched message did not reach a late transient-local subscriber")
		}
	})
	t.Run("best effort subscriber, reliable publisher", func(t *testing.T) {
		t.Parallel()
		topic := "/interop/reliable_to_best_effort"
		node := rclgotest.Node(t, "best_effort_sub", "/interop")
		got := receive(t, node, topic, qos.NewSensorDataProfile())
		rclgotest.Spin(t, node.Context().Spin)
		rclgotest.Start(t, "ros2", "topic", "pub", "--rate", "20", "--qos-reliability", "reliable", topic, "std_msgs/msg/String", "{data: reliable}")
		select {
		case data := <-got:
			require.Equal(t, "reliable", data)
		case <-time.After(rclgotest.Timeout):
			t.Fatal("no message from a reliable publisher")
		}
	})
}
