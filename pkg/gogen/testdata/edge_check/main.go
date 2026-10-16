// Command check verifies generated bindings for rclgo_test_interfaces: constants,
// defaults, an in-process publish/subscribe round trip of a filled message, and
// that requests, responses, goals, feedback and results with a bounded sequence
// over its bound are refused.
// It prints "HEX:<serialized Edge>" and "SPELLINGS:<serialized default
// Spellings>" for cross-checking with rclpy.
package main

import (
	"context"
	"encoding/hex"
	"errors"
	"fmt"
	"os"
	"reflect"
	"time"

	geometry_msgs_msg "example.com/edge/msgs/geometry_msgs/msg"
	edge_action "example.com/edge/msgs/rclgo_test_interfaces/action"
	edge "example.com/edge/msgs/rclgo_test_interfaces/msg"
	edge_srv "example.com/edge/msgs/rclgo_test_interfaces/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

func check(ok bool, format string, a ...any) {
	if !ok {
		fmt.Printf("FAIL: "+format+"\n", a...)
		os.Exit(1)
	}
}

func filled() *edge.Edge {
	m := edge.NewEdge()
	m.Type, m.Range, m.Func, m.XYZ, m.Xyz = 1, 2, 3, 4, 5
	m.Url2 = "u2"
	m.WithDefault = -7
	m.StrDefault = "changed"
	m.ArrayDefault = [3]float64{0.5, -0.25, 1e10}
	m.StrsDefault = []string{"x", "y,z", ""}
	m.BoundedStr = "abcde"
	m.BoundedSeq = []int32{-1, 0, 1}
	m.BoundedStrs = []string{"abcd", "é"}
	m.Inner = edge.Inner{Value: -9223372036854775808, Name: "inner"}
	m.Inners = [2]edge.Inner{{Value: 1, Name: "a"}, {Value: 2, Name: "b"}}
	m.InnerSeq = []edge.Inner{{Value: 3, Name: "c"}}
	m.Point = geometry_msgs_msg.Point{X: 1.5, Y: -2.5, Z: 3.25}
	m.Points = []geometry_msgs_msg.Point{{X: 1}, {Y: 2}}
	m.Stamp.Sec, m.Stamp.Nanosec = 1700000000, 999999999
	m.Wide = "wide é€ 😀"
	return m
}

func main() {
	// Constants.
	check(edge.Edge_C_BOOL == true && edge.Edge_C_BYTE == 255 && edge.Edge_C_CHAR == 65, "bool/byte/char constants")
	check(edge.Edge_C_F32 == 1.5 && edge.Edge_C_F64 == -2.25, "float constants")
	check(edge.Edge_C_I8 == -128 && edge.Edge_C_U8 == 255 && edge.Edge_C_I16 == -32768 && edge.Edge_C_U16 == 65535, "8/16-bit constants")
	check(edge.Edge_C_I32 == -2147483648 && edge.Edge_C_U32 == 4294967295, "32-bit constants")
	check(edge.Edge_C_I64 == -9223372036854775808 && edge.Edge_C_U64 == 18446744073709551615, "64-bit constants")
	check(edge.Edge_C_STR == "string, with comma", "string constant %q", edge.Edge_C_STR)

	// Defaults.
	d := edge.NewEdge()
	check(d.WithDefault == 42, "int default %d", d.WithDefault)
	check(d.StrDefault == "hello, world", "string default %q", d.StrDefault)
	check(d.ArrayDefault == [3]float64{1.5, 2.5, 3.5}, "array default %v", d.ArrayDefault)
	check(reflect.DeepEqual(d.StrsDefault, []string{"a", "b,c"}), "string sequence default %q", d.StrsDefault)

	// Values rosidl accepts in spellings Go does not.
	check(edge.Spellings_C_TRUE && edge.Spellings_C_ONE, "bool constants True and 1")
	check(edge.Spellings_C_DEC == 10 && edge.Spellings_C_HEX == 16, "integer constants 010 and 0x10: %d %d", edge.Spellings_C_DEC, edge.Spellings_C_HEX)
	s := edge.NewSpellings()
	check(s.BTrue && !s.BZero, "bool defaults True and 0")
	check(s.IDec == 10 && s.UHex == 255 && s.IBin == -5, "integer defaults: %d %d %d", s.IDec, s.UHex, s.IBin)
	check(s.Bs == [2]bool{true, false}, "bool array default %v", s.Bs)
	check(reflect.DeepEqual(s.Ints, []int16{10, 16}), "integer sequence default %v", s.Ints)
	spellings, err := rclgo.Serialize(s)
	check(err == nil, "serialize Spellings: %v", err)

	// Serialization round trip.
	in := filled()
	buf, err := rclgo.Serialize(in)
	check(err == nil, "serialize: %v", err)
	out, err := rclgo.Deserialize(buf, edge.EdgeTypeSupport)
	check(err == nil, "deserialize: %v", err)
	check(reflect.DeepEqual(in, out), "round trip:\n in %+v\nout %+v", in, out)
	check(reflect.DeepEqual(in, in.Clone()), "Clone")

	// Publish/subscribe in process, and the service and action types.
	ctx, err := rclgo.NewContext(0, nil)
	check(err == nil, "context: %v", err)
	defer ctx.Close()
	node, err := ctx.NewNode("edge_check", "/edge")
	check(err == nil, "node: %v", err)
	got := make(chan *edge.Edge, 1)
	_, err = edge.NewEdgeSubscription(node, "edge", nil, func(m *edge.Edge, _ *rclgo.MessageInfo, err error) {
		if err == nil {
			select {
			case got <- m:
			default:
			}
		}
	})
	check(err == nil, "subscription: %v", err)
	pub, err := edge.NewEdgePublisher(node, "edge", nil)
	check(err == nil, "publisher: %v", err)
	// The service answers with an over-bound response, which must fail in
	// its handler; the action server sends over-bound feedback, which must
	// fail in the sender.
	responseErr := make(chan error, 1)
	_, err = edge_srv.NewOpService(node, "op", nil, func(_ *rclgo.ServiceInfo, _ *edge_srv.Op_Request, send edge_srv.OpServiceResponseSender) {
		resp := edge_srv.NewOp_Response()
		resp.Values = []int32{1, 2, 3} // int32[<=2]
		select {
		case responseErr <- send.SendResponse(resp):
		default:
		}
	})
	check(err == nil, "service: %v", err)
	feedbackErr := make(chan error, 1)
	_, err = edge_action.NewRunServer(node, "run", edge_action.NewRunAction(
		func(_ context.Context, h *edge_action.RunGoalHandle) (*edge_action.Run_Result, error) {
			fb, err := h.Accept()
			if err != nil {
				return nil, err
			}
			result := edge_action.NewRun_Result()
			if h.Description.Order == 99 { // asked for an over-bound result
				result.Last = []int32{1, 2, 3} // int32[<=2]
				return result, nil
			}
			over := edge_action.NewRun_Feedback()
			over.Partial = []int32{1, 2, 3} // int32[<=2]
			select {
			case feedbackErr <- fb.Send(over):
			default:
			}
			return result, nil
		}), nil)
	check(err == nil, "action server: %v", err)
	opClient, err := edge_srv.NewOpClient(node, "op", nil)
	check(err == nil, "client: %v", err)
	runClient, err := edge_action.NewRunClient(node, "run", nil)
	check(err == nil, "action client: %v", err)
	spin, cancel := context.WithCancel(context.Background())
	defer cancel()
	go ctx.Spin(spin) //nolint:errcheck

	// Over-bound requests and goals are refused before they are sent.
	overReq := edge_srv.NewOp_Request()
	overReq.Args = []int32{1, 2, 3}
	_, _, err = opClient.Send(context.Background(), overReq)
	check(errors.Is(err, types.ErrBoundExceeded), "over-bound request: %v", err)
	overGoal := edge_action.NewRun_Goal()
	overGoal.Limits = []int32{1, 2, 3}
	_, _, err = runClient.SendGoal(context.Background(), overGoal)
	check(errors.Is(err, types.ErrBoundExceeded), "over-bound goal: %v", err)
	// A request at the bound reaches the service (retried until discovery),
	// whose over-bound response fails in the handler; the client gets no
	// response, as with rclcpp, whose handler would throw.
	atBound := edge_srv.NewOp_Request()
	atBound.Args = []int32{1, 2}
	deadline := time.After(10 * time.Second)
	for done := false; !done; {
		c, stop := context.WithTimeout(context.Background(), 300*time.Millisecond)
		_, _, err = opClient.Send(c, atBound)
		stop()
		check(err != nil, "a response with an over-bound sequence was delivered")
		select {
		case err := <-responseErr:
			check(errors.Is(err, types.ErrBoundExceeded), "over-bound response: %v", err)
			done = true
		case <-deadline:
			check(false, "the request never reached the service")
		default:
		}
	}
	// A goal at the bound runs; its over-bound feedback fails in the sender.
	goal := edge_action.NewRun_Goal()
	goal.Limits = []int32{1, 2}
	for done := false; !done; {
		c, stop := context.WithTimeout(context.Background(), 300*time.Millisecond)
		_, _, err = runClient.SendGoal(c, goal)
		stop()
		select {
		case err := <-feedbackErr:
			check(errors.Is(err, types.ErrBoundExceeded), "over-bound feedback: %v", err)
			done = true
		case <-deadline:
			check(false, "the goal never reached the action server")
		case <-time.After(50 * time.Millisecond):
		}
	}
	// A result that can't be sent aborts the goal, so the client still gets
	// an answer.
	overResult := edge_action.NewRun_Goal()
	overResult.Order = 99
	c, stop := context.WithTimeout(context.Background(), 10*time.Second)
	_, _, err = runClient.WatchGoal(c, overResult, nil)
	stop()
	var failed *rclgo.GoalFailedError
	check(errors.As(err, &failed) && failed.Status == rclgo.GoalAborted, "over-bound result: want an aborted goal, got %v", err)

	deadline = time.After(10 * time.Second)
	for done := false; !done; {
		check(pub.Publish(in) == nil, "publish")
		select {
		case m := <-got:
			check(reflect.DeepEqual(in, m), "pub/sub:\n in %+v\nout %+v", in, m)
			done = true
		case <-deadline:
			check(false, "message never received")
		case <-time.After(50 * time.Millisecond):
		}
	}
	fmt.Println("HEX:" + hex.EncodeToString(buf))
	fmt.Println("SPELLINGS:" + hex.EncodeToString(spellings))
	fmt.Println("OK")
}
