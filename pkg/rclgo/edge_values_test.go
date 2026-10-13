package rclgo_test

import (
	"context"
	"fmt"
	"math"
	"reflect"
	"strings"
	"testing"
	"time"

	builtin_interfaces_msg "github.com/merlindrones/rclgo/pkg/msgs/builtin_interfaces/msg"
	test_msgs_action "github.com/merlindrones/rclgo/pkg/msgs/test_msgs/action"
	test_msgs "github.com/merlindrones/rclgo/pkg/msgs/test_msgs/msg"
	test_msgs_srv "github.com/merlindrones/rclgo/pkg/msgs/test_msgs/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Edge values through every path a message takes: serialization, a topic, a
// service and an action. After test_msgs' message_fixtures (used by rclcpp's
// and system_tests' communication tests): the min and max of every integer
// type, ±max, ±Inf, NaN, -0 and denormal floats, and empty, non-ASCII and
// at-the-bound strings and sequences.

// basicTypesEdges returns BasicTypes values that together hit the limits of
// every field.
func basicTypesEdges() []test_msgs.BasicTypes {
	return []test_msgs.BasicTypes{
		{},
		{
			BoolValue: true, ByteValue: math.MaxUint8, CharValue: math.MaxUint8,
			Float32Value: math.MaxFloat32, Float64Value: math.MaxFloat64,
			Int8Value: math.MaxInt8, Uint8Value: math.MaxUint8,
			Int16Value: math.MaxInt16, Uint16Value: math.MaxUint16,
			Int32Value: math.MaxInt32, Uint32Value: math.MaxUint32,
			Int64Value: math.MaxInt64, Uint64Value: math.MaxUint64,
		},
		{
			Float32Value: -math.MaxFloat32, Float64Value: -math.MaxFloat64,
			Int8Value: math.MinInt8, Int16Value: math.MinInt16,
			Int32Value: math.MinInt32, Int64Value: math.MinInt64,
		},
		{Float32Value: float32(math.Inf(1)), Float64Value: math.Inf(-1)},
		{Float32Value: float32(math.NaN()), Float64Value: math.NaN()},
		{Float32Value: math.SmallestNonzeroFloat32, Float64Value: math.Copysign(0, -1)},
	}
}

// edgeStrings are string values every string field must carry unchanged:
// empty, multi-byte UTF-8, a character outside the Basic Multilingual Plane
// (a surrogate pair in a wstring), quotes and backslashes, and a long value.
var edgeStrings = []string{
	"",
	"Hellö wörld € ハロー",
	"😀 outside the BMP",
	`quote " apostrophe ' backslash \ tab` + "\t" + "newline\n",
	strings.Repeat("long", 4096),
}

// requireSame is require.Equal that also treats NaN as equal to NaN and
// tells -0 from 0: it compares the Go syntax representation.
func requireSame(t *testing.T, want, got any, msgAndArgs ...any) {
	t.Helper()
	require.Equal(t, fmt.Sprintf("%#v", want), fmt.Sprintf("%#v", got), msgAndArgs...)
}

// copyFields sets every field of dst (a pointer to a struct) to the field of
// the same name in src, for moving values between message types with the
// same fields (BasicTypes, BasicTypes_Request, BasicTypes_Response).
func copyFields(dst, src any) {
	d := reflect.ValueOf(dst).Elem()
	s := reflect.Indirect(reflect.ValueOf(src))
	for i := 0; i < s.NumField(); i++ {
		if f := d.FieldByName(s.Type().Field(i).Name); f.IsValid() {
			f.Set(s.Field(i))
		}
	}
}

// edgeMessages returns test_msgs messages filled with edge values, keyed by a
// description.
func edgeMessages() map[string]types.Message {
	msgs := map[string]types.Message{}
	for i, b := range basicTypesEdges() {
		msgs[fmt.Sprintf("BasicTypes/%d", i)] = &b
	}
	for i, s := range edgeStrings {
		m := test_msgs.NewStrings()
		m.StringValue = s
		m.BoundedStringValue = strings.Repeat("x", 22) // exactly the bound
		msgs[fmt.Sprintf("Strings/%d", i)] = m
		w := test_msgs.NewWStrings()
		w.WstringValue = s
		w.ArrayOfWstrings = [3]string{s, "", "😀"}
		w.BoundedSequenceOfWstrings = []string{s, s, s} // at the bound of 3
		w.UnboundedSequenceOfWstrings = []string{s}
		msgs[fmt.Sprintf("WStrings/%d", i)] = w
	}
	edges := basicTypesEdges()
	bounded := test_msgs.NewBoundedSequences() // every sequence at its bound of 3
	bounded.BoolValues = []bool{true, false, true}
	bounded.ByteValues = []byte{0, 1, math.MaxUint8}
	bounded.CharValues = []byte{0, 1, math.MaxUint8}
	bounded.Float32Values = []float32{float32(math.Inf(1)), float32(math.NaN()), -math.MaxFloat32}
	bounded.Float64Values = []float64{math.Inf(-1), math.NaN(), math.SmallestNonzeroFloat64}
	bounded.Int8Values = []int8{math.MinInt8, 0, math.MaxInt8}
	bounded.Uint8Values = []uint8{0, 1, math.MaxUint8}
	bounded.Int16Values = []int16{math.MinInt16, 0, math.MaxInt16}
	bounded.Uint16Values = []uint16{0, 1, math.MaxUint16}
	bounded.Int32Values = []int32{math.MinInt32, 0, math.MaxInt32}
	bounded.Uint32Values = []uint32{0, 1, math.MaxUint32}
	bounded.Int64Values = []int64{math.MinInt64, 0, math.MaxInt64}
	bounded.Uint64Values = []uint64{0, 1, math.MaxUint64}
	bounded.StringValues = []string{edgeStrings[0], edgeStrings[1], edgeStrings[2]}
	bounded.BasicTypesValues = edges[1:4]
	bounded.ConstantsValues = make([]test_msgs.Constants, 3)
	bounded.DefaultsValues = make([]test_msgs.Defaults, 3)
	msgs["BoundedSequences/at bound"] = bounded
	msgs["BoundedSequences/empty"] = &test_msgs.BoundedSequences{}
	big := test_msgs.NewUnboundedSequences()
	big.Int64Values = make([]int64, 100000)
	for i := range big.Int64Values {
		big.Int64Values[i] = math.MinInt64 + int64(i)
	}
	big.BasicTypesValues = edges
	msgs["UnboundedSequences/100k"] = big
	builtins := test_msgs.NewBuiltins()
	builtins.DurationValue = builtin_interfaces_msg.Duration{Sec: math.MinInt32, Nanosec: 999999999}
	builtins.TimeValue = builtin_interfaces_msg.Time{Sec: math.MaxInt32, Nanosec: 999999999}
	msgs["Builtins/limits"] = builtins
	return msgs
}

func TestEdgeValuesSerializationRoundTrip(t *testing.T) {
	for name, in := range edgeMessages() {
		t.Run(name, func(t *testing.T) {
			buf, err := rclgo.Serialize(in)
			require.NoError(t, err)
			out, err := rclgo.Deserialize(buf, in.GetTypeSupport())
			require.NoError(t, err)
			requireSame(t, in, out)
		})
	}
}

func TestEdgeValuesPublishSubscribe(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("edge_values", "/edge_values_test")
	require.NoError(t, err)
	reliable := qos.NewDefault()
	reliable.History = qos.HistoryKeepAll
	type sub struct {
		pub *rclgo.Publisher
		got chan types.Message
	}
	subs := map[string]sub{} // one topic per message type
	for name, in := range edgeMessages() {
		typ := strings.SplitN(name, "/", 2)[0]
		if _, ok := subs[typ]; ok {
			continue
		}
		ts := in.GetTypeSupport()
		got := make(chan types.Message, 16)
		_, err := node.NewSubscription("edge_"+typ, ts, &rclgo.SubscriptionOptions{Qos: reliable}, func(s *rclgo.Subscription) {
			m := ts.New()
			if _, err := s.TakeMessage(m); err == nil {
				got <- m
			}
		})
		require.NoError(t, err)
		pub, err := node.NewPublisher("edge_"+typ, ts, &rclgo.PublisherOptions{Qos: reliable})
		require.NoError(t, err)
		subs[typ] = sub{pub, got}
	}
	startSpin(t, rclctx.Spin)
	for name, in := range edgeMessages() {
		t.Run(name, func(t *testing.T) {
			s := subs[strings.SplitN(name, "/", 2)[0]]
			want := fmt.Sprintf("%#v", in)
			deadline := time.After(spinTimeout)
			for {
				// Publish until this message arrives: the first ones may go
				// out before discovery, and earlier subtests' extra copies
				// may still be queued.
				require.NoError(t, s.pub.Publish(in))
				select {
				case out := <-s.got:
					if fmt.Sprintf("%#v", out) == want {
						return
					}
				case <-deadline:
					t.Fatal("message never received unchanged")
				case <-time.After(50 * time.Millisecond):
				}
			}
		})
	}
}

func TestEdgeValuesServices(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("edge_values_srv", "/edge_values_test")
	require.NoError(t, err)

	// Each service echoes its request in its response.
	_, err = test_msgs_srv.NewBasicTypesService(node, "basic_types", nil,
		func(_ *rclgo.ServiceInfo, req *test_msgs_srv.BasicTypes_Request, send test_msgs_srv.BasicTypesServiceResponseSender) {
			var resp test_msgs_srv.BasicTypes_Response
			copyFields(&resp, req)
			require.NoError(t, send.SendResponse(&resp))
		})
	require.NoError(t, err)
	_, err = test_msgs_srv.NewArraysService(node, "arrays", nil,
		func(_ *rclgo.ServiceInfo, req *test_msgs_srv.Arrays_Request, send test_msgs_srv.ArraysServiceResponseSender) {
			var resp test_msgs_srv.Arrays_Response
			copyFields(&resp, req)
			require.NoError(t, send.SendResponse(&resp))
		})
	require.NoError(t, err)
	_, err = test_msgs_srv.NewEmptyService(node, "empty", nil,
		func(_ *rclgo.ServiceInfo, _ *test_msgs_srv.Empty_Request, send test_msgs_srv.EmptyServiceResponseSender) {
			require.NoError(t, send.SendResponse(test_msgs_srv.NewEmpty_Response()))
		})
	require.NoError(t, err)
	basic, err := test_msgs_srv.NewBasicTypesClient(node, "basic_types", nil)
	require.NoError(t, err)
	arrays, err := test_msgs_srv.NewArraysClient(node, "arrays", nil)
	require.NoError(t, err)
	empty, err := test_msgs_srv.NewEmptyClient(node, "empty", nil)
	require.NoError(t, err)
	startSpin(t, rclctx.Spin)

	t.Run("Empty", func(t *testing.T) {
		resp := rclgotest.Call(t, empty.Client, test_msgs_srv.NewEmpty_Request())
		require.IsType(t, &test_msgs_srv.Empty_Response{}, resp)
	})
	for i, b := range basicTypesEdges() {
		t.Run(fmt.Sprintf("BasicTypes/%d", i), func(t *testing.T) {
			req := test_msgs_srv.NewBasicTypes_Request()
			copyFields(req, &b)
			req.StringValue = edgeStrings[i%len(edgeStrings)]
			resp := rclgotest.Call(t, basic.Client, req)
			want := test_msgs_srv.NewBasicTypes_Response()
			copyFields(want, req)
			requireSame(t, want, resp)
		})
	}
	t.Run("Arrays", func(t *testing.T) {
		edges := basicTypesEdges()
		req := test_msgs_srv.NewArrays_Request()
		req.BoolValues = [3]bool{true, false, true}
		req.CharValues = [3]byte{0, 1, math.MaxUint8}
		req.Float32Values = [3]float32{float32(math.Inf(-1)), float32(math.NaN()), math.SmallestNonzeroFloat32}
		req.Float64Values = [3]float64{math.Inf(1), math.Copysign(0, -1), -math.MaxFloat64}
		req.Int8Values = [3]int8{math.MinInt8, 0, math.MaxInt8}
		req.Uint16Values = [3]uint16{0, 1, math.MaxUint16}
		req.Int64Values = [3]int64{math.MinInt64, 0, math.MaxInt64}
		req.Uint64Values = [3]uint64{0, 1, math.MaxUint64}
		req.StringValues = [3]string{edgeStrings[0], edgeStrings[2], edgeStrings[4]}
		req.BasicTypesValues = [3]test_msgs.BasicTypes{edges[1], edges[2], edges[4]}
		resp := rclgotest.Call(t, arrays.Client, req)
		want := test_msgs_srv.NewArrays_Response()
		copyFields(want, req)
		requireSame(t, want, resp)
	})
}

func TestEdgeValuesAction(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("edge_values_action", "/edge_values_test")
	require.NoError(t, err)
	// The server echoes the goal as one feedback message and as the result.
	_, err = test_msgs_action.NewNestedMessageServer(node, "nested", test_msgs_action.NewNestedMessageAction(
		func(_ context.Context, h *test_msgs_action.NestedMessageGoalHandle) (*test_msgs_action.NestedMessage_Result, error) {
			fb, err := h.Accept()
			if err != nil {
				return nil, err
			}
			var feedback test_msgs_action.NestedMessage_Feedback
			copyFields(&feedback, h.Description)
			if err := fb.Send(&feedback); err != nil {
				return nil, err
			}
			var result test_msgs_action.NestedMessage_Result
			copyFields(&result, h.Description)
			return &result, nil
		}), nil)
	require.NoError(t, err)
	client, err := test_msgs_action.NewNestedMessageClient(node, "nested", nil)
	require.NoError(t, err)
	startSpin(t, rclctx.Spin)

	for i, b := range basicTypesEdges() {
		t.Run(fmt.Sprintf("%d", i), func(t *testing.T) {
			goal := test_msgs_action.NewNestedMessage_Goal()
			goal.NestedField = b
			goal.NestedFieldNoPkg.DurationValue = builtin_interfaces_msg.Duration{Sec: math.MinInt32, Nanosec: uint32(i)}
			goal.NestedFieldNoPkg.TimeValue = builtin_interfaces_msg.Time{Sec: math.MaxInt32, Nanosec: 999999999}
			goal.NestedDifferentPkg = builtin_interfaces_msg.Time{Sec: int32(-i), Nanosec: 1}
			var feedback test_msgs_action.NestedMessage_Feedback
			var result *test_msgs_action.NestedMessage_GetResult_Response
			requireEventually(t, "goal result", func() error {
				// A goal sent before discovery is lost: retry with a short
				// timeout.
				ctx, cancel := context.WithTimeout(context.Background(), time.Second)
				defer cancel()
				var err error
				result, _, err = client.WatchGoal(ctx, goal, func(_ context.Context, m *test_msgs_action.NestedMessage_FeedbackMessage) {
					feedback = m.Feedback
				})
				return err
			})
			var want test_msgs_action.NestedMessage_Result
			copyFields(&want, goal)
			requireSame(t, &want, &result.Result)
			var wantFeedback test_msgs_action.NestedMessage_Feedback
			copyFields(&wantFeedback, goal)
			// Feedback can race the result (the action protocol doesn't
			// order them), so it is only checked if it arrived.
			if feedback != (test_msgs_action.NestedMessage_Feedback{}) {
				requireSame(t, wantFeedback, feedback)
			}
		})
	}
}
