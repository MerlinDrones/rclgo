/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"fmt"
	"reflect"
	"testing"
	"time"

	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	test_msgs "github.com/merlindrones/rclgo/pkg/msgs/test_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Message fidelity, after rclcpp's test_serialized_message and system_tests'
// test_communication: every test_msgs type, filled with a distinct value in
// every field, must come back unchanged from serialization and from a real
// publisher/subscription pair.

var testMessageTypes = []struct {
	name string
	ts   types.MessageTypeSupport
}{
	{"Arrays", test_msgs.ArraysTypeSupport},
	{"BasicTypes", test_msgs.BasicTypesTypeSupport},
	{"BoundedPlainSequences", test_msgs.BoundedPlainSequencesTypeSupport},
	{"BoundedSequences", test_msgs.BoundedSequencesTypeSupport},
	{"Builtins", test_msgs.BuiltinsTypeSupport},
	{"Constants", test_msgs.ConstantsTypeSupport},
	{"Defaults", test_msgs.DefaultsTypeSupport},
	{"Empty", test_msgs.EmptyTypeSupport},
	{"MultiNested", test_msgs.MultiNestedTypeSupport},
	{"Nested", test_msgs.NestedTypeSupport},
	{"Strings", test_msgs.StringsTypeSupport},
	{"UnboundedSequences", test_msgs.UnboundedSequencesTypeSupport},
	{"WStrings", test_msgs.WStringsTypeSupport},
}

// filledMessage returns a new message of type ts with every field set to a
// distinct, non-default value. Sequences get two elements (bounded sequences
// in test_msgs allow up to three) and strings stay within the 22-byte bound;
// strings contain non-ASCII characters to exercise UTF-8 and UTF-16 (wstring)
// conversion.
func filledMessage(ts types.MessageTypeSupport) types.Message {
	msg := ts.New()
	n := 0
	fillValue(reflect.ValueOf(msg).Elem(), &n)
	return msg
}

func fillValue(v reflect.Value, n *int) {
	switch v.Kind() {
	case reflect.Struct:
		for i := 0; i < v.NumField(); i++ {
			if v.Type().Field(i).IsExported() {
				fillValue(v.Field(i), n)
			}
		}
		return
	case reflect.Array:
		for i := 0; i < v.Len(); i++ {
			fillValue(v.Index(i), n)
		}
		return
	case reflect.Slice:
		s := reflect.MakeSlice(v.Type(), 2, 2)
		for i := 0; i < 2; i++ {
			fillValue(s.Index(i), n)
		}
		v.Set(s)
		return
	}
	*n++
	switch v.Kind() {
	case reflect.Bool:
		v.SetBool(*n%2 == 1)
	case reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		v.SetInt(int64(-(*n % 100)))
	case reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64:
		v.SetUint(uint64(*n % 200))
	case reflect.Float32, reflect.Float64:
		v.SetFloat(float64(*n) + 0.25)
	case reflect.String:
		v.SetString(fmt.Sprintf("s%dé€", *n%1000))
	default:
		panic("unhandled field kind " + v.Kind().String())
	}
}

func TestSerializationRoundTrip(t *testing.T) {
	for _, tc := range testMessageTypes {
		t.Run(tc.name, func(t *testing.T) {
			in := filledMessage(tc.ts)
			if tc.name != "Empty" && tc.name != "Constants" { // no fields to fill
				require.NotEqual(t, tc.ts.New(), in, "the filler left the message at its defaults")
			}
			buf, err := rclgo.Serialize(in)
			require.NoError(t, err)
			out, err := rclgo.Deserialize(buf, tc.ts)
			require.NoError(t, err)
			require.Equal(t, in, out)
		})
	}
}

func TestPublishSubscribeRoundTrip(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("roundtrip", "/messages_test")
	require.NoError(t, err)
	startSpin(t, rclctx.Spin)
	for _, tc := range testMessageTypes {
		t.Run(tc.name, func(t *testing.T) {
			topic := "roundtrip_" + tc.name
			got := make(chan types.Message, 1)
			_, err := node.NewSubscription(topic, tc.ts, nil, func(s *rclgo.Subscription) {
				m := tc.ts.New()
				if _, err := s.TakeMessage(m); err == nil {
					select {
					case got <- m:
					default:
					}
				}
			})
			require.NoError(t, err)
			pub, err := node.NewPublisher(topic, tc.ts, nil)
			require.NoError(t, err)
			in := filledMessage(tc.ts)
			deadline := time.After(spinTimeout)
			for {
				require.NoError(t, pub.Publish(in))
				select {
				case out := <-got:
					require.Equal(t, in, out)
					return
				case <-deadline:
					t.Fatal("message never received")
				case <-time.After(50 * time.Millisecond):
				}
			}
		})
	}
}

// Deserialize rejects input too short to hold the CDR header, for every RMW
// (Cyclone DDS crashes on an empty buffer if it gets one).
func TestDeserializeRejectsShortInput(t *testing.T) {
	for _, n := range []int{0, 1, 3} {
		_, err := rclgo.Deserialize(make([]byte, n), test_msgs.ArraysTypeSupport)
		require.Error(t, err, "%d bytes", n)
	}
}

// PublishSerialized and TakeSerializedMessage move CDR bytes unchanged.
func TestSerializedPublishAndTake(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("serialized", "/messages_test")
	require.NoError(t, err)
	reliable := qos.NewDefault()
	reliable.History = qos.HistoryKeepAll
	sub, err := node.NewSubscription("serialized", test_msgs.ArraysTypeSupport, &rclgo.SubscriptionOptions{Qos: reliable}, nil)
	require.NoError(t, err)
	pub, err := node.NewPublisher("serialized", test_msgs.ArraysTypeSupport, &rclgo.PublisherOptions{Qos: reliable})
	require.NoError(t, err)
	in := filledMessage(test_msgs.ArraysTypeSupport)
	buf, err := rclgo.Serialize(in)
	require.NoError(t, err)

	requireEventually(t, "serialized message", func() error {
		if err := pub.PublishSerialized(buf); err != nil {
			return err
		}
		time.Sleep(20 * time.Millisecond)
		raw, info, err := sub.TakeSerializedMessage()
		if err != nil {
			return err
		}
		require.NotNil(t, info)
		require.Equal(t, buf, raw)
		out, err := rclgo.Deserialize(raw, test_msgs.ArraysTypeSupport)
		require.NoError(t, err)
		require.Equal(t, in, out)
		return nil
	})
}

// Invalid topic names are rejected (rcl's test_validate_topic_name).
func TestInvalidTopicNames(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("invalid_topics", "/messages_test")
	require.NoError(t, err)
	for _, topic := range []string{"", "bad topic", "/trailing/", "1starts_with_digit", "double//slash"} {
		_, err := node.NewPublisher(topic, std_msgs_msg.StringTypeSupport, nil)
		require.Error(t, err, "publisher on %q", topic)
		_, err = node.NewSubscription(topic, std_msgs_msg.StringTypeSupport, nil, func(*rclgo.Subscription) {})
		require.Error(t, err, "subscription on %q", topic)
	}
}
