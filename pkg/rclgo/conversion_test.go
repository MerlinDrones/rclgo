/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"testing"

	test_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/test_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// toCAndBack converts msg to its C struct, the path Publish and every service
// and action request or response takes, and optionally back into dst.
func toCAndBack(msg, dst types.Message) {
	ts := msg.GetTypeSupport()
	buf := ts.PrepareMemory()
	defer ts.ReleaseMemory(buf)
	ts.AsCStruct(buf, msg)
	if dst != nil {
		ts.AsGoStruct(dst, buf)
	}
}

// Converting messages to C frees everything it allocates: string fields,
// string and message sequences, and sequences that __init filled with default
// values (each of these leaked once per message before).
func TestMessageConversionDoesNotLeak(t *testing.T) {
	strs := test_msgs_msg.NewStrings()
	strs.StringValue = "a string longer than the empty default"
	wstrs := test_msgs_msg.NewWStrings()
	wstrs.WstringValue = "wide string"
	arrays := test_msgs_msg.NewArrays()
	arrays.StringValues = [3]string{"x", "yy", "zzz"}
	bounded := test_msgs_msg.NewBoundedSequences()
	bounded.StringValues = []string{"one", "two"}
	bounded.BasicTypesValues = make([]test_msgs_msg.BasicTypes, 2)
	unbounded := test_msgs_msg.NewUnboundedSequences()
	unbounded.StringValues = []string{"one", "two", "three"}
	unbounded.Int32Values = []int32{1, 2, 3}
	nested := test_msgs_msg.NewMultiNested()
	nested.UnboundedSequenceOfBoundedSequences = []test_msgs_msg.BoundedSequences{*bounded, *bounded}
	nested.BoundedSequenceOfArrays = []test_msgs_msg.Arrays{*arrays}
	for _, tc := range []struct {
		name string
		msg  types.Message
	}{
		{"Strings", strs}, {"WStrings", wstrs}, {"Arrays", arrays},
		{"BoundedSequences", bounded}, {"UnboundedSequences", unbounded}, {"MultiNested", nested},
	} {
		m := tc.msg
		t.Run(tc.name, func(t *testing.T) {
			for i := 0; i < 50; i++ { // let malloc's free lists settle
				toCAndBack(m, nil)
			}
			const n = 2000
			before := rclgotest.CHeapInUse()
			for i := 0; i < n; i++ {
				toCAndBack(m, nil)
			}
			grown := int64(rclgotest.CHeapInUse()) - int64(before)
			t.Logf("C heap grew by %d bytes over %d conversions (%.1f per conversion)", grown, n, float64(grown)/n)
			require.Less(t, grown, int64(16<<10), "conversions to C leak")
		})
	}
}

// A message reused for decoding loses the previous message's sequence elements
// when the next message's sequence is empty.
func TestEmptySequenceClearsReusedMessage(t *testing.T) {
	full := test_msgs_msg.NewUnboundedSequences()
	full.Int32Values = []int32{1, 2, 3}
	full.StringValues = []string{"a", "b"}
	full.BasicTypesValues = make([]test_msgs_msg.BasicTypes, 2)
	empty := test_msgs_msg.NewUnboundedSequences()

	dst := test_msgs_msg.NewUnboundedSequences()
	toCAndBack(full, dst)
	require.Len(t, dst.Int32Values, 3)
	toCAndBack(empty, dst)
	require.Empty(t, dst.Int32Values)
	require.Empty(t, dst.StringValues)
	require.Empty(t, dst.BasicTypesValues)
}
