package rclgo_test

import (
	"strings"
	"testing"

	test_msgs "github.com/merlindrones/rclgo/pkg/msgs/test_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// A bounded sequence longer than its bound is refused before it reaches rmw,
// on every rmw implementation (Cyclone DDS would send it, Fast DDS and Zenoh
// fail to publish it), as rclcpp refuses it on assignment. Bounded strings
// are not checked, as in rclcpp.

func overBound() *test_msgs.BoundedSequences {
	m := test_msgs.NewBoundedSequences()
	m.Int32Values = []int32{1, 2, 3, 4} // int32[<=3]
	return m
}

func TestBoundsCheckedBeforeSending(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("bounds", "/bounds_test")
	require.NoError(t, err)
	pub, err := node.NewPublisher("bounded", test_msgs.BoundedSequencesTypeSupport, nil)
	require.NoError(t, err)

	t.Run("publish", func(t *testing.T) {
		err := pub.Publish(overBound())
		require.ErrorIs(t, err, types.ErrBoundExceeded)
		require.Contains(t, err.Error(), "int32_values has 4 elements, more than its bound of 3")
		atBound := overBound()
		atBound.Int32Values = atBound.Int32Values[:3]
		require.NoError(t, pub.Publish(atBound))
	})
	t.Run("serialize", func(t *testing.T) {
		_, err := rclgo.Serialize(overBound())
		require.ErrorIs(t, err, types.ErrBoundExceeded)
	})
	t.Run("nested path", func(t *testing.T) {
		m := test_msgs.NewMultiNested()
		m.UnboundedSequenceOfBoundedSequences = []test_msgs.BoundedSequences{{}, *overBound()}
		err := m.CheckBounds()
		require.ErrorIs(t, err, types.ErrBoundExceeded)
		require.Contains(t, err.Error(), "unbounded_sequence_of_bounded_sequences[1].int32_values has 4 elements")
		m.BoundedSequenceOfArrays = make([]test_msgs.Arrays, 4) // Arrays[<=3]
		require.ErrorIs(t, m.CheckBounds(), types.ErrBoundExceeded)
	})
	t.Run("bounded strings unchecked", func(t *testing.T) {
		m := test_msgs.NewStrings()
		m.BoundedStringValue = strings.Repeat("x", 30) // string<=22
		require.NoError(t, m.CheckBounds())
	})
}
