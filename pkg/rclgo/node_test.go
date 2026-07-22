package rclgo_test

import (
	"context"
	"testing"
	"time"

	std_msgs_msg2 "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func requireTopicNamesAndTypes(t *testing.T, node *rclgo.Node, expected map[string][]string) {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	for {
		actual, err := node.GetTopicNamesAndTypes(true)
		require.NoError(t, err)
		// Filter out system topics that vary between ROS distros (e.g. /rosout absent in Jazzy)
		filtered := make(map[string][]string, len(actual))
		for k, v := range actual {
			if k != "/rosout" {
				filtered[k] = v
			}
		}
		if assert.ObjectsAreEqualValues(expected, filtered) {
			return
		}
		select {
		case <-ctx.Done():
			require.EqualValues(t, expected, filtered)
		case <-time.After(100 * time.Millisecond):
		}
	}
}

// TestNodePublishesRosout is a regression test for
// https://github.com/MerlinDrones/rclgo/issues/13: rclgo nodes must appear as
// publishers on /rosout, matching rclcpp/rclpy behavior. Since rcl 9.x (Jazzy)
// rcl_node_init no longer creates this publisher, so NewNode must create it
// explicitly.
func TestNodePublishesRosout(t *testing.T) {
	setNewDomainID()

	rclctx, err := newDefaultRCLContext()
	require.NoError(t, err)
	defer rclctx.Close()

	node, err := rclctx.NewNode("rosout_node", "rosout_test")
	require.NoError(t, err)

	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	for {
		topics, err := node.GetTopicNamesAndTypes(true)
		require.NoError(t, err)
		if types, ok := topics["/rosout"]; ok {
			assert.Contains(t, types, "rcl_interfaces/msg/Log")
			return
		}
		select {
		case <-ctx.Done():
			t.Fatal("node did not create a /rosout publisher")
		case <-time.After(100 * time.Millisecond):
		}
	}
}

func TestNodeGetTopicNamesAndTypes(t *testing.T) {
	setNewDomainID()

	rclctx1, err := newDefaultRCLContext()
	require.NoError(t, err)
	defer rclctx1.Close()
	node1, err := rclctx1.NewNode("node1", "topic_names_and_types_test")
	require.NoError(t, err)

	rclctx2, err := newDefaultRCLContext()
	require.NoError(t, err)
	defer rclctx2.Close()
	node2, err := rclctx2.NewNode("node2", "topic_names_and_types_test")
	require.NoError(t, err)

	t.Log("node1 in empty network")
	requireTopicNamesAndTypes(t, node1, map[string][]string{})

	t.Log("node2 in empty network")
	requireTopicNamesAndTypes(t, node2, map[string][]string{})

	t.Log("new publisher")
	_, err = std_msgs_msg2.NewBoolPublisher(node1, "test_topic", nil)
	require.NoError(t, err)

	t.Log("node1 after publisher")
	requireTopicNamesAndTypes(t, node1, map[string][]string{
		"/topic_names_and_types_test/test_topic": {"std_msgs/msg/Bool"},
	})

	t.Log("node2 after publisher")
	requireTopicNamesAndTypes(t, node2, map[string][]string{
		"/topic_names_and_types_test/test_topic": {"std_msgs/msg/Bool"},
	})

	t.Log("new int publisher")
	intpub, err := std_msgs_msg2.NewInt64Publisher(node1, "test_topic2", nil)
	require.NoError(t, err)

	t.Log("node1 after creating int publisher")
	requireTopicNamesAndTypes(t, node1, map[string][]string{
		"/topic_names_and_types_test/test_topic":  {"std_msgs/msg/Bool"},
		"/topic_names_and_types_test/test_topic2": {"std_msgs/msg/Int64"},
	})

	t.Log("node2 after creating int publisher")
	requireTopicNamesAndTypes(t, node2, map[string][]string{
		"/topic_names_and_types_test/test_topic":  {"std_msgs/msg/Bool"},
		"/topic_names_and_types_test/test_topic2": {"std_msgs/msg/Int64"},
	})

	t.Log("publish int")
	err = intpub.Publish(std_msgs_msg2.NewInt64())
	require.NoError(t, err)

	t.Log("node1 after publishing int")
	requireTopicNamesAndTypes(t, node1, map[string][]string{
		"/topic_names_and_types_test/test_topic":  {"std_msgs/msg/Bool"},
		"/topic_names_and_types_test/test_topic2": {"std_msgs/msg/Int64"},
	})

	t.Log("node2 after publishing int")
	requireTopicNamesAndTypes(t, node2, map[string][]string{
		"/topic_names_and_types_test/test_topic":  {"std_msgs/msg/Bool"},
		"/topic_names_and_types_test/test_topic2": {"std_msgs/msg/Int64"},
	})

	t.Log("new string publisher")
	_, err = std_msgs_msg2.NewStringPublisher(node2, "test_topic", nil)
	require.NoError(t, err)

	t.Log("node1 after second publisher")
	requireTopicNamesAndTypes(t, node1, map[string][]string{
		"/topic_names_and_types_test/test_topic":  {"std_msgs/msg/Bool", "std_msgs/msg/String"},
		"/topic_names_and_types_test/test_topic2": {"std_msgs/msg/Int64"},
	})

	t.Log("node2 after second publisher")
	requireTopicNamesAndTypes(t, node2, map[string][]string{
		"/topic_names_and_types_test/test_topic":  {"std_msgs/msg/Bool", "std_msgs/msg/String"},
		"/topic_names_and_types_test/test_topic2": {"std_msgs/msg/Int64"},
	})
}
