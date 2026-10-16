package rclgo_test

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"testing"
	"time"

	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Graph queries, after rclcpp's test_node_graph. Discovery is asynchronous, so
// every check polls until the graph shows the expected state.

// requireEventually polls f until it returns nil or spinTimeout passes.
func requireEventually(t *testing.T, what string, f func() error) {
	t.Helper()
	deadline := time.Now().Add(spinTimeout)
	for {
		err := f()
		if err == nil {
			return
		}
		if time.Now().After(deadline) {
			t.Fatalf("%s: %v", what, err)
		}
		time.Sleep(50 * time.Millisecond)
	}
}

func requireNamesAndTypes(want map[string][]string, get func() (map[string][]string, error)) func() error {
	return func() error {
		got, err := get()
		if err != nil {
			return err
		}
		for name, types := range want {
			if !slices.Equal(got[name], types) {
				return fmt.Errorf("got %v", got)
			}
		}
		return nil
	}
}

func TestNodeGraph(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("graph_node", "/graph_ns")
	require.NoError(t, err)
	observerCtx := newSpinTestContext(t)
	observer, err := observerCtx.NewNode("graph_observer", "/graph_ns")
	require.NoError(t, err)

	_, err = node.NewPublisher("pub_topic", std_msgs_msg.StringTypeSupport, nil)
	require.NoError(t, err)
	_, err = node.NewSubscription("sub_topic", std_msgs_msg.Int32TypeSupport, nil, func(*rclgo.Subscription) {})
	require.NoError(t, err)
	_, err = node.NewService("add", example_interfaces_srv.AddTwoIntsTypeSupport, nil,
		func(*rclgo.ServiceInfo, types.Message, rclgo.ServiceResponseSender) {})
	require.NoError(t, err)
	_, err = node.NewClient("other_add", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
	require.NoError(t, err)
	_, err = node.NewActionServer("fib", rclgo.NewAction(example_interfaces_action.FibonacciTypeSupport,
		func(context.Context, *rclgo.GoalHandle) (types.Message, error) { return nil, errors.New("unused") }), nil)
	require.NoError(t, err)
	_, err = node.NewActionClient("other_fib", example_interfaces_action.FibonacciTypeSupport, nil)
	require.NoError(t, err)

	t.Run("node names", func(t *testing.T) {
		requireEventually(t, "graph_node not listed", func() error {
			names, namespaces, err := observer.GetNodeNames()
			if err != nil {
				return err
			}
			require.Len(t, namespaces, len(names))
			i := slices.Index(names, "graph_node")
			if i < 0 || namespaces[i] != "/graph_ns" {
				return fmt.Errorf("names %v, namespaces %v", names, namespaces)
			}
			return nil
		})
	})
	// demangle=true reports ROS names and types; false reports the raw DDS
	// names (rt/..., std_msgs::msg::dds_::String_).
	t.Run("publishers by node", func(t *testing.T) {
		requireEventually(t, "publisher", requireNamesAndTypes(
			map[string][]string{"/graph_ns/pub_topic": {"std_msgs/msg/String"}},
			func() (map[string][]string, error) {
				return observer.GetPublisherNamesAndTypesByNode(true, "graph_node", "/graph_ns")
			}))
	})
	t.Run("subscribers by node", func(t *testing.T) {
		requireEventually(t, "subscriber", requireNamesAndTypes(
			map[string][]string{"/graph_ns/sub_topic": {"std_msgs/msg/Int32"}},
			func() (map[string][]string, error) {
				return observer.GetSubscriberNamesAndTypesByNode(true, "graph_node", "/graph_ns")
			}))
	})
	t.Run("services by node", func(t *testing.T) {
		requireEventually(t, "service", requireNamesAndTypes(
			map[string][]string{"/graph_ns/add": {"example_interfaces/srv/AddTwoInts"}},
			func() (map[string][]string, error) {
				return observer.GetServiceNamesAndTypesByNode("graph_node", "/graph_ns")
			}))
	})
	t.Run("clients by node", func(t *testing.T) {
		requireEventually(t, "client", requireNamesAndTypes(
			map[string][]string{"/graph_ns/other_add": {"example_interfaces/srv/AddTwoInts"}},
			func() (map[string][]string, error) {
				return observer.GetClientNamesAndTypesByNode("graph_node", "/graph_ns")
			}))
	})
	t.Run("action servers by node", func(t *testing.T) {
		requireEventually(t, "action server", requireNamesAndTypes(
			map[string][]string{"/graph_ns/fib": {"example_interfaces/action/Fibonacci"}},
			func() (map[string][]string, error) {
				return observer.GetActionServerNamesAndTypesByNode("graph_node", "/graph_ns")
			}))
	})
	t.Run("action clients by node", func(t *testing.T) {
		requireEventually(t, "action client", requireNamesAndTypes(
			map[string][]string{"/graph_ns/other_fib": {"example_interfaces/action/Fibonacci"}},
			func() (map[string][]string, error) {
				return observer.GetActionClientNamesAndTypesByNode("graph_node", "/graph_ns")
			}))
	})
	t.Run("subscriptions info by topic", func(t *testing.T) {
		requireEventually(t, "subscription endpoint", func() error {
			infos, err := observer.GetSubscriptionsInfoByTopic("/graph_ns/sub_topic", true)
			if err != nil {
				return err
			}
			if len(infos) != 1 {
				return errors.New("no single endpoint")
			}
			require.Equal(t, "graph_node", infos[0].NodeName)
			require.Equal(t, "/graph_ns", infos[0].NodeNamespace)
			require.Equal(t, "std_msgs/msg/Int32", infos[0].TopicType)
			require.Equal(t, rclgo.EndpointSubscription, infos[0].EndpointType)
			return nil
		})
	})
	t.Run("unknown node", func(t *testing.T) {
		_, err := observer.GetPublisherNamesAndTypesByNode(false, "no_such_node", "/graph_ns")
		require.Error(t, err, "querying a node that does not exist must fail")
	})
}

// ExpandTopicName follows rcl's rules (after rclcpp's
// test_expand_topic_or_service_name).
func TestExpandTopicName(t *testing.T) {
	for _, tc := range []struct{ in, want string }{
		{"chatter", "/ns/chatter"},
		{"/chatter", "/chatter"},
		{"~/chatter", "/ns/talker/chatter"},
		{"{node}/chatter", "/ns/talker/chatter"},
		{"{ns}/chatter", "/ns/chatter"},
		{"{custom}/chatter", "/ns/value/chatter"},
	} {
		got, err := rclgo.ExpandTopicName(tc.in, "talker", "/ns", map[string]string{"custom": "value"})
		require.NoError(t, err, tc.in)
		require.Equal(t, tc.want, got, tc.in)
	}
	_, err := rclgo.ExpandTopicName("{unknown}/chatter", "talker", "/ns", nil)
	require.Error(t, err, "an unknown substitution must fail")
}
