package rclgo_test

import (
	"testing"
	"time"

	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	test_msgs "github.com/merlindrones/rclgo/pkg/msgs/test_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
)

// Benchmarks for the hot paths (after rclcpp's test/benchmark), with
// allocation counts, as a baseline for executor work.

func newBenchNode(b *testing.B, name string) *rclgo.Node {
	b.Helper()
	// No --log-level DEBUG here: per-call debug logging would dominate.
	rclctx, err := rclgo.NewContext(0, nil)
	if err != nil {
		b.Fatal(err)
	}
	b.Cleanup(func() { rclctx.Close() })
	node, err := rclctx.NewNode(name, "/bench")
	if err != nil {
		b.Fatal(err)
	}
	return node
}

func BenchmarkPublish(b *testing.B) {
	node := newBenchNode(b, "bench_publish")
	pub, err := node.NewPublisher("bench_publish", std_msgs_msg.StringTypeSupport, nil)
	if err != nil {
		b.Fatal(err)
	}
	msg := std_msgs_msg.NewString()
	msg.Data = "benchmark payload"
	b.ReportAllocs()
	for b.Loop() {
		if err := pub.Publish(msg); err != nil {
			b.Fatal(err)
		}
	}
}

// BenchmarkPublishTake publishes one message and takes it back by polling,
// without Spin: the cost of one message through rclgo and the RMW.
func BenchmarkPublishTake(b *testing.B) {
	node := newBenchNode(b, "bench_take")
	profile := qos.NewDefault()
	profile.History = qos.HistoryKeepAll
	sub, err := node.NewSubscription("bench_take", std_msgs_msg.StringTypeSupport, &rclgo.SubscriptionOptions{Qos: profile}, nil)
	if err != nil {
		b.Fatal(err)
	}
	pub, err := node.NewPublisher("bench_take", std_msgs_msg.StringTypeSupport, &rclgo.PublisherOptions{Qos: profile})
	if err != nil {
		b.Fatal(err)
	}
	in, out := std_msgs_msg.NewString(), std_msgs_msg.NewString()
	in.Data = "benchmark payload"
	roundTrip := func(timeout time.Duration) bool {
		if err := pub.Publish(in); err != nil {
			b.Fatal(err)
		}
		deadline := time.Now().Add(timeout)
		for time.Now().Before(deadline) {
			if info, err := sub.TakeMessage(out); err == nil && info != nil {
				return true
			}
		}
		return false
	}
	// Messages published before discovery completes are lost.
	warm := time.Now().Add(spinTimeout)
	for !roundTrip(50 * time.Millisecond) {
		if time.Now().After(warm) {
			b.Fatal("publisher and subscription never matched")
		}
	}
	for { // drain warm-up messages that arrived late
		if info, err := sub.TakeMessage(out); err != nil || info == nil {
			break
		}
	}
	b.ReportAllocs()
	for b.Loop() {
		if !roundTrip(time.Second) {
			b.Fatal("message lost")
		}
	}
}

func BenchmarkSerializeArrays(b *testing.B) {
	msg := filledMessage(test_msgs.ArraysTypeSupport)
	b.ReportAllocs()
	for b.Loop() {
		if _, err := rclgo.Serialize(msg); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkDeserializeArrays(b *testing.B) {
	buf, err := rclgo.Serialize(filledMessage(test_msgs.ArraysTypeSupport))
	if err != nil {
		b.Fatal(err)
	}
	b.ReportAllocs()
	for b.Loop() {
		if _, err := rclgo.Deserialize(buf, test_msgs.ArraysTypeSupport); err != nil {
			b.Fatal(err)
		}
	}
}
