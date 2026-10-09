package rclgo_test

import (
	"os"
	"testing"

	"github.com/merlindrones/rclgo/pkg/rclgo"
)

// ExpandTopicName must return an error, not panic, on any name.
func FuzzExpandTopicName(f *testing.F) {
	for _, s := range []string{"chatter", "~/x", "{node}/y", "{unknown}", "/a//b", "", "{", "}", "~", "{ns}{node}"} {
		f.Add(s)
	}
	f.Fuzz(func(t *testing.T, name string) {
		_, _ = rclgo.ExpandTopicName(name, "node", "/ns", map[string]string{"sub": "v"})
	})
}

// Deserialize must return an error, not crash, on arbitrary bytes: the
// buffer may come from the network (TakeSerializedMessage).
//
// Fast DDS (rmw_fastrtps_cpp 8.4.4, Fast-CDR 2.2.8, the Jazzy default)
// crashes in rmw_deserialize on a truncated buffer such as {0, 0, 0, 1} for
// most test_msgs types; rclpy crashes the same way, so this is not an rclgo
// defect. Run this target with another RMW, e.g.
// RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, which rejects such input cleanly.
//
// Fuzzing this target under Cyclone DDS has also driven a worker to over
// 20 GB of memory (likely a sequence length taken from the input before it is
// bounds-checked). Always fuzz it under a memory cap, e.g.
//
//	(ulimit -v 4000000; RMW_IMPLEMENTATION=rmw_cyclonedds_cpp go test -run '^$' -fuzz FuzzDeserialize -parallel 2 ./pkg/rclgo/)
func FuzzDeserialize(f *testing.F) {
	if rmw := os.Getenv("RMW_IMPLEMENTATION"); rmw == "" || rmw == "rmw_fastrtps_cpp" {
		f.Skip("rmw_fastrtps_cpp crashes on truncated input; set RMW_IMPLEMENTATION to another RMW")
	}
	for _, tc := range testMessageTypes {
		buf, err := rclgo.Serialize(filledMessage(tc.ts))
		if err != nil {
			f.Fatal(err)
		}
		f.Add(buf)
	}
	f.Add([]byte{})
	f.Add([]byte{0, 1, 0, 0, 0xff, 0xff, 0xff, 0x7f})
	types := []string{"BoundedSequences", "UnboundedSequences", "Strings", "WStrings", "MultiNested", "Arrays"}
	f.Fuzz(func(t *testing.T, buf []byte) {
		for _, tc := range testMessageTypes {
			for _, name := range types {
				if tc.name == name {
					_, _ = rclgo.Deserialize(buf, tc.ts)
				}
			}
		}
	})
}
