//go:build cgo

package qos

import (
	"testing"
	"time"
)

func TestProfile_AsCStruct_FromCStruct_RoundTrip(t *testing.T) {
	in := Profile{
		History:                      HistoryKeepLast,
		Depth:                        7,
		Reliability:                  ReliabilityReliable,
		Durability:                   DurabilityTransientLocal,
		Deadline:                     123 * time.Nanosecond,
		Lifespan:                     456 * time.Nanosecond,
		Liveliness:                   LivelinessAutomatic,
		LivelinessLeaseDuration:      789 * time.Nanosecond,
		AvoidROSNamespaceConventions: true,
	}
	out := roundTripProfileForTest(in)

	if out.History != in.History {
		t.Fatalf("History: got %v want %v", out.History, in.History)
	}
	if out.Depth != in.Depth {
		t.Fatalf("Depth: got %d want %d", out.Depth, in.Depth)
	}
	if out.Reliability != in.Reliability {
		t.Fatalf("Reliability: got %v want %v", out.Reliability, in.Reliability)
	}
	if out.Durability != in.Durability {
		t.Fatalf("Durability: got %v want %v", out.Durability, in.Durability)
	}
	if out.Deadline != in.Deadline {
		t.Fatalf("Deadline: got %v want %v", out.Deadline, in.Deadline)
	}
	if out.Lifespan != in.Lifespan {
		t.Fatalf("Lifespan: got %v want %v", out.Lifespan, in.Lifespan)
	}
	if out.Liveliness != in.Liveliness {
		t.Fatalf("Liveliness: got %v want %v", out.Liveliness, in.Liveliness)
	}
	if out.LivelinessLeaseDuration != in.LivelinessLeaseDuration {
		t.Fatalf("LivelinessLeaseDuration: got %v want %v", out.LivelinessLeaseDuration, in.LivelinessLeaseDuration)
	}
	if out.AvoidROSNamespaceConventions != in.AvoidROSNamespaceConventions {
		t.Fatalf("AvoidROSNamespaceConventions: got %v want %v", out.AvoidROSNamespaceConventions, in.AvoidROSNamespaceConventions)
	}
}
