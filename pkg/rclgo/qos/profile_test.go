// pkg/rclgo/qos/profile_test.go
package qos

import (
	"testing"
	"time"
)

func TestNewSystemDefault(t *testing.T) {
	p := NewSystemDefault()
	if p.History != HistorySystemDefault {
		t.Fatalf("History=%v", p.History)
	}
	if p.Depth != 0 {
		t.Fatalf("Depth=%d", p.Depth)
	}
	if p.Reliability != ReliabilitySystemDefault {
		t.Fatalf("Reliability=%v", p.Reliability)
	}
	if p.Durability != DurabilitySystemDefault {
		t.Fatalf("Durability=%v", p.Durability)
	}
	if p.Deadline != DurationUnspecified {
		t.Fatalf("Deadline=%v", p.Deadline)
	}
	if p.Lifespan != DurationUnspecified {
		t.Fatalf("Lifespan=%v", p.Lifespan)
	}
	if p.Liveliness != LivelinessSystemDefault {
		t.Fatalf("Liveliness=%v", p.Liveliness)
	}
	if p.LivelinessLeaseDuration != DurationUnspecified {
		t.Fatalf("LivelinessLeaseDuration=%v", p.LivelinessLeaseDuration)
	}
	if p.AvoidROSNamespaceConventions {
		t.Fatalf("AvoidROSNamespaceConventions should default to false")
	}
}

func TestNewDefault(t *testing.T) {
	p := NewDefault()
	if p.History != HistoryKeepLast || p.Depth != 10 {
		t.Fatalf("History/Depth=%v/%d", p.History, p.Depth)
	}
	if p.Reliability != ReliabilityReliable {
		t.Fatalf("Reliability=%v", p.Reliability)
	}
	if p.Durability != DurabilityVolatile {
		t.Fatalf("Durability=%v", p.Durability)
	}
	if p.Deadline != 0 || p.Lifespan != 0 {
		t.Fatalf("Deadline/Lifespan=%v/%v", p.Deadline, p.Lifespan)
	}
	if p.Liveliness != LivelinessSystemDefault {
		t.Fatalf("Liveliness=%v", p.Liveliness)
	}
}

func TestNewKeepLastAndKeepAll(t *testing.T) {
	kl := NewKeepLast(7)
	if kl.History != HistoryKeepLast || kl.Depth != 7 {
		t.Fatalf("KeepLast History/Depth=%v/%d", kl.History, kl.Depth)
	}
	ka := NewKeepAll()
	if ka.History != HistoryKeepAll || ka.Depth != 0 {
		t.Fatalf("KeepAll History/Depth=%v/%d", ka.History, ka.Depth)
	}
}

func TestFluentWithers(t *testing.T) {
	base := NewDefault()

	// WithReliability should return a modified copy and not mutate base.
	modR := base.WithReliability(ReliabilityBestEffort)
	if modR.Reliability != ReliabilityBestEffort {
		t.Fatalf("WithReliability failed: %v", modR.Reliability)
	}
	if base.Reliability != ReliabilityReliable {
		t.Fatalf("base mutated: %v", base.Reliability)
	}

	// WithDurability copy semantics.
	modD := base.WithDurability(DurabilityTransientLocal)
	if modD.Durability != DurabilityTransientLocal {
		t.Fatalf("WithDurability failed: %v", modD.Durability)
	}
	if base.Durability != DurabilityVolatile {
		t.Fatalf("base mutated: %v", base.Durability)
	}

	// WithHistory sets both history and depth.
	modH := base.WithHistory(HistoryKeepAll, 0)
	if modH.History != HistoryKeepAll || modH.Depth != 0 {
		t.Fatalf("WithHistory failed: %v/%d", modH.History, modH.Depth)
	}

	// Sanity check: durations are just time.Duration; no surprises.
	modTime := base
	modTime.Deadline = 123 * time.Nanosecond
	if modTime.Deadline != 123*time.Nanosecond {
		t.Fatalf("Deadline not set as duration: %v", modTime.Deadline)
	}
}
