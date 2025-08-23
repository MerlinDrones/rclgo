// pkg/rclgo/qos/profiles_test.go
package qos

import "testing"

func TestNewClockProfile(t *testing.T) {
	p := NewClockProfile()
	if p.History != HistoryKeepLast || p.Depth != 1 {
		t.Fatalf("History/Depth=%v/%d", p.History, p.Depth)
	}
	if p.Reliability != ReliabilityBestEffort {
		t.Fatalf("Reliability=%v", p.Reliability)
	}
	if p.Durability != DurabilityVolatile {
		t.Fatalf("Durability=%v", p.Durability)
	}
	if p.Liveliness != LivelinessAutomatic {
		t.Fatalf("Liveliness=%v", p.Liveliness)
	}
}

func TestNewSensorDataProfile(t *testing.T) {
	p := NewSensorDataProfile()
	if p.History != HistoryKeepLast || p.Depth != 5 {
		t.Fatalf("History/Depth=%v/%d", p.History, p.Depth)
	}
	if p.Reliability != ReliabilityBestEffort {
		t.Fatalf("Reliability=%v", p.Reliability)
	}
	if p.Durability != DurabilityVolatile {
		t.Fatalf("Durability=%v", p.Durability)
	}
	if p.Liveliness != LivelinessAutomatic {
		t.Fatalf("Liveliness=%v", p.Liveliness)
	}
}

func TestNewParameterEventsProfile(t *testing.T) {
	p := NewParameterEventsProfile()
	if p.History != HistoryKeepAll || p.Depth != 0 {
		t.Fatalf("History/Depth=%v/%d", p.History, p.Depth)
	}
	if p.Reliability != ReliabilityReliable {
		t.Fatalf("Reliability=%v", p.Reliability)
	}
	if p.Durability != DurabilityTransientLocal {
		t.Fatalf("Durability=%v", p.Durability)
	}
	if p.Liveliness != LivelinessAutomatic {
		t.Fatalf("Liveliness=%v", p.Liveliness)
	}
}
