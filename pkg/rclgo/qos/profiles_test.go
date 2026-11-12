/*
This file is part of rclgo

Copyright © 2025 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

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

func TestNewBestAvailableProfile(t *testing.T) {
	p := NewBestAvailableProfile()
	if p.History != HistoryKeepLast || p.Depth != 10 {
		t.Fatalf("History/Depth=%v/%d", p.History, p.Depth)
	}
	if p.Reliability != ReliabilityBestAvailable {
		t.Fatalf("Reliability=%v", p.Reliability)
	}
	if p.Durability != DurabilityBestAvailable {
		t.Fatalf("Durability=%v", p.Durability)
	}
	if p.Liveliness != LivelinessBestAvailable {
		t.Fatalf("Liveliness=%v", p.Liveliness)
	}
}

func TestNewParameterEventsProfile(t *testing.T) {
	p := NewParameterEventsProfile()
	// rmw_qos_profile_parameter_events
	if p.History != HistoryKeepLast || p.Depth != 1000 {
		t.Fatalf("History/Depth=%v/%d", p.History, p.Depth)
	}
	if p.Reliability != ReliabilityReliable {
		t.Fatalf("Reliability=%v", p.Reliability)
	}
	if p.Durability != DurabilityVolatile {
		t.Fatalf("Durability=%v", p.Durability)
	}
	if p.Liveliness != LivelinessSystemDefault {
		t.Fatalf("Liveliness=%v", p.Liveliness)
	}
}
