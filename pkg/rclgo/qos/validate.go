/*
This file is part of rclgo

Copyright © 2025 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package qos

import (
	"errors"
	"fmt"
)

// Normalize mutates q to a canonical form.
func (q *Profile) Normalize() {
	switch q.History {
	case HistoryKeepLast:
		if q.Depth < 1 {
			q.Depth = 1
		}
	case HistoryKeepAll:
		q.Depth = 0
	}

	if q.Lifespan < 0 {
		q.Lifespan = 0
	}
	if q.Deadline < 0 {
		q.Deadline = 0
	}
	if q.LivelinessLeaseDuration < 0 {
		q.LivelinessLeaseDuration = 0
	}
}

// Validate checks internal consistency; call after Normalize.
func (q Profile) Validate() error {
	switch q.History {
	case HistoryKeepLast, HistoryKeepAll, HistorySystemDefault, HistoryUnknown:
	default:
		return fmt.Errorf("invalid History=%v", q.History)
	}
	if q.Depth < 0 {
		return errors.New("Depth must not be negative")
	}
	if q.History == HistoryKeepLast && q.Depth < 1 {
		return errors.New("KeepLast requires Depth >= 1")
	}

	switch q.Reliability {
	case ReliabilityBestEffort, ReliabilityReliable, ReliabilitySystemDefault, ReliabilityUnknown, ReliabilityBestAvailable:
	default:
		return fmt.Errorf("invalid Reliability=%v", q.Reliability)
	}
	switch q.Durability {
	case DurabilityVolatile, DurabilityTransientLocal, DurabilitySystemDefault, DurabilityUnknown, DurabilityBestAvailable:
	default:
		return fmt.Errorf("invalid Durability=%v", q.Durability)
	}
	switch q.Liveliness {
	case LivelinessAutomatic, LivelinessManualByTopic, LivelinessSystemDefault, LivelinessUnknown, LivelinessBestAvailable:
	default:
		return fmt.Errorf("invalid Liveliness=%v", q.Liveliness)
	}
	// Every time.Duration fits an rmw_time_t; DurationInfinite is
	// RMW_DURATION_INFINITE.
	return nil
}

// WithNormalized returns a normalized+validated copy.
func (q Profile) WithNormalized() (Profile, error) {
	cp := q
	cp.Normalize()
	return cp, cp.Validate()
}

// Compatibility helpers (subscriber perspective).
func IsReliabilityCompatible(requested, offered ReliabilityPolicy) bool {
	switch requested {
	case ReliabilityReliable:
		return offered == ReliabilityReliable
	case ReliabilityBestEffort:
		return offered == ReliabilityBestEffort || offered == ReliabilityReliable
	default:
		return false
	}
}

func IsDurabilityCompatible(requested, offered DurabilityPolicy) bool {
	switch requested {
	case DurabilityTransientLocal:
		return offered == DurabilityTransientLocal
	case DurabilityVolatile:
		return offered == DurabilityVolatile || offered == DurabilityTransientLocal
	default:
		return false
	}
}
