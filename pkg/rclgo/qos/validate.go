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

	// Negative durations mean infinite, as rmw_time_from_nsec has it.
	if q.Lifespan < 0 {
		q.Lifespan = DurationInfinite
	}
	if q.Deadline < 0 {
		q.Deadline = DurationInfinite
	}
	if q.LivelinessLeaseDuration < 0 {
		q.LivelinessLeaseDuration = DurationInfinite
	}
}

// Validate checks internal consistency; call after Normalize. The *Unknown
// policies are only reported (by graph queries about a peer); they are not
// valid for creating an entity.
func (q Profile) Validate() error {
	switch q.History {
	case HistoryKeepLast, HistoryKeepAll, HistorySystemDefault:
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
	case ReliabilityBestEffort, ReliabilityReliable, ReliabilitySystemDefault, ReliabilityBestAvailable:
	default:
		return fmt.Errorf("invalid Reliability=%v", q.Reliability)
	}
	switch q.Durability {
	case DurabilityVolatile, DurabilityTransientLocal, DurabilitySystemDefault, DurabilityBestAvailable:
	default:
		return fmt.Errorf("invalid Durability=%v", q.Durability)
	}
	switch q.Liveliness {
	case LivelinessAutomatic, LivelinessManualByTopic, LivelinessSystemDefault, LivelinessBestAvailable:
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

// IsReliabilityCompatible reports whether a subscription requesting
// reliability requested can receive from a publisher offering offered. It is
// false only where rmw reports an error (reliable requested, best effort
// offered); policies resolved later (system default, unknown, best
// available) may match, so they are not ruled out. CheckCompatible gives
// rmw's full verdict, including deadline and liveliness.
func IsReliabilityCompatible(requested, offered ReliabilityPolicy) bool {
	return !(requested == ReliabilityReliable && offered == ReliabilityBestEffort)
}

// IsDurabilityCompatible is IsReliabilityCompatible for durability: false only
// for transient local requested and volatile offered.
func IsDurabilityCompatible(requested, offered DurabilityPolicy) bool {
	return !(requested == DurabilityTransientLocal && offered == DurabilityVolatile)
}
