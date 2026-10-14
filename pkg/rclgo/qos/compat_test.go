package qos

import (
	"os"
	"testing"
	"time"
)

// The compatibility helpers agree with rmw_qos_profile_check_compatible on
// every reliability and durability combination, including the policies
// resolved later (system default, unknown, best available): they rule a pair
// out exactly when rmw reports an error.
func TestCompatibilityHelpersMatchRMW(t *testing.T) {
	if os.Getenv("RMW_IMPLEMENTATION") == "rmw_zenoh_cpp" {
		t.Skip("rmw_zenoh_cpp reports every pair compatible")
	}
	reliabilities := []ReliabilityPolicy{ReliabilitySystemDefault, ReliabilityReliable, ReliabilityBestEffort, ReliabilityUnknown, ReliabilityBestAvailable}
	durabilities := []DurabilityPolicy{DurabilitySystemDefault, DurabilityTransientLocal, DurabilityVolatile, DurabilityUnknown, DurabilityBestAvailable}
	base := NewDefault()
	for _, offered := range reliabilities {
		for _, requested := range reliabilities {
			pub, sub := base, base
			pub.Reliability, sub.Reliability = offered, requested
			c, reason, err := CheckCompatible(pub, sub)
			if err != nil {
				t.Fatal(err)
			}
			if got, want := IsReliabilityCompatible(requested, offered), c != CompatibilityError; got != want {
				t.Errorf("reliability offered %v requested %v: helper %v, rmw %v (%s)", offered, requested, got, c, reason)
			}
		}
	}
	for _, offered := range durabilities {
		for _, requested := range durabilities {
			pub, sub := base, base
			pub.Durability, sub.Durability = offered, requested
			c, reason, err := CheckCompatible(pub, sub)
			if err != nil {
				t.Fatal(err)
			}
			if got, want := IsDurabilityCompatible(requested, offered), c != CompatibilityError; got != want {
				t.Errorf("durability offered %v requested %v: helper %v, rmw %v (%s)", offered, requested, got, c, reason)
			}
		}
	}
}

// CheckCompatible also applies rmw's deadline and liveliness rules, which the
// helpers don't cover.
func TestCheckCompatibleDeadlineAndLiveliness(t *testing.T) {
	if os.Getenv("RMW_IMPLEMENTATION") == "rmw_zenoh_cpp" {
		t.Skip("rmw_zenoh_cpp reports every pair compatible")
	}
	explicit := func() Profile {
		p := NewDefault()
		p.Liveliness = LivelinessAutomatic
		p.Deadline, p.LivelinessLeaseDuration = DurationInfinite, DurationInfinite
		return p
	}
	pub, sub := explicit(), explicit()
	pub.Deadline, sub.Deadline = 2*time.Second, time.Second // offered deadline longer than requested
	if c, reason, _ := CheckCompatible(pub, sub); c != CompatibilityError {
		t.Errorf("deadline offered 2 s, requested 1 s: %v (%s), want error", c, reason)
	}
	pub, sub = explicit(), explicit()
	pub.Liveliness, sub.Liveliness = LivelinessAutomatic, LivelinessManualByTopic
	if c, reason, _ := CheckCompatible(pub, sub); c != CompatibilityError {
		t.Errorf("liveliness offered automatic, requested manual by topic: %v (%s), want error", c, reason)
	}
	if c, reason, _ := CheckCompatible(explicit(), explicit()); c != CompatibilityOK {
		t.Errorf("identical explicit profiles: %v (%s), want ok", c, reason)
	}
}

// Validate over every enum value: settable values pass; out-of-range values
// and the *Unknown policies (reported, never settable) are rejected.
func TestValidateEnumTables(t *testing.T) {
	for v := HistoryPolicy(0); v <= HistoryUnknown+1; v++ {
		p := NewDefault()
		p.History, p.Depth = v, 5
		p.Normalize()
		if err := p.Validate(); (err == nil) != (v < HistoryUnknown) {
			t.Errorf("History %d: Validate %v", v, err)
		}
	}
	for v := ReliabilityPolicy(0); v <= ReliabilityBestAvailable+1; v++ {
		p := NewDefault()
		p.Reliability = v
		if err := p.Validate(); (err == nil) != (v <= ReliabilityBestAvailable && v != ReliabilityUnknown) {
			t.Errorf("Reliability %d: Validate %v", v, err)
		}
	}
	for v := DurabilityPolicy(0); v <= DurabilityBestAvailable+1; v++ {
		p := NewDefault()
		p.Durability = v
		if err := p.Validate(); (err == nil) != (v <= DurabilityBestAvailable && v != DurabilityUnknown) {
			t.Errorf("Durability %d: Validate %v", v, err)
		}
	}
	for v := LivelinessPolicy(0); v <= LivelinessBestAvailable+1; v++ {
		p := NewDefault()
		p.Liveliness = v
		valid := v <= LivelinessBestAvailable && v != 2 && v != LivelinessUnknown // 2: the removed manual-by-node
		if err := p.Validate(); (err == nil) != valid {
			t.Errorf("Liveliness %d: Validate %v", v, err)
		}
	}
}

// Negative durations mean infinite, as in rmw_time_from_nsec.
func TestNegativeDurationsAreInfinite(t *testing.T) {
	sec, nsec := DurationToRMW(-time.Second)
	if got := DurationFromRMW(sec, nsec); got != DurationInfinite {
		t.Errorf("DurationToRMW(-1 s) = %d s %d ns, want infinite", sec, nsec)
	}
	p := NewDefault()
	p.Deadline, p.Lifespan, p.LivelinessLeaseDuration = -1, -time.Hour, -5*time.Second
	p.Normalize()
	if p.Deadline != DurationInfinite || p.Lifespan != DurationInfinite || p.LivelinessLeaseDuration != DurationInfinite {
		t.Errorf("Normalize: %v %v %v, want infinite", p.Deadline, p.Lifespan, p.LivelinessLeaseDuration)
	}
}
