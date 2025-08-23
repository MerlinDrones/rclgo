package qos

import "testing"

func TestNormalizeKeepLastDepth(t *testing.T) {
	q := Profile{
		History:     HistoryKeepLast,
		Depth:       0,
		Reliability: ReliabilityBestEffort,
		Durability:  DurabilityVolatile,
		Liveliness:  LivelinessAutomatic,
	}
	q.Normalize()
	if q.Depth != 1 {
		t.Fatalf("Depth expected 1, got %d", q.Depth)
	}
	if err := q.Validate(); err != nil {
		t.Fatalf("Validate failed: %v", err)
	}
}

func TestNormalizeKeepAllIgnoresDepth(t *testing.T) {
	q := Profile{
		History:     HistoryKeepAll,
		Depth:       42,
		Reliability: ReliabilityBestEffort,
		Durability:  DurabilityVolatile,
		Liveliness:  LivelinessAutomatic,
	}
	q.Normalize()
	if q.Depth != 0 {
		t.Fatalf("Depth expected 0 for KeepAll, got %d", q.Depth)
	}
}

func TestValidateRejectsInvalidEnums(t *testing.T) {
	q := Profile{
		History:     255, // invalid
		Depth:       1,
		Reliability: ReliabilityBestEffort,
		Durability:  DurabilityVolatile,
		Liveliness:  LivelinessAutomatic,
	}
	if err := q.Validate(); err == nil {
		t.Fatal("expected error for invalid History")
	}
}

func TestWithNormalized(t *testing.T) {
	q := Profile{
		History:     HistoryKeepLast,
		Depth:       0,
		Reliability: ReliabilityReliable,
		Durability:  DurabilityTransientLocal,
		Liveliness:  LivelinessAutomatic,
	}
	cp, err := q.WithNormalized()
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if cp.Depth != 1 {
		t.Fatalf("normalized Depth=1 expected, got %d", cp.Depth)
	}
}

func TestReliabilityCompatibility(t *testing.T) {
	if !IsReliabilityCompatible(ReliabilityBestEffort, ReliabilityReliable) {
		t.Fatal("BestEffort should accept Reliable offer")
	}
	if IsReliabilityCompatible(ReliabilityReliable, ReliabilityBestEffort) {
		t.Fatal("Reliable request must not accept BestEffort offer")
	}
}

func TestDurabilityCompatibility(t *testing.T) {
	if !IsDurabilityCompatible(DurabilityVolatile, DurabilityTransientLocal) {
		t.Fatal("Volatile should accept TransientLocal offer")
	}
	if IsDurabilityCompatible(DurabilityTransientLocal, DurabilityVolatile) {
		t.Fatal("TransientLocal request must not accept Volatile offer")
	}
}
