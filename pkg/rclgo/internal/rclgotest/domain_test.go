/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgotest

import (
	"fmt"
	"os"
	"path/filepath"
	"syscall"
	"testing"
)

// lockedElsewhere reports whether another open file (as another process
// would have) can't lock domain id's file.
func lockedElsewhere(t *testing.T, id int) bool {
	t.Helper()
	f, err := os.Open(filepath.Join(os.TempDir(), "rclgo-test-domains", fmt.Sprintf("%d.lock", id)))
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	if err := syscall.Flock(int(f.Fd()), syscall.LOCK_EX|syscall.LOCK_NB); err != nil {
		return true
	}
	syscall.Flock(int(f.Fd()), syscall.LOCK_UN) //nolint:errcheck
	return false
}

// A reserved domain is in 1-101, differs from the other reserved and avoided
// ones, and is locked against other processes until it is released.
func TestReserveDomain(t *testing.T) {
	a := ReserveDomain(nil)
	defer ReleaseDomain(a)
	b := ReserveDomain(map[int]bool{50: true})
	defer ReleaseDomain(b)
	for _, id := range []int{a, b} {
		if id < 1 || id > 101 {
			t.Fatalf("domain %d outside 1-101", id)
		}
	}
	if a == b || b == 50 {
		t.Fatalf("reserved %d and %d (50 avoided)", a, b)
	}
	if !lockedElsewhere(t, a) {
		t.Fatalf("domain %d is not locked against other processes", a)
	}
	ReleaseDomain(a)
	if lockedElsewhere(t, a) {
		t.Fatalf("domain %d is still locked after ReleaseDomain", a)
	}
}
