/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgotest

import (
	"fmt"
	"math/rand"
	"os"
	"path/filepath"
	"sync"
	"syscall"
)

// Test binaries of different packages run at the same time (go test runs
// packages in parallel), each on its own DDS domain. A domain is reserved
// with an exclusive flock on a file in the temporary directory, which the
// kernel releases when the process exits, however it exits; another test
// process on this machine then doesn't pick it.

var reserved = struct {
	sync.Mutex
	files map[int]*os.File
}{files: map[int]*os.File{}}

// ReserveDomain returns a domain in 1-101 that no other process on this
// machine has reserved and that avoid doesn't contain, and keeps it reserved
// until ReleaseDomain or the end of the process. If every domain is taken, it
// returns a random one without reserving it.
func ReserveDomain(avoid map[int]bool) int {
	dir := filepath.Join(os.TempDir(), "rclgo-test-domains")
	_ = os.MkdirAll(dir, 0o777)
	reserved.Lock()
	defer reserved.Unlock()
	for _, i := range rand.Perm(101) { //nolint:gosec
		id := i + 1
		if avoid[id] || reserved.files[id] != nil {
			continue
		}
		f, err := os.OpenFile(filepath.Join(dir, fmt.Sprintf("%d.lock", id)), os.O_CREATE|os.O_RDONLY, 0o666)
		if err != nil {
			continue
		}
		if err := syscall.Flock(int(f.Fd()), syscall.LOCK_EX|syscall.LOCK_NB); err != nil {
			f.Close()
			continue
		}
		reserved.files[id] = f
		return id
	}
	return 1 + rand.Intn(101) //nolint:gosec
}

// ReleaseDomain releases a domain reserved with ReserveDomain.
func ReleaseDomain(id int) {
	reserved.Lock()
	defer reserved.Unlock()
	if f := reserved.files[id]; f != nil {
		f.Close() // releases the lock
		delete(reserved.files, id)
	}
}
