/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

// Package faultinject makes calls from rclgo to rcl, rcl_action, rmw and
// rcutils fail on demand, to test rclgo's error paths, as rclcpp does with
// mimick and rcutils fault injection (which the binary rcl packages are built
// without).
//
// Every such function rclgo calls that returns an error code is wrapped at
// link time (ld --wrap), in test binaries that import this package only. A
// wrapper passes the call through unless it is the call chosen with Arm; that
// call returns the chosen error code (NULL for functions returning a pointer)
// and sets the rcl error message
// "injected fault in <function>", without calling the real function. A
// failing *_fini function is the exception: the real function runs first,
// because rcl's fini functions release everything even when they report an
// error.
//
// The wrappers are generated: run go generate after rclgo starts calling a new
// function.
package faultinject

//go:generate go run ./gen

/*
#cgo LDFLAGS: -Wl,--wrap=rcutils_log
#include <stdlib.h>
#include "faultinject.h"
*/
import "C"

import (
	"errors"
	"fmt"
	"os"
	"runtime"
	"runtime/debug"
	"sort"
	"strings"
	"testing"
	"time"
	"unsafe"
)

// Error codes to inject (rcl_ret_t, which shares its first values with
// rmw_ret_t and rcutils_ret_t).
const (
	RetError    = 1
	RetTimeout  = 2
	RetBadAlloc = 10
)

// Arm makes the n-th wrapped call (from 1) after it fail with ret. If function
// is not empty, only calls to that function count.
func Arm(n int, function string, ret int) {
	cs := C.CString(function)
	defer C.free(unsafe.Pointer(cs))
	C.rclgo_fi_arm(C.long(n), cs, C.int(ret))
}

// Disarm stops injecting and returns the number of wrapped calls counted since
// Arm, and the function that was made to fail, or "" if none was.
func Disarm() (calls int, failed string) {
	var buf [128]C.char
	n := C.rclgo_fi_disarm(&buf[0], C.size_t(len(buf)))
	return int(n), C.GoString(&buf[0])
}

// Record runs f and returns how many wrapped calls to each function were
// made meanwhile, by f or by anything else in the process.
func Record(f func()) map[string]int {
	C.rclgo_fi_record(1)
	defer C.rclgo_fi_record(0)
	f()
	counts := map[string]int{}
	var name *C.char
	var count C.long
	for i := 0; C.rclgo_fi_recorded(C.int(i), &name, &count) != 0; i++ {
		counts[C.GoString(name)] = int(count)
	}
	return counts
}

// A Run is one run of a scenario with one call made to fail.
type Run struct {
	Function string  // the function whose call was made to fail
	Call     int     // which of its calls, from 1
	Hit      bool    // whether the scenario reached that call
	Errs     []error // the errors the scenario observed
	Logged   string  // the first message rclgo logged with the injected error
}

// Logged returns the first message logged since Arm that carries an injected
// error, or "".
func Logged() string {
	var buf [512]C.char
	C.rclgo_fi_logged(&buf[0], C.size_t(len(buf)))
	return C.GoString(&buf[0])
}

// Reported reports whether one of the errors the scenario observed is the
// injected one.
func (r Run) Reported() bool {
	for _, err := range r.Errs {
		if err != nil && strings.Contains(err.Error(), "injected fault in "+r.Function) {
			return true
		}
	}
	return false
}

func (r Run) String() string { return fmt.Sprintf("%s call %d", r.Function, r.Call) }

// Excluded lists functions Sweep does not make fail, with the reason.
var Excluded = map[string]string{}

// HangTimeout is how long a run may take before Sweep reports it as hung and
// ends the test binary, printing every goroutine.
var HangTimeout = 60 * time.Second

// Sweep runs scenario unarmed and records the wrapped calls it makes, then
// runs it again for each of the first maxCalls calls of each function it
// called, making that call fail with ret. scenario returns every error it
// observed: from rclgo calls, Spin, callbacks and Close. It must release
// everything it creates before it returns.
//
// Sweep fails t if the unarmed run observes an error, or if a run panics. A
// hung run ends the test binary. after, if not nil, is called after each run.
func Sweep(t *testing.T, maxCalls, ret int, scenario func(t *testing.T) []error, after func(t *testing.T, r Run)) []Run {
	t.Helper()
	var baseline []error
	counts := Record(func() { baseline = scenario(t) })
	if err := errors.Join(baseline...); err != nil {
		t.Fatalf("the scenario fails without injected faults: %v", err)
	}
	var runs []Run
	for _, fn := range sortedKeys(counts) {
		if why, ok := Excluded[fn]; ok {
			t.Logf("not failing %s: %s", fn, why)
			continue
		}
		for call := 1; call <= counts[fn] && call <= maxCalls; call++ {
			r := Run{Function: fn, Call: call}
			t.Run(r.String(), func(t *testing.T) {
				watchdog := time.AfterFunc(HangTimeout, func() {
					buf := make([]byte, 1<<22)
					n := runtime.Stack(buf, true)
					fmt.Fprintf(os.Stderr, "faultinject: %s: run hung for %v\n%s\n", r, HangTimeout, buf[:n])
					os.Exit(2)
				})
				defer watchdog.Stop()
				defer func() {
					if p := recover(); p != nil {
						Disarm()
						t.Errorf("%s: panic: %v\n%s", r, p, debug.Stack())
					}
				}()
				Arm(call, fn, ret)
				r.Errs = scenario(t)
				r.Logged = Logged()
				_, failed := Disarm()
				r.Hit = failed != ""
				if after != nil {
					after(t, r)
				}
			})
			runs = append(runs, r)
		}
	}
	return runs
}

func sortedKeys(m map[string]int) []string {
	keys := make([]string, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return keys
}
