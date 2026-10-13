package params_test

import (
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
)

func intValue(i int64) params.Value { return params.Value{Kind: params.KindInt64, Int64: i} }

// An on-set callback can read parameters, as in rclcpp; changing parameters
// from it fails instead of deadlocking.
func TestOnSetCallbackReadsAndCannotChange(t *testing.T) {
	pm, err := params.NewManager(rclgotest.Node(t, "callback", "/params_test"))
	if err != nil {
		t.Fatal(err)
	}
	if _, err := pm.Declare("a", intValue(1), params.Descriptor{}); err != nil {
		t.Fatal(err)
	}
	if _, err := pm.Declare("b", intValue(10), params.Descriptor{}); err != nil {
		t.Fatal(err)
	}
	var seenB int64
	var setErr, declareErr error
	var undeclared bool
	pm.OnSet(func(ps []params.Parameter) params.SetResult {
		b, _ := pm.Get("b")
		seenB = b.Value.Int64
		_ = pm.List()
		_, setErr = pm.Set(params.Parameter{Name: "b", Value: intValue(11)})
		_, declareErr = pm.Declare("c", intValue(0), params.Descriptor{})
		undeclared = pm.Undeclare("b") == nil
		return params.SetResult{Successful: ps[0].Value.Int64 < 100, Reason: "too large"}
	})

	done := make(chan struct{})
	go func() {
		defer close(done)
		if _, err := pm.Set(params.Parameter{Name: "a", Value: intValue(2)}); err != nil {
			t.Errorf("Set: %v", err)
		}
		if _, err := pm.Set(params.Parameter{Name: "a", Value: intValue(200)}); err == nil || !strings.Contains(err.Error(), "too large") {
			t.Errorf("Set rejected by the callback: %v", err)
		}
	}()
	select {
	case <-done:
	case <-time.After(rclgotest.Timeout):
		t.Fatal("Set deadlocked in the on-set callback")
	}
	if seenB != 10 {
		t.Errorf("the callback read b = %d, want 10", seenB)
	}
	for what, err := range map[string]error{"Set": setErr, "Declare": declareErr} {
		if err == nil || !strings.Contains(err.Error(), "from an on-set callback") {
			t.Errorf("%s from the callback: %v, want an error", what, err)
		}
	}
	if undeclared {
		t.Error("Undeclare from the callback succeeded")
	}
	if a, _ := pm.Get("a"); a.Value.Int64 != 2 {
		t.Errorf("a = %d, want 2", a.Value.Int64)
	}
	if b, _ := pm.Get("b"); b.Value.Int64 != 10 {
		t.Errorf("b = %d, want 10 (changed from the callback)", b.Value.Int64)
	}
}

// Changing the callback while other goroutines set parameters is safe (run
// with -race); a set from another goroutine waits for a running callback
// instead of failing.
func TestOnSetConcurrentWithSet(t *testing.T) {
	pm, err := params.NewManager(rclgotest.Node(t, "callback_race", "/params_test"))
	if err != nil {
		t.Fatal(err)
	}
	if _, err := pm.Declare("x", intValue(0), params.Descriptor{}); err != nil {
		t.Fatal(err)
	}
	var wg sync.WaitGroup
	for g := 0; g < 4; g++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := 0; i < 200; i++ {
				if _, err := pm.Set(params.Parameter{Name: "x", Value: intValue(int64(i))}); err != nil {
					t.Errorf("Set: %v", err)
					return
				}
			}
		}()
	}
	for i := 0; i < 200; i++ {
		pm.OnSet(func([]params.Parameter) params.SetResult {
			time.Sleep(10 * time.Microsecond)
			return params.SetResult{Successful: true}
		})
	}
	wg.Wait()
}

func TestSimTimeEnabled(t *testing.T) {
	pm, err := params.NewManager(rclgotest.Node(t, "sim_time_enabled", "/params_test"))
	if err != nil {
		t.Fatal(err)
	}
	if pm.SimTimeEnabled() {
		t.Fatal("use_sim_time is true by default")
	}
	if _, err := pm.SetValue("use_sim_time", params.Value{Kind: params.KindBool, Bool: true}); err != nil {
		t.Fatal(err)
	}
	if !pm.SimTimeEnabled() {
		t.Fatal("SimTimeEnabled after setting use_sim_time")
	}
}
