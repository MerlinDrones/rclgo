package rclgo

import (
	"context"
	"testing"
	"time"
)

// Context.Close closes a running wait set's own guard conditions as well as
// the wait set. If Run refreshes in between, the closed cancelWait drops out
// of the wait set and a later stop trigger goes unseen. Run must notice and
// return. This forces that order deterministically.
func TestRunReturnsWhenItsCancelGuardIsClosed(t *testing.T) {
	ctx, err := NewContext(0, nil)
	if err != nil {
		t.Fatalf("NewContext: %v", err)
	}
	defer ctx.Close()
	ws, err := ctx.NewWaitSet()
	if err != nil {
		t.Fatalf("NewWaitSet: %v", err)
	}
	if err := ws.followResources(&ctx.rosResourceStore); err != nil {
		t.Fatalf("followResources: %v", err)
	}
	done := make(chan error, 1)
	go func() { done <- ws.Run(context.Background()) }()
	time.Sleep(200 * time.Millisecond) // Run is waiting

	// Close the guard (deferred: Run holds it), then make Run refresh.
	if err := ws.cancelWait.Close(); err != nil {
		t.Fatalf("close cancelWait: %v", err)
	}
	if err := ws.refresh.Trigger(); err != nil {
		t.Fatalf("trigger refresh: %v", err)
	}
	time.Sleep(200 * time.Millisecond) // Run has refreshed
	ws.stop()

	select {
	case <-done:
	case <-time.After(5 * time.Second):
		ws.refresh.Trigger() //nolint:errcheck
		t.Fatal("Run did not return after its cancel guard was closed")
	}
	ws.Close() //nolint:errcheck
}
