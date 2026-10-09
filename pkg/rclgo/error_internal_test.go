package rclgo

import (
	"strings"
	"testing"
)

// Turning an rcl error into a Go error must clear rcl's error state, so that a
// later error does not carry the text of an earlier one and rcutils does not
// warn about overwriting it.
func TestRCLErrorStateIsResetAfterUse(t *testing.T) {
	ctx, err := NewContext(0, nil)
	if err != nil {
		t.Fatalf("NewContext: %v", err)
	}
	defer ctx.Close()
	// rcl sets its error state for an invalid node name.
	_, err = ctx.NewNode("bad name!", "")
	if err == nil {
		t.Fatal("an invalid node name must fail")
	}
	if strings.Contains(err.Error(), "error not set") {
		t.Fatalf("the error does not carry rcl's message: %v", err)
	}
	if rclErrorIsSet() {
		t.Fatal("rcl error state is still set after the error was returned")
	}
}
