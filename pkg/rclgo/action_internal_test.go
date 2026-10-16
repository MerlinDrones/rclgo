package rclgo

import (
	"errors"
	"fmt"
	"strings"
	"testing"

	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

// A benign take failure must be recognized, also when wrapped, so that
// it is not logged as an error.
func TestIsActionServerTakeFailed(t *testing.T) {
	takeFailed := errorsCastC(2201, "take") // RCL_RET_ACTION_SERVER_TAKE_FAILED
	if !isActionServerTakeFailed(takeFailed) {
		t.Errorf("not recognized: %v", takeFailed)
	}
	if !isActionServerTakeFailed(fmt.Errorf("wrapped: %w", takeFailed)) {
		t.Error("wrapped take failure not recognized")
	}
	if isActionServerTakeFailed(errors.New("other")) {
		t.Error("unrelated error recognized as a take failure")
	}
	if isActionServerTakeFailed(errorsCastC(2103, "take")) { // RCL_RET_ACTION_CLIENT_TAKE_FAILED
		t.Error("client take failure recognized as a server take failure")
	}
}

// Logging for an unknown goal must not dereference a nil goal.
func TestGoalLogMessage(t *testing.T) {
	if msg := goalLogMessage(nil, "failed"); !strings.Contains(msg, "<unknown>") || !strings.HasSuffix(msg, "failed") {
		t.Errorf("unexpected message for nil goal: %q", msg)
	}
	var id types.GoalID
	id[0] = 1
	if msg := goalLogMessage(&id, "failed"); !strings.Contains(msg, id.String()) {
		t.Errorf("message does not contain goal ID: %q", msg)
	}
}
