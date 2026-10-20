package rclgo

import (
	"errors"
	"fmt"
	"os"
	"reflect"
	"regexp"
	"strconv"
	"testing"

	"github.com/stretchr/testify/require"
)

// errorsCastC maps every return code in its generated switch to the error
// type named there, carrying the code, the context and (except for routine
// take failures) a stack trace; any other code gives UnknownReturnCode.
func TestErrorsCastCTable(t *testing.T) {
	src, err := os.ReadFile("errortypes.gen.go")
	require.NoError(t, err)
	cases := regexp.MustCompile(`case C\.(\w+):\n\s+return &(\w+)\{rclError: rclError\{rclRetCode: (\d+),`).FindAllSubmatch(src, -1)
	require.Greater(t, len(cases), 40, "cases parsed from errortypes.gen.go")
	takeFailed := map[string]bool{
		"RCL_RET_SUBSCRIPTION_TAKE_FAILED": true, "RCL_RET_CLIENT_TAKE_FAILED": true, "RCL_RET_SERVICE_TAKE_FAILED": true,
		"RCL_RET_ACTION_CLIENT_TAKE_FAILED": true, "RCL_RET_ACTION_SERVER_TAKE_FAILED": true, "RCL_RET_EVENT_TAKE_FAILED": true,
	}
	// Test files can't use cgo: call errorsCastC through reflect, converting
	// the code to C.rcl_ret_t.
	cast := func(code int, context string) (error, *rclError) {
		f := reflect.ValueOf(errorsCastC)
		out := f.Call([]reflect.Value{reflect.ValueOf(code).Convert(f.Type().In(0)), reflect.ValueOf(context)})
		err := out[0].Interface().(error)
		// Every error type is struct{ rclError }.
		base := (*rclError)(reflect.ValueOf(err).UnsafePointer())
		return err, base
	}
	seen := map[int]string{}
	for _, c := range cases {
		constant, typ := string(c[1]), string(c[2])
		code, err := strconv.Atoi(string(c[3]))
		require.NoError(t, err)
		require.NotContains(t, seen, code, "%s and %s share code %d", seen[code], constant, code)
		seen[code] = constant

		got, base := cast(code, "while testing "+constant)
		require.Equal(t, "*rclgo."+typ, fmt.Sprintf("%T", got), constant)
		require.Equal(t, code, base.rclRetCode, constant)
		require.Contains(t, got.Error(), "[*rclgo."+typ+"] while testing "+constant)
		if takeFailed[constant] {
			require.Empty(t, base.trace, "%s is routine and gets no stack trace", constant)
		} else {
			require.Contains(t, base.trace, "TestErrorsCastCTable", "stack trace of %s", constant)
		}
	}
	// Declared types that errorsCastC never returns, because their codes are
	// shared: RCL_RET_ACTION_NAME_INVALID has 2000, as RCL_RET_EVENT_INVALID
	// has, and RMW_RET_NODE_NAME_NON_EXISTENT has 203, as the rcl code has.
	// (The RCL_RET_* codes shared with rmw are aliases: Error = RmwError.)
	returned := map[string]bool{"UnknownReturnCode": true}
	for _, c := range cases {
		returned[string(c[2])] = true
	}
	var unreachable []string
	for _, m := range regexp.MustCompile(`(?m)^type (\w+) struct \{\n\s+rclError\n\}`).FindAllSubmatch(src, -1) {
		if !returned[string(m[1])] {
			unreachable = append(unreachable, string(m[1]))
		}
	}
	require.ElementsMatch(t, []string{"ActionNameInvalid", "RmwNodeNameNonExistent"}, unreachable)

	for _, code := range []int{-1, 3000 - 1, 99999} {
		if _, ok := seen[code]; ok {
			continue
		}
		got, base := cast(code, "unknown")
		var u *UnknownReturnCode
		require.True(t, errors.As(got, &u), "code %d: %T", code, got)
		require.Equal(t, code, base.rclRetCode)
	}
}
