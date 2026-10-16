package rclgo

/*
#include <rcl/logging.h>
#include <stdio.h>
*/
import "C"

import (
	"fmt"
	"os"
	"runtime/debug"
	"unsafe"
)

// LoggingOutputHandler is the function signature of logging output handling.
// Backwards compatibility is not guaranteed for this type alias. Use it only if
// necessary.
type LoggingOutputHandler = func(
	location unsafe.Pointer,
	severity int,
	name unsafe.Pointer,
	timestamp int64,
	format unsafe.Pointer,
	args unsafe.Pointer,
)

// GetLoggingOutputHandler returns the current logging output handler.
func GetLoggingOutputHandler() LoggingOutputHandler {
	return *currentLoggingOutputHandler.Load()
}

// SetLoggingOutputHandler sets the current logging output handler to h. If h ==
// nil, DefaultLoggingOutputHandler is used. h receives messages once logging is
// initialized (by Init, NewContext or InitLogging); before that rcutils prints
// messages with its own handler. A panic in h is printed to stderr and the
// message is dropped; it doesn't reach the logging call.
func SetLoggingOutputHandler(h LoggingOutputHandler) {
	if h == nil {
		h = DefaultLoggingOutputHandler
	}
	currentLoggingOutputHandler.Store(&h)
}

// DefaultLoggingOutputHandler is the logging output handler used by default,
// which logs messages based on ROS parameters used to initialize the logging
// system.
func DefaultLoggingOutputHandler(
	location unsafe.Pointer,
	severity int,
	name unsafe.Pointer,
	timestamp int64,
	format unsafe.Pointer,
	args unsafe.Pointer,
) {
	C.rcl_logging_multiple_output_handler(
		(*C.rcutils_log_location_t)(location),
		C.int(severity),
		(*C.char)(name),
		C.rcutils_time_point_value_t(timestamp),
		(*C.char)(format),
		(*C.va_list)(args),
	)

	// If realtime logging is enabled, flush stdout and stderr immediately
	// to ensure logs are written without buffering delays.
	if realtimeLogging.Load() {
		C.fflush(C.stdout)
		C.fflush(C.stderr)
	}
}

//export loggingOutputHandler
func loggingOutputHandler(
	location *C.rcutils_log_location_t,
	severity C.int,
	name *C.char,
	timestamp C.rcutils_time_point_value_t,
	format *C.char,
	args *C.va_list,
) {
	// rclgo_output_handler (logging.go) holds the logging mutex. A panic must
	// not unwind through the C frames: it would leave the mutex locked, and
	// every later log call on another thread would block.
	defer func() {
		if r := recover(); r != nil {
			fmt.Fprintf(os.Stderr, "rclgo: the logging output handler panicked: %v\n%s", r, debug.Stack())
		}
	}()
	(*currentLoggingOutputHandler.Load())(
		unsafe.Pointer(location),
		int(severity),
		unsafe.Pointer(name),
		int64(timestamp),
		unsafe.Pointer(format),
		unsafe.Pointer(args),
	)
}
