/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo

/*
#include <rcl/logging.h>
#include <stdio.h>

const rcutils_log_location_t zero_location = {
    .function_name = "",
    .file_name = "",
    .line_number = 0
};

#include <pthread.h>
#include <rcl/logging_rosout.h>
#include <rcl/node.h>

// The Go output handler (logginghandler.go).
extern void loggingOutputHandler(rcutils_log_location_t* location, int severity, char* name,
    rcutils_time_point_value_t timestamp, char* format, va_list* args);

// rcl's and rcutils' logging state (the output handler, logger levels and the
// process-wide table of /rosout publishers) is not thread-safe. Like rclcpp's
// global logging mutex, this mutex serializes every use of it. It is recursive
// because rcl logs from inside these calls, on the same thread, which re-enters
// the output handler.
static pthread_mutex_t rclgo_logging_mutex;
static pthread_once_t rclgo_logging_mutex_once = PTHREAD_ONCE_INIT;

static void rclgo_logging_mutex_init(void) {
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&rclgo_logging_mutex, &attr);
    pthread_mutexattr_destroy(&attr);
}

static void rclgo_logging_lock(void) {
    pthread_once(&rclgo_logging_mutex_once, rclgo_logging_mutex_init);
    pthread_mutex_lock(&rclgo_logging_mutex);
}

static void rclgo_logging_unlock(void) {
    pthread_mutex_unlock(&rclgo_logging_mutex);
}

static void rclgo_output_handler(const rcutils_log_location_t* location, int severity, const char* name,
    rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
    rclgo_logging_lock();
    loggingOutputHandler((rcutils_log_location_t*)location, severity, (char*)name, timestamp, (char*)format, args);
    rclgo_logging_unlock();
}

static rcl_ret_t rclgo_logging_configure(const rcl_arguments_t* args, rcl_allocator_t allocator) {
    rclgo_logging_lock();
    rcl_ret_t rc = rcl_logging_configure(args, &allocator);
    if (rc == RCL_RET_OK) {
        rcutils_logging_set_output_handler(rclgo_output_handler);
    }
    rclgo_logging_unlock();
    return rc;
}

static rcl_ret_t rclgo_logging_fini(void) {
    rclgo_logging_lock();
    rcl_ret_t rc = rcl_logging_fini();
    rclgo_logging_unlock();
    return rc;
}

// Variable argument functions can't be called from Go so a wrapper is required.
// rcutils_log looks up the logger's level in rcutils' logger table before it
// calls the output handler, so the whole call holds the mutex: a concurrent
// SetLevel changes that table.
void rcutils_log_wrapper(
    const rcutils_log_location_t* location,
    int severity,
    const char* name,
    const char* format,
    const char* msg
) {
    rclgo_logging_lock();
    rcutils_log(location, severity, name, format, msg);
    rclgo_logging_unlock();
}

static int rclgo_get_logger_level(const char* name) {
    rclgo_logging_lock();
    int level = rcutils_logging_get_logger_level(name);
    rclgo_logging_unlock();
    return level;
}

static int rclgo_get_logger_effective_level(const char* name) {
    rclgo_logging_lock();
    int level = rcutils_logging_get_logger_effective_level(name);
    rclgo_logging_unlock();
    return level;
}

static bool rclgo_logger_is_enabled_for(const char* name, int severity) {
    rclgo_logging_lock();
    bool enabled = rcutils_logging_logger_is_enabled_for(name, severity);
    rclgo_logging_unlock();
    return enabled;
}

static rcutils_ret_t rclgo_set_logger_level(const char* name, int level) {
    rclgo_logging_lock();
    rcutils_ret_t rc = rcutils_logging_set_logger_level(name, level);
    rclgo_logging_unlock();
    return rc;
}

// Used by rcl.go when nodes are created and closed. rcl_node_init and
// rcl_node_fini also take the mutex, as rclcpp's NodeBase holds its logging
// mutex around them: creating and finalizing nodes on several threads at once
// crashed in rcl_node_fini without it.
rcl_ret_t rclgo_node_init(rcl_node_t* node, const char* name, const char* namespace_,
    rcl_context_t* context, const rcl_node_options_t* options) {
    rclgo_logging_lock();
    rcl_ret_t rc = rcl_node_init(node, name, namespace_, context, options);
    rclgo_logging_unlock();
    return rc;
}

rcl_ret_t rclgo_node_fini(rcl_node_t* node) {
    rclgo_logging_lock();
    rcl_ret_t rc = rcl_node_fini(node);
    rclgo_logging_unlock();
    return rc;
}

rcl_ret_t rclgo_rosout_init_publisher_for_node(rcl_node_t* node) {
    rclgo_logging_lock();
    rcl_ret_t rc = rcl_logging_rosout_init_publisher_for_node(node);
    rclgo_logging_unlock();
    return rc;
}

rcl_ret_t rclgo_rosout_fini_publisher_for_node(rcl_node_t* node) {
    rclgo_logging_lock();
    rcl_ret_t rc = rcl_logging_rosout_fini_publisher_for_node(node);
    rclgo_logging_unlock();
    return rc;
}
*/
import "C"
import (
	"errors"
	"fmt"
	"os"
	"regexp"
	"runtime"
	"strings"
	"sync"
	"sync/atomic"
	"unsafe"
)

// The severity levels of log messages / loggers.
type LogSeverity uint32

// Copypaste from /opt/ros/jazzy/include/rcutils/logging.h

const (
	LogSeverityUnset LogSeverity = 0  ///< The unset log level
	LogSeverityDebug LogSeverity = 10 ///< The debug log level
	LogSeverityInfo  LogSeverity = 20 ///< The info log level
	LogSeverityWarn  LogSeverity = 30 ///< The warn log level
	LogSeverityError LogSeverity = 40 ///< The error log level
	LogSeverityFatal LogSeverity = 50 ///< The fatal log level
)

func (s LogSeverity) String() string {
	switch s {
	case LogSeverityUnset:
		return "UNSET"
	case LogSeverityDebug:
		return "DEBUG"
	case LogSeverityInfo:
		return "INFO"
	case LogSeverityWarn:
		return "WARN"
	case LogSeverityError:
		return "ERROR"
	case LogSeverityFatal:
		return "FATAL"
	default:
		return ""
	}
}

// InitLogging initializes the logging system, which is required for using
// logging functionality.
//
// Logging configuration can be updated by calling InitLogging again with the
// desired args.
//
// If the logging system has not yet been initialized on the first call of Init
// or NewContext, logging is initialized by that call using the passed
// arguments. Unlike InitLogging, Init and NewContext will not update logging
// configuration if logging has already been initialized.
func InitLogging(args *Args) error {
	return rclInitLogging(args, true)
}

var (
	loggingMutex sync.Mutex
	// currentLoggingOutputHandler is read by every log call, from the
	// output handler, which must not take loggingMutex (rcl logs from inside
	// calls made with loggingMutex held).
	currentLoggingOutputHandler = func() *atomic.Pointer[LoggingOutputHandler] {
		var p atomic.Pointer[LoggingOutputHandler]
		h := LoggingOutputHandler(DefaultLoggingOutputHandler)
		p.Store(&h)
		return &p
	}()
	loggingInitialized = false
	// rosoutPublishers counts the nodes whose /rosout publisher is not
	// finalized yet, and finiPending records a FiniLogging deferred until it
	// reaches zero: a node whose finalization is deferred (a running wait set
	// still holds one of its entities) finalizes its publisher in rcl's
	// logging state after Uninit. Both are guarded by loggingMutex.
	rosoutPublishers int
	finiPending      bool
	loggingAllocator = func() *C.rcl_allocator_t {
		alloc := (*C.rcl_allocator_t)(C.malloc(C.sizeof_rcl_allocator_t))
		*alloc = C.rcl_get_default_allocator()
		return alloc
	}()
	// realtimeLogging enables unbuffered, immediate flush logging when true.
	// Set via RCLGO_REALTIME_LOGGING=1 environment variable.
	realtimeLogging atomic.Bool
)

func rclInitLogging(rclArgs *Args, update bool) error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if rclArgs == nil {
		var err error
		if rclArgs, _, err = ParseArgs(nil); err != nil {
			return err
		}
	}
	loggingMutex.Lock()
	defer loggingMutex.Unlock()
	finiPending = false
	if loggingInitialized && !update {
		return nil
	}
	loggingInitialized = true

	// Check environment variable for realtime logging mode
	if os.Getenv("RCLGO_REALTIME_LOGGING") == "1" {
		realtimeLogging.Store(true)

		// Also set RCUTILS_LOGGING_BUFFERED_STREAM=0 if not already set
		// This ensures both rclgo's stdout/stderr flushing AND rcutils console
		// stream unbuffering are active for complete real-time console output.
		if os.Getenv("RCUTILS_LOGGING_BUFFERED_STREAM") == "" {
			os.Setenv("RCUTILS_LOGGING_BUFFERED_STREAM", "0")
		}
	}

	// Use rcl_logging_configure instead of rcl_logging_configure_with_output_handler
	// to ensure proper initialization of the logging backend (including filesystem logging).
	// The default output handler will automatically call rcl_logging_multiple_output_handler,
	// which handles both console and filesystem output.
	//
	// The output handler is rclgo's, which allows for Go-level customization
	// while still delegating to the default rcl handler that writes to files.
	rc := C.rclgo_logging_configure(&rclArgs.parsed, *loggingAllocator)
	runtime.KeepAlive(rclArgs)
	if rc != C.RCL_RET_OK {
		return errorsCastC(rc, "rclInitLogging -> rcl_logging_configure()")
	}

	return nil
}

// FiniLogging finalizes the logging system and flushes all buffered log messages.
// This should be called before program termination to ensure all logs are written.
func FiniLogging() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	loggingMutex.Lock()
	defer loggingMutex.Unlock()
	if !loggingInitialized {
		return nil
	}
	if rosoutPublishers > 0 {
		// Finalized when the last node's /rosout publisher is.
		finiPending = true
		return nil
	}
	return finiLoggingLocked()
}

// finiLoggingLocked finalizes the logging system; loggingMutex must be held
// and the OS thread pinned.
func finiLoggingLocked() error {
	rc := C.rclgo_logging_fini()
	if rc != C.RCL_RET_OK {
		return errorsCastC(rc, "rcl_logging_fini()")
	}
	loggingInitialized = false
	return nil
}

// rosoutPublisherAdded records a node's new /rosout publisher.
func rosoutPublisherAdded() {
	loggingMutex.Lock()
	defer loggingMutex.Unlock()
	rosoutPublishers++
}

// rosoutPublisherRemoved records that a node's /rosout publisher was
// finalized, and runs a FiniLogging that was waiting for it.
func rosoutPublisherRemoved() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	loggingMutex.Lock()
	defer loggingMutex.Unlock()
	rosoutPublishers--
	if rosoutPublishers == 0 && finiPending {
		finiPending = false
		return finiLoggingLocked()
	}
	return nil
}

// IsRealtimeLogging returns true if realtime (unbuffered) logging is enabled.
func IsRealtimeLogging() bool {
	loggingMutex.Lock()
	defer loggingMutex.Unlock()
	return realtimeLogging.Load()
}

var rcutilsLogFormat = C.CString("%s")

// logNamed logs msg with severity level to logger named name. logNamed should
// not be used for logging directly. Use one of the exported logging functions
// instead.
func logNamed(level LogSeverity, name, msg string) error {
	cmsg := C.CString(msg)
	defer C.free(unsafe.Pointer(cmsg))

	loc := C.zero_location
	// Because convenience wrappers are provided for logging, two stack frames
	// must be skipped; one for logNamed and one for the wrapper.
	if pc, file, line, ok := runtime.Caller(2); ok {
		if fn := runtime.FuncForPC(pc); fn != nil {
			loc.function_name = C.CString(fn.Name())
			defer C.free(unsafe.Pointer(loc.function_name))
		}
		loc.file_name = C.CString(file)
		defer C.free(unsafe.Pointer(loc.file_name))
		loc.line_number = C.size_t(line)
	}
	cname := C.CString(name)
	defer C.free(unsafe.Pointer(cname))
	C.rcutils_log_wrapper(&loc, C.int(level), cname, rcutilsLogFormat, cmsg)
	return nil
}

var defaultLogger = &Logger{
	name:  "",
	cname: C.CString(""),
}

// Logger can be used to log messages using the ROS 2 logging system.
//
// Loggers are usable only after logging has been initialized. See InitLogging.
//
// Logging methods prefixed with "Log" take the logging level as the first
// parameter. Methods prefixed with the name of a logging level are shorthands
// to "Log" methods, and log using the prefixed logging level.
//
// Logging methods suffixed with "", "f" or "ln" format their arguments in the
// same way as fmt.Print, fmt.Printf and fmt.Println, respectively.
type Logger struct {
	name  string
	cname *C.char
}

var invalidLoggerNameRegex = regexp.MustCompile(`(^\.)|(\.$)|(\.\.)`)

// GetLogger returns the logger named name. If name is empty, the default logger
// is returned. Returns nil if name is invalid.
func GetLogger(name string) *Logger {
	if name == "" {
		return defaultLogger
	}
	if invalidLoggerNameRegex.MatchString(name) {
		return nil
	}
	l := &Logger{
		name:  name,
		cname: C.CString(name),
	}
	runtime.SetFinalizer(l, func(l *Logger) {
		C.free(unsafe.Pointer(l.cname))
		l.cname = nil
	})
	return l
}

func (l *Logger) Name() string {
	return l.name
}

// Parent returns the parent logger of l. If l has no parent, the default logger
// is returned.
func (l *Logger) Parent() *Logger {
	i := strings.LastIndexByte(l.name, '.')
	if i == -1 {
		return defaultLogger
	}
	return GetLogger(l.name[:i])
}

// Child returns the child logger of l named name. Returns nil if name is
// invalid.
func (l *Logger) Child(name string) *Logger {
	if l.name == "" {
		return GetLogger(name)
	}
	return GetLogger(l.name + "." + name)
}

// Level returns the logging level of l. Note that this is not necessarily the
// same as EffectiveLevel.
func (l *Logger) Level() (LogSeverity, error) {
	runtime.LockOSThread() // rcutils error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	level := C.rclgo_get_logger_level(l.cname)
	runtime.KeepAlive(l)
	if level < 0 {
		return 0, errors.New("failed to get log level: " + errorString())
	}
	return LogSeverity(level), nil
}

// SetLevel sets the logging level of l.
func (l *Logger) SetLevel(level LogSeverity) error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	loggingMutex.Lock()
	defer loggingMutex.Unlock()
	ret := C.rclgo_set_logger_level(l.cname, C.int(level))
	runtime.KeepAlive(l)
	if ret != C.RCL_RET_OK {
		return errorsCastC(ret, "SetLoggerLevel")
	}
	return nil
}

// IsEnabledFor returns true if l can log messages whose severity is at least
// level and false if not.
func (l *Logger) IsEnabledFor(level LogSeverity) bool {
	b := C.rclgo_logger_is_enabled_for(l.cname, C.int(level))
	runtime.KeepAlive(l)
	return bool(b)
}

// EffectiveLevel returns the effective logging level of l, which considers the
// logging levels of l's ancestors as well as the logging level of l itself.
// Note that this is not necessarily the same as Level.
func (l *Logger) EffectiveLevel() (LogSeverity, error) {
	runtime.LockOSThread() // rcutils error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	level := C.rclgo_get_logger_effective_level(l.cname)
	runtime.KeepAlive(l)
	if level < 0 {
		return 0, errors.New("failed to get effective log level: " + errorString())
	}
	return LogSeverity(level), nil
}

func (l *Logger) Log(level LogSeverity, a ...interface{}) error {
	return logNamed(level, l.name, fmt.Sprint(a...))
}

func (l *Logger) Debug(a ...interface{}) error {
	return logNamed(LogSeverityDebug, l.name, fmt.Sprint(a...))
}
func (l *Logger) Info(a ...interface{}) error {
	return logNamed(LogSeverityInfo, l.name, fmt.Sprint(a...))
}
func (l *Logger) Warn(a ...interface{}) error {
	return logNamed(LogSeverityWarn, l.name, fmt.Sprint(a...))
}
func (l *Logger) Error(a ...interface{}) error {
	return logNamed(LogSeverityError, l.name, fmt.Sprint(a...))
}
func (l *Logger) Fatal(a ...interface{}) error {
	return logNamed(LogSeverityFatal, l.name, fmt.Sprint(a...))
}

func sprintln(a ...interface{}) string {
	b := fmt.Sprintln(a...)
	return b[:len(b)-1]
}

func (l *Logger) Logln(level LogSeverity, a ...interface{}) error {
	return logNamed(level, l.name, sprintln(a...))
}

func (l *Logger) Debugln(a ...interface{}) error {
	return logNamed(LogSeverityDebug, l.name, sprintln(a...))
}
func (l *Logger) Infoln(a ...interface{}) error {
	return logNamed(LogSeverityInfo, l.name, sprintln(a...))
}
func (l *Logger) Warnln(a ...interface{}) error {
	return logNamed(LogSeverityWarn, l.name, sprintln(a...))
}
func (l *Logger) Errorln(a ...interface{}) error {
	return logNamed(LogSeverityError, l.name, sprintln(a...))
}
func (l *Logger) Fatalln(a ...interface{}) error {
	return logNamed(LogSeverityFatal, l.name, sprintln(a...))
}

func (l *Logger) Logf(level LogSeverity, format string, a ...interface{}) error {
	return logNamed(level, l.name, fmt.Sprintf(format, a...))
}

func (l *Logger) Debugf(format string, a ...interface{}) error {
	return logNamed(LogSeverityDebug, l.name, fmt.Sprintf(format, a...))
}
func (l *Logger) Infof(format string, a ...interface{}) error {
	return logNamed(LogSeverityInfo, l.name, fmt.Sprintf(format, a...))
}
func (l *Logger) Warnf(format string, a ...interface{}) error {
	return logNamed(LogSeverityWarn, l.name, fmt.Sprintf(format, a...))
}
func (l *Logger) Errorf(format string, a ...interface{}) error {
	return logNamed(LogSeverityError, l.name, fmt.Sprintf(format, a...))
}
func (l *Logger) Fatalf(format string, a ...interface{}) error {
	return logNamed(LogSeverityFatal, l.name, fmt.Sprintf(format, a...))
}
