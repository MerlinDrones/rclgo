# Logging Improvements Summary

**Date:** 2025-10-21
**Branch:** feature/logging → humble
**Status:** ✅ MERGED

---

## Overview

Comprehensive improvements to rclgo's ROS 2 logging implementation addressing zero-byte log files, adding realtime console logging support, and creating a full test suite.

---

## Issues Resolved

### 1. Zero-Byte Log Files ✅ FIXED

**Problem:** Log files showed as 0 bytes during node execution and sometimes remained empty after exit.

**Root Cause:** Buffered I/O without proper flush on shutdown.

**Solution:**
- Added `FiniLogging()` function that calls `rcl_logging_fini()`
- Modified `Uninit()` to automatically call `FiniLogging()`
- Ensures all buffered logs are flushed to disk on clean shutdown

**Verification:**
```bash
./param_demo --ros-args --log-level DEBUG
# During execution: 0 bytes (expected - buffered)
# After Ctrl+C: 4.6KB with complete log data ✅
```

### 2. Launch File Compatibility ✅ FIXED

**Problem:** Launch files failed with "package not found" errors for Go executables.

**Root Cause:**
1. Used `launch_ros.actions.Node` which requires ROS 2 packages
2. Relative path resolution issues when running from different directories

**Solution:**
- Changed to `launch.actions.ExecuteProcess` for direct binary execution
- Fixed path resolution: `Path(__file__).resolve()` before computing parent directories

**Verification:**
```bash
ros2 launch test_single_node.launch.py log_level:=DEBUG
# ✅ Executable found
# ✅ YAML params loaded (fps=30 from yaml vs default 15)
# ✅ DEBUG logging active
# ✅ launch.log created (22KB)
```

---

## New Features

### RCLGO_REALTIME_LOGGING Environment Variable

**Purpose:** Enable immediate console output for development and debugging.

**Usage:**
```bash
RCLGO_REALTIME_LOGGING=1 ./my_node
```

**Behavior:**
- Console output (stdout/stderr) flushed immediately after each log message
- Automatically sets `RCUTILS_LOGGING_BUFFERED_STREAM=0` if not already set (for complete console unbuffering)
- Useful for live monitoring with `tail -f` and debugging crashes
- **File logging** still buffered by RCL backend (spdlog) - by design for performance

**What gets flushed:**
- ✅ rclgo's stdout/stderr (via `fflush()`)
- ✅ rcutils console streams (via `RCUTILS_LOGGING_BUFFERED_STREAM=0`)
- ❌ ROS log files in `~/.ros/log/` (still buffered by spdlog)

**For real-time file capture:**
Use shell redirection to capture the unbuffered console output:
```bash
RCLGO_REALTIME_LOGGING=1 ./my_node 2>&1 | tee my_realtime_logs.txt
```

**⚠️ Production Warning:**
```
DO NOT use RCLGO_REALTIME_LOGGING=1 in production!

✓ Use for: Local development, debugging, live monitoring
✗ Avoid for: Production deployments, performance-critical applications

Reason: Frequent flush operations impact performance
```

---

## Code Changes

### pkg/rclgo/logging.go
```go
// Added environment variable support
if os.Getenv("RCLGO_REALTIME_LOGGING") == "1" {
    realtimeLogging = true
}

// Added cleanup function
func FiniLogging() error {
    rc := C.rcl_logging_fini()
    if rc != C.RCL_RET_OK {
        return errorsCastC(rc, "rcl_logging_fini()")
    }
    loggingInitialized = false
    return nil
}
```

### pkg/rclgo/logginghandler.go
```go
// Added realtime flush
if realtimeLogging {
    C.fflush(C.stdout)
    C.fflush(C.stderr)
}
```

### pkg/rclgo/context.go
```go
// Added automatic logging cleanup
func Uninit() (err error) {
    if defaultContext != nil {
        err = defaultContext.Close()
        defaultContext = nil
    }
    // Finalize logging system to flush all buffered logs
    if loggingErr := FiniLogging(); loggingErr != nil {
        err = errors.Join(err, loggingErr)
    }
    return
}
```

---

## Test Suite

Created comprehensive test infrastructure in `tests/logging/`:

### Files Created
```
tests/logging/
├── README.md                  # Complete testing guide (9.6KB)
├── TEST_RESULTS.md            # Detailed results and analysis (12KB)
├── config/
│   └── params.yaml            # Test parameters
├── test_single_node.launch.py # Single node launch
├── test_multi_node.launch.py  # Multi-node launch
└── test_script_launch.sh      # Automated test script
```

### Test Coverage

| Scenario | Status | Verified |
|----------|--------|----------|
| Direct execution (ros2 run) | ✅ PASS | 4.6KB log with data |
| Launch file execution | ✅ PASS | YAML params + DEBUG logs |
| Realtime console logging | ✅ PASS | Immediate stdout/stderr flush |
| YAML parameter loading | ✅ PASS | fps=30 from YAML (not default) |
| Log file creation | ✅ PASS | Non-zero after exit |
| Script-based launch | ✅ READY | Automated testing script |

---

## Behavioral Notes

### Buffered Logging (Default)

**Expected Behavior:**
1. Log files appear as **0 bytes during execution** (buffered in memory)
2. Logs written to disk **on clean exit** (via FiniLogging())
3. **No data loss** - all logs are flushed on shutdown

**This is CORRECT behavior** for production:
- Better performance (fewer disk writes)
- Standard practice for all buffered I/O systems
- Guaranteed flush on clean shutdown (rclgo.Uninit())

### Realtime Logging (Development Only)

**RCLGO_REALTIME_LOGGING=1:**
- Console: Flushed immediately ✅
- Files: Still buffered by spdlog ℹ️

**Why file buffering persists:**
- `fflush(stdout/stderr)` only affects console streams
- spdlog (RCL backend) manages file buffering internally
- This is **by design** - file buffering provides performance benefits

---

## Production Recommendations

### ✅ DO:
- Use default buffered logging (no environment variable)
- Ensure nodes call `rclgo.Uninit()` in defer or cleanup
- Check logs after node shutdown for post-mortem analysis
- Use `--log-level` for appropriate verbosity

### ⚠️ DON'T:
- Don't use `RCLGO_REALTIME_LOGGING=1` in production
- Don't check log file size while node is running (buffered!)
- Don't skip `rclgo.Uninit()` - logs won't be flushed

### 💡 Development Tips:
- Use `RCLGO_REALTIME_LOGGING=1` for debugging crashes
- Use `tail -f` with realtime mode for live monitoring
- Check logs after Ctrl+C to verify buffered data was written

---

## Environment Variable Comparison (2025-10-30)

### Test: RCLGO_REALTIME_LOGGING vs RCUTILS_LOGGING_BUFFERED_STREAM

**Question:** Does the ROS 2 standard `RCUTILS_LOGGING_BUFFERED_STREAM=0` enable real-time file logging?

**Test Script:** `tests/logging/test_env_vars.sh`

**Results:** ❌ No - Neither environment variable enables real-time file logging.

| Environment Variable | Affects Console | Affects Files | ROS 2 Standard |
|---------------------|-----------------|---------------|----------------|
| `RCLGO_REALTIME_LOGGING=1` | ✅ Yes | ❌ No | ❌ No (rclgo custom) |
| `RCUTILS_LOGGING_BUFFERED_STREAM=0` | ✅ Yes | ❌ No | ✅ Yes |
| `RCUTILS_CONSOLE_OUTPUT_FORMAT` | ℹ️ Format only | ℹ️ Format only | ✅ Yes |

**Test Details (6 scenarios tested):**

All 6 tests showed:
- **During execution:** Log files = 0 bytes (buffered)
- **After exit:** Log files = 4676 bytes (flushed correctly)

This confirms:
1. `RCLGO_REALTIME_LOGGING=1` flushes Go's stdout/stderr buffers
2. `RCUTILS_LOGGING_BUFFERED_STREAM=0` controls rcutils console stream buffering
3. **File logging uses spdlog's internal buffering**, which is independent of C stdio buffering
4. Both environment variables work for console output, neither affects file output
5. `FiniLogging()` correctly flushes all buffered file data on shutdown

**Improvement (2025-10-30):**
`RCLGO_REALTIME_LOGGING=1` now automatically sets `RCUTILS_LOGGING_BUFFERED_STREAM=0` if not already set, providing complete console unbuffering with a single environment variable.

**Why file logging remains buffered:**

```
Console Path:                    File Path:
  Go log → C stdout/stderr        Go log → rcutils → spdlog file sink
  ↓                              ↓
  fflush() works ✅              fflush() doesn't reach here ❌
  RCUTILS_* works ✅             RCUTILS_* doesn't reach here ❌
```

spdlog file sinks use `std::ofstream` with their own internal buffering, separate from C stdio.

**Recommendation:**
- Use `RCLGO_REALTIME_LOGGING=1` for complete console unbuffering (now sets both flags automatically)
- For real-time file capture: `RCLGO_REALTIME_LOGGING=1 ./node 2>&1 | tee logfile.txt`
- Neither environment variable enables real-time ROS log files (`~/.ros/log/`) - this is by design for performance

---

## Future Work

Potential enhancements for future versions:

1. **Periodic flush:** Add optional periodic flush (e.g., every 5 seconds) in addition to exit flush
2. **Signal handling:** Ensure `FiniLogging()` called on SIGTERM/SIGINT
3. **Crash recovery:** Use `defer` in main() to catch panics and flush logs
4. **Log rotation:** Implement rotation for long-running nodes
5. ~~**File realtime mode:** Investigate deeper spdlog integration for true file-level realtime logging~~
   - **Update 2025-10-30:** Tested - spdlog file buffering is independent of stdio. Would require custom file sink implementation.

---

## Migration Guide

### For Existing Code

No changes required! All improvements are **backward compatible**:

```go
// Existing code continues to work
rclArgs, _, _ := rclgo.ParseArgs(os.Args[1:])
rclgo.Init(rclArgs)
defer rclgo.Uninit()  // Now automatically calls FiniLogging()

node, _ := rclgo.NewNode("my_node", "")
defer node.Close()

node.Logger().Info("Hello, World!")  // Logged as before
```

### For New Development

Optional: Use realtime logging during development:

```bash
# Development
RCLGO_REALTIME_LOGGING=1 ./my_node

# Production (default buffered logging)
./my_node
```

---

## Commits

1. **e269480b** - `feat(logging): add realtime logging support and fix log flush on exit`
   - Core implementation: FiniLogging(), RCLGO_REALTIME_LOGGING, test suite

2. **af1cb0f1** - `fix(tests): resolve absolute paths in launch file for ExecuteProcess`
   - Fixed path resolution bug in launch files

3. **e45b080d** - `docs(tests): finalize test results and add production warnings`
   - Updated documentation with test results and production warnings

4. **bfd8120a** - `feat(logging): merge logging improvements from feature/logging`
   - Merge commit to humble branch

---

## References

- **Test Documentation:** `tests/logging/README.md`
- **Test Results:** `tests/logging/TEST_RESULTS.md`
- **Environment Variable Tests:** `tests/logging/test_env_vars.sh`
- **ROS 2 Logging Docs:** https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html
- **ROS 2 Environment Variables:** https://docs.ros.org/en/kilted/Concepts/Intermediate/About-Logging.html#environment-variables
- **rclgo CLAUDE.md:** Project documentation and architecture

---

**Merge Status:** ✅ Complete - feature/logging merged to humble
**All Tests:** ✅ Passing
**Production Ready:** ✅ Yes
**Environment Variable Analysis:** ✅ Complete (2025-10-30)
