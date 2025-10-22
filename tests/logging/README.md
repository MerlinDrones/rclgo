# RCLgo Logging Test Suite

This directory contains comprehensive tests for validating rclgo's ROS 2 logging implementation.

## Overview

The test suite validates logging behavior across 4 different execution scenarios:
1. Single node via `ros2 run`
2. Single node via launch file
3. Multiple nodes via launch file
4. Multiple nodes via shell script calling launch file

For each scenario, we test 5 critical aspects:
- Default behavior (no arguments)
- ROS arguments (--log-level, etc.)
- YAML parameter loading
- Non-zero log files (no empty logs)
- Correct log file locations

## Directory Structure

```
tests/logging/
├── README.md                        # This file
├── config/
│   └── params.yaml                  # Test parameter configuration
├── test_single_node.launch.py       # Scenario 2: Single node launch
├── test_multi_node.launch.py        # Scenario 3: Multi-node launch
└── test_script_launch.sh            # Scenario 4: Script-based launch
```

## Prerequisites

1. **Build rclgo examples:**
   ```bash
   cd /home/dave/Git/merlin/Swarmos/rclgo
   source /opt/ros/humble/setup.bash
   source cgo-flags.env

   # Build param_demo
   cd examples/param_demo
   go build

   # Build publisher/subscriber
   cd ../publisher_subscriber/publisher
   go build
   cd ../subscriber
   go build
   ```

2. **Source ROS 2 environment:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

## Test Scenarios

### Scenario 1: Single Node via `ros2 run`

**Purpose:** Test direct node execution without launch infrastructure.

```bash
cd examples/param_demo

# Test 1a: Default behavior
./param_demo

# Test 1b: With log level
./param_demo --ros-args --log-level DEBUG

# Test 1c: With YAML params
./param_demo --ros-args --params-file ../../tests/logging/config/params.yaml

# Test 1d: With realtime logging
RCLGO_REALTIME_LOGGING=1 ./param_demo
```

**Expected outcomes:**
- ✅ Node runs without errors
- ✅ Console output visible
- ✅ Log files created in `~/.ros/log/<node_name>_<pid>_<timestamp>.log`
- ✅ Log files are non-zero size
- ✅ Log level changes output verbosity
- ✅ YAML parameters loaded correctly

---

### Scenario 2: Single Node via Launch File

**Purpose:** Test logging when node is launched via ROS 2 launch system.

```bash
cd tests/logging

# Test 2a: Default launch
ros2 launch test_single_node.launch.py

# Test 2b: With log level argument
ros2 launch test_single_node.launch.py log_level:=DEBUG

# Test 2c: With custom params file
ros2 launch test_single_node.launch.py params_file:=/path/to/custom_params.yaml
```

**Expected outcomes:**
- ✅ Node launches successfully
- ✅ Console output visible on terminal
- ✅ Logs written to `~/.ros/log/<timestamp>/` directory
- ✅ `launch.log` contains launch system logs
- ✅ Node-specific log files exist and are non-zero size
- ✅ Parameters from YAML file are loaded

**Check logs:**
```bash
# Find the latest log directory
ls -lt ~/.ros/log/ | head -5

# Examine logs
LOG_DIR=$(ls -t ~/.ros/log/ | head -1)
ls -lh ~/.ros/log/${LOG_DIR}/
cat ~/.ros/log/${LOG_DIR}/launch.log
cat ~/.ros/log/${LOG_DIR}/param_demo-*.log
```

---

### Scenario 3: Multiple Nodes via Launch File

**Purpose:** Test logging with multiple concurrent nodes.

```bash
cd tests/logging

# Test 3a: Launch publisher + subscriber
ros2 launch test_multi_node.launch.py

# Test 3b: With debug logging
ros2 launch test_multi_node.launch.py log_level:=DEBUG
```

**Expected outcomes:**
- ✅ Both nodes launch successfully
- ✅ **Console output is interleaved** (messages from both nodes visible)
- ✅ **Separate log files** for each node
- ✅ All log files are non-zero size
- ✅ Logs in `~/.ros/log/<timestamp>/` with:
  - `launch.log`
  - `publisher-*.log`
  - `subscriber-*.log`

**Validation:**
```bash
LOG_DIR=$(ls -t ~/.ros/log/ | head -1)
cd ~/.ros/log/${LOG_DIR}/

# Check for separate log files
echo "Log files:"
ls -lh *.log

# Verify no zero-byte logs
find . -name "*.log" -size 0

# Should output nothing (no zero-byte files)
```

---

### Scenario 4: Script-based Launch

**Purpose:** Test logging when launch is called from a shell script (common in production).

```bash
cd tests/logging

# Test 4a: Run script (default multi-node launch)
./test_script_launch.sh

# Test 4b: With custom log level
./test_script_launch.sh test_multi_node.launch.py DEBUG

# Test 4c: With realtime logging
RCLGO_REALTIME_LOGGING=1 ./test_script_launch.sh

# Test 4d: Single node launch
./test_script_launch.sh test_single_node.launch.py INFO
```

**Expected outcomes:**
- ✅ Script sets `ROS_LOG_DIR` correctly
- ✅ Logs written to script-specified directory
- ✅ Script reports log file locations
- ✅ All log files non-zero
- ✅ Script detects and warns about zero-byte logs

---

## Validation Checklist

Use this checklist to systematically validate all scenarios:

| Scenario | Default | ROS Args | YAML Params | Non-zero Logs | Correct Location |
|----------|---------|----------|-------------|---------------|------------------|
| 1. `ros2 run` single | ☐ | ☐ | ☐ | ☐ | ☐ |
| 2. Launch single | ☐ | ☐ | ☐ | ☐ | ☐ |
| 3. Launch multi | ☐ | ☐ | ☐ | ☐ | ☐ |
| 4. Script launch | ☐ | ☐ | ☐ | ☐ | ☐ |

### How to Validate Each Column

**Default:**
- Run command without any arguments
- Node should start and log messages
- Check `~/.ros/log/` for log files

**ROS Args:**
- Run with `--ros-args --log-level DEBUG`
- Console should show DEBUG messages
- Log file should contain DEBUG output

**YAML Params:**
- Run with `--params-file config/params.yaml`
- Node should print parameter values from YAML
- Example: `camera.fps -> 30` (from YAML, not default 15)

**Non-zero Logs:**
- After running, check log file sizes:
  ```bash
  find ~/.ros/log/ -name "*.log" -size 0
  # Should output nothing
  ```

**Correct Location:**
- Default: `~/.ros/log/`
- Launch files: `~/.ros/log/<timestamp>/`
- Script: Wherever `ROS_LOG_DIR` is set

---

## Environment Variables

### `RCLGO_REALTIME_LOGGING`

When set to `1`, enables realtime console output for development:
- Console logs (stdout/stderr) are flushed immediately after each message
- No console buffering delays
- Useful for debugging and live monitoring
- **File logging** still buffered by RCL backend (spdlog)

**Usage:**
```bash
RCLGO_REALTIME_LOGGING=1 ./param_demo
RCLGO_REALTIME_LOGGING=1 ros2 launch test_single_node.launch.py
```

⚠️ **PRODUCTION WARNING:**
```
DO NOT use RCLGO_REALTIME_LOGGING=1 in production!

- Impacts performance due to frequent flush operations
- Buffered logging (default) is significantly faster
- Production should use default buffered mode
- Logs are guaranteed to flush on clean shutdown (rclgo.Uninit())

Use realtime logging ONLY for:
✓ Local development
✓ Debugging crashes
✓ Live log monitoring with tail -f
✗ Production deployments
✗ Performance-critical applications
```

### `ROS_LOG_DIR`

ROS 2 standard environment variable that specifies log directory:
```bash
export ROS_LOG_DIR=/tmp/my_logs
./param_demo
# Logs will be in /tmp/my_logs/
```

---

## Common Issues and Solutions

### Issue: Zero-byte log files

**Symptoms:**
```bash
$ find ~/.ros/log/ -name "*.log" -size 0
/home/user/.ros/log/2025-10-21-15-30-00/param_demo-12345.log
```

**Cause:** Logging backend not properly flushed before process exit.

**Solution:** Ensure `rclgo.Uninit()` is called (should now call `FiniLogging()` internally).

---

### Issue: Logs in wrong location

**Symptoms:** Logs not appearing in expected `ROS_LOG_DIR`.

**Debugging:**
```bash
# Check ROS environment
env | grep ROS

# Check actual log location
lsof -p <pid> | grep log
```

**Solution:** Ensure `ROS_LOG_DIR` is set before `rclgo.Init()`.

---

### Issue: YAML parameters not loading

**Symptoms:** Node uses default values instead of YAML values.

**Debugging:**
```bash
# Enable debug logging
./param_demo --ros-args --log-level DEBUG --params-file config/params.yaml

# Check for parameter loading messages
```

**Solution:**
1. Verify YAML structure matches node name
2. Call `params.LoadYAML()` before `DeclareIfMissing()`
3. Check param_demo example for correct pattern

---

### Issue: Console output not interleaved in multi-node launch

**Symptoms:** Only one node's output visible on console.

**Cause:** Launch file `output` parameter not set correctly.

**Solution:** Ensure `output='both'` in launch file Node definition.

---

## Expected Log Format

### Console Output

```
[INFO] [<timestamp>] [param_demo]: Publishing: "test message"
[DEBUG] [<timestamp>] [param_demo]: Detailed debug info
[WARN] [<timestamp>] [subscriber]: Warning message
```

### Log File Contents

```
[INFO] [<timestamp>] [param_demo]: Node FQN: /param_demo (name=param_demo)
[INFO] [<timestamp>] [param_demo]: Loaded YAML params from params.yaml
[INFO] [<timestamp>] [param_demo]: camera.fps -> 30
[INFO] [<timestamp>] [param_demo]: tick fps=30 frame_id=test_camera exposure=0.015000
```

---

## Automated Test Script (Future)

TODO: Create `run_all_tests.sh` that:
1. Builds all examples
2. Runs all 4 scenarios with all 5 validation criteria
3. Generates test matrix report
4. Returns 0 if all pass, 1 if any fail

---

## Test Results Template

Use this template to document test results:

```markdown
## Test Run: 2025-10-21

**Environment:**
- ROS 2 Version: Humble
- rclgo Commit: abc1234
- OS: Ubuntu 22.04

### Scenario 1: ros2 run
- [ ] Default: PASS/FAIL - <notes>
- [ ] ROS Args: PASS/FAIL - <notes>
- [ ] YAML: PASS/FAIL - <notes>
- [ ] Non-zero: PASS/FAIL - <notes>
- [ ] Location: PASS/FAIL - <notes>

### Scenario 2: Launch Single
...

### Known Issues
- Zero-byte logs in scenario X
- Location issue in scenario Y

### Next Steps
- Fix issue Z
- Add test coverage for W
```

---

## References

- [ROS 2 Logging Documentation](https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html)
- [Launch File Logging](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html#parameters-arguments-and-substitutions)
- [rclgo CLAUDE.md](../../CLAUDE.md)
