#!/bin/bash
# Test script to compare RCLGO_REALTIME_LOGGING vs RCUTILS_LOGGING_BUFFERED_STREAM
# This tests whether the ROS 2 standard environment variable provides better file flushing

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}RCLgo Logging Environment Variable Test${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
RCLGO_ROOT="${SCRIPT_DIR}/../.."
PARAM_DEMO="${RCLGO_ROOT}/examples/param_demo/param_demo"

# Check if param_demo exists
if [ ! -f "${PARAM_DEMO}" ]; then
    echo -e "${RED}Error: param_demo not found at ${PARAM_DEMO}${NC}"
    echo "Please build it first:"
    echo "  cd ${RCLGO_ROOT}/examples/param_demo"
    echo "  source /opt/ros/\${ROS_DISTRO}/setup.bash"
    echo "  source ${RCLGO_ROOT}/cgo-flags.env"
    echo "  go build"
    exit 1
fi

# Create test log directory
TEST_LOG_DIR="/tmp/rclgo_logging_test_$(date +%s)"
mkdir -p "${TEST_LOG_DIR}"
echo -e "${GREEN}Test logs will be in: ${TEST_LOG_DIR}${NC}"
echo ""

# Helper function to run test and check file size during execution
run_test() {
    local test_name="$1"
    local env_vars="$2"
    local description="$3"

    echo -e "${YELLOW}=== Test: ${test_name} ===${NC}"
    echo -e "${BLUE}Description: ${description}${NC}"
    echo -e "${BLUE}Environment: ${env_vars}${NC}"
    echo ""

    # Create test-specific directory
    local test_dir="${TEST_LOG_DIR}/${test_name}"
    mkdir -p "${test_dir}"

    # Run the node in background
    echo "Starting node..."
    if [ -z "${env_vars}" ]; then
        ROS_LOG_DIR="${test_dir}" "${PARAM_DEMO}" --ros-args --log-level DEBUG &
    else
        env ${env_vars} ROS_LOG_DIR="${test_dir}" "${PARAM_DEMO}" --ros-args --log-level DEBUG &
    fi
    local pid=$!
    echo "Node PID: ${pid}"

    # Wait for node to start and generate some logs
    sleep 2

    # Check log file size DURING execution
    echo ""
    echo "Checking log files DURING execution (after 2 seconds):"
    if find "${test_dir}" -name "*.log" -type f | grep -q .; then
        find "${test_dir}" -name "*.log" -type f -exec sh -c 'echo "  $(basename "$1"): $(stat -c%s "$1") bytes"' _ {} \;

        # Check if any logs are non-zero
        local has_data=0
        while IFS= read -r logfile; do
            size=$(stat -c%s "$logfile")
            if [ "$size" -gt 0 ]; then
                has_data=1
                echo -e "  ${GREEN}✓ Real-time data detected: $(basename "$logfile") has ${size} bytes${NC}"
            fi
        done < <(find "${test_dir}" -name "*.log" -type f)

        if [ $has_data -eq 0 ]; then
            echo -e "  ${YELLOW}⚠ All files are 0 bytes (buffered)${NC}"
        fi
    else
        echo -e "  ${RED}✗ No log files found yet${NC}"
    fi

    # Let it run a bit longer
    sleep 2

    # Stop the node gracefully
    echo ""
    echo "Stopping node (SIGINT for clean shutdown)..."
    kill -INT ${pid}
    wait ${pid} 2>/dev/null || true

    # Check log file size AFTER exit
    echo ""
    echo "Checking log files AFTER clean exit:"
    if find "${test_dir}" -name "*.log" -type f | grep -q .; then
        find "${test_dir}" -name "*.log" -type f -exec sh -c 'echo "  $(basename "$1"): $(stat -c%s "$1") bytes"' _ {} \;

        # Verify all logs have data
        local all_ok=1
        while IFS= read -r logfile; do
            size=$(stat -c%s "$logfile")
            if [ "$size" -eq 0 ]; then
                echo -e "  ${RED}✗ FAIL: $(basename "$logfile") is still 0 bytes${NC}"
                all_ok=0
            else
                echo -e "  ${GREEN}✓ PASS: $(basename "$logfile") has data (${size} bytes)${NC}"
            fi
        done < <(find "${test_dir}" -name "*.log" -type f)

        if [ $all_ok -eq 1 ]; then
            echo -e "${GREEN}✓ All log files have data after exit${NC}"
        fi
    else
        echo -e "${RED}✗ No log files found${NC}"
    fi

    echo ""
    echo "---"
    echo ""
}

# Test 1: Default (buffered) behavior
run_test "test1_default" \
    "" \
    "Default buffered logging (control test)"

# Test 2: RCLGO_REALTIME_LOGGING=1 (custom rclgo feature)
run_test "test2_rclgo_realtime" \
    "RCLGO_REALTIME_LOGGING=1" \
    "rclgo custom realtime logging (flushes stdout/stderr only)"

# Test 3: RCUTILS_LOGGING_BUFFERED_STREAM=0 (ROS 2 standard)
run_test "test3_rcutils_unbuffered" \
    "RCUTILS_LOGGING_BUFFERED_STREAM=0" \
    "ROS 2 standard unbuffered stream (should affect console output)"

# Test 4: RCUTILS_LOGGING_BUFFERED_STREAM=1 (force line buffered)
run_test "test4_rcutils_line_buffered" \
    "RCUTILS_LOGGING_BUFFERED_STREAM=1" \
    "ROS 2 standard line buffered (flush on newline)"

# Test 5: Both together
run_test "test5_both" \
    "RCLGO_REALTIME_LOGGING=1 RCUTILS_LOGGING_BUFFERED_STREAM=0" \
    "Both rclgo and ROS 2 unbuffered options combined"

# Test 6: RCUTILS with stdout
run_test "test6_rcutils_stdout" \
    "RCUTILS_LOGGING_BUFFERED_STREAM=0 RCUTILS_LOGGING_USE_STDOUT=1" \
    "Unbuffered logging to stdout instead of stderr"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Test Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "All tests completed. Logs saved in:"
echo "  ${TEST_LOG_DIR}"
echo ""
echo -e "${YELLOW}Key Findings to Look For:${NC}"
echo "1. Did any test show non-zero log files DURING execution?"
echo "   - If yes, that configuration enables real-time file logging"
echo ""
echo "2. Did all tests show data AFTER exit?"
echo "   - If yes, FiniLogging() is working correctly"
echo ""
echo "3. Compare test2 (RCLGO_REALTIME_LOGGING) vs test3 (RCUTILS_LOGGING_BUFFERED_STREAM=0)"
echo "   - Which one (if any) actually flushes log FILES during execution?"
echo ""
echo -e "${GREEN}To examine logs in detail:${NC}"
echo "  cd ${TEST_LOG_DIR}"
echo "  ls -lh */*.log"
echo ""