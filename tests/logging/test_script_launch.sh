#!/bin/bash
# Test script for launching nodes via shell script
# This tests scenario 4: Multiple nodes launched from a shell script that calls a launch file

set -e  # Exit on error

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RCLGO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "========================================="
echo "Logging Test: Script-based Launch"
echo "========================================="
echo ""
echo "RCLGO_ROOT: ${RCLGO_ROOT}"
echo "SCRIPT_DIR: ${SCRIPT_DIR}"
echo ""

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing ROS 2 Humble environment..."
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS 2 Humble not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# Set log directory for this test run
TEST_LOG_DIR="${HOME}/.ros/log/test_script_launch_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${TEST_LOG_DIR}"
export ROS_LOG_DIR="${TEST_LOG_DIR}"

echo "ROS_LOG_DIR set to: ${ROS_LOG_DIR}"
echo ""

# Check for optional RCLGO_REALTIME_LOGGING environment variable
if [ "${RCLGO_REALTIME_LOGGING}" = "1" ]; then
    echo "Realtime logging ENABLED (RCLGO_REALTIME_LOGGING=1)"
else
    echo "Realtime logging disabled (set RCLGO_REALTIME_LOGGING=1 to enable)"
fi
echo ""

# Determine which launch file to use (default to multi-node)
LAUNCH_FILE="${1:-${SCRIPT_DIR}/test_multi_node.launch.py}"
LOG_LEVEL="${2:-INFO}"

echo "Launch file: ${LAUNCH_FILE}"
echo "Log level: ${LOG_LEVEL}"
echo ""

# Launch the nodes
echo "Launching nodes..."
echo "Press Ctrl+C to stop"
echo ""

ros2 launch "${LAUNCH_FILE}" log_level:="${LOG_LEVEL}"

# After launch finishes (Ctrl+C), show log summary
echo ""
echo "========================================="
echo "Launch finished. Log summary:"
echo "========================================="
echo ""
echo "Log directory: ${ROS_LOG_DIR}"
echo ""
echo "Log files created:"
ls -lh "${ROS_LOG_DIR}/" 2>/dev/null || echo "No log files found"
echo ""

# Check for zero-byte logs
ZERO_BYTE_LOGS=$(find "${ROS_LOG_DIR}" -type f -size 0 2>/dev/null | wc -l)
if [ "${ZERO_BYTE_LOGS}" -gt 0 ]; then
    echo "WARNING: Found ${ZERO_BYTE_LOGS} zero-byte log file(s):"
    find "${ROS_LOG_DIR}" -type f -size 0
else
    echo "✓ All log files contain data (no zero-byte logs)"
fi
echo ""

echo "To view logs, run:"
echo "  ls -lh ${ROS_LOG_DIR}/"
echo "  cat ${ROS_LOG_DIR}/<logfile>"
