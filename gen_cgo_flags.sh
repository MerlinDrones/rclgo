#!/usr/bin/env sh

# This script generates cgo-flags.env for ROS 2 WORKSPACE development.
# Use this when building ROS 2 nodes that need both base ROS 2 and workspace overlay packages.
#
# For rclgo development (standard ROS 2 packages only), use gen_msgs.sh instead.

set -e

# Check if ROS_WS environment variable is set
if [ -z "$ROS_WS" ]; then
    echo "ERROR: ROS_WS environment variable is not set!"
    echo ""
    echo "Please set ROS_WS to your workspace directory. For example:"
    echo "  export ROS_WS=\${HOME}/Git/merlin/Swarmos/swarmos-nodes"
    echo ""
    echo "Add this to your ~/.bashrc or ~/.zshrc to make it permanent."
    exit 1
fi

# Check if workspace install directory exists
if [ ! -d "${ROS_WS}/install" ]; then
    echo "WARNING: Workspace install directory not found: ${ROS_WS}/install"
    echo ""
    echo "Have you built your workspace? Run:"
    echo "  cd ${ROS_WS}"
    echo "  colcon build"
    echo ""
    read -p "Continue anyway? [y/N]: " continue_choice
    case "$continue_choice" in
        y|Y)
            echo "Continuing..."
            ;;
        *)
            echo "Operation cancelled."
            exit 1
            ;;
    esac
fi

# Default packages to include (can be overridden by command-line arguments)
DEFAULT_PACKAGES="px4_msgs swarmos_msgs"
PACKAGES="${*:-$DEFAULT_PACKAGES}"

echo "=== ROS 2 Workspace: Generate cgo-flags.env ==="
echo ""
echo "This script generates cgo-flags.env for building Go programs with ROS 2."
echo ""
echo "Configuration:"
echo "  - Workspace: ${ROS_WS}"
echo "  - Output file: ${ROS_WS}/cgo-flags.env"
echo "  - Root paths:"
echo "      1. /opt/ros/humble (base ROS 2)"
echo "      2. ${ROS_WS}/install (workspace overlay)"
if [ -n "$PACKAGES" ]; then
    echo "  - Custom packages to scan: ${PACKAGES}"
else
    echo "  - Custom packages to scan: (none - will include all standard ROS 2 packages)"
fi
echo ""
read -p "Do you want to proceed (p), view help (h), or quit (q)? [p/h/q]: " choice

case "$choice" in
  h|H)
    echo "Showing help for rclgo-gen generate-cgo-flags..."
    go run ./cmd/rclgo-gen generate-cgo-flags --help
    exit 0
    ;;
  q|Q)
    echo "Operation cancelled."
    exit 1
    ;;
  p|P|y|Y)
    echo "Proceeding with cgo-flags generation..."
    ;;
  *)
    echo "Invalid choice. Operation cancelled."
    exit 1
    ;;
esac

# Build the command with --include-package flags for each package
CMD="go run ./cmd/rclgo-gen generate-cgo-flags \
  --root-path /opt/ros/humble \
  --root-path ${ROS_WS}/install \
  --output ${ROS_WS}/cgo-flags.env"

for pkg in $PACKAGES; do
    CMD="$CMD --include-package $pkg"
done

CMD="$CMD --ignore-ros-distro-mismatch"

echo ""
echo "Running: $CMD"
echo ""

eval $CMD

if [ $? -eq 0 ]; then
    echo ""
    echo "SUCCESS! Generated: ${ROS_WS}/cgo-flags.env"
    echo ""
    echo "To use this file, source it before building Go programs:"
    echo "  source ${ROS_WS}/cgo-flags.env"
    echo ""
    echo "Or add to your shell initialization file (~/.bashrc or ~/.zshrc):"
    echo "  export ROS_WS=\${HOME}/Git/merlin/Swarmos/swarmos-nodes"
    echo "  source \${ROS_WS}/cgo-flags.env"
else
    echo ""
    echo "ERROR: Failed to generate cgo-flags.env"
    exit 1
fi
