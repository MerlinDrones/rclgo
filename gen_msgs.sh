#!/usr/bin/env sh

# This script generates ROS2 message bindings for rclgo DEVELOPMENT.
# This is for working on rclgo itself, using only standard ROS 2 packages.
#
# For ROS 2 workspace development (with custom packages like px4_msgs),
# see gen_cgo_flags.sh instead.

echo "=== rclgo Development: Generate ROS2 Message Bindings ==="
echo ""
echo "This script generates Go bindings for STANDARD ROS 2 packages only."
echo "It uses the following settings:"
echo "  - Root path: /opt/ros/humble (base ROS 2 installation)"
echo "  - Destination: ./pkg/msgs"
echo "  - Included packages: std_msgs, std_srvs, sensor_msgs, geometry_msgs,"
echo "                       example_interfaces, test_msgs, action_msgs,"
echo "                       builtin_interfaces, unique_identifier_msgs,"
echo "                       rcl_interfaces, service_msgs, lifecycle_msgs,"
echo "                       rosgraph_msgs"
echo ""
echo "This will also generate: ./cgo-flags.env (for rclgo development)"
echo ""
read -p "Do you want to proceed (p), view help (h), or quit (q)? [p/h/q]: " choice

case "$choice" in
  h|H)
    echo "Showing help for rclgo-gen generate..."
    go run ./cmd/rclgo-gen generate --help
    exit 0
    ;;
  q|Q)
    echo "Operation cancelled."
    exit 1
    ;;
  p|P|y|Y)
    echo "Proceeding with message generation..."
    ;;
  *)
    echo "Invalid choice. Operation cancelled."
    exit 1
    ;;
esac

go run ./cmd/rclgo-gen generate \
  --root-path /opt/ros/humble \
  --dest-path ./pkg/msgs \
  --message-module-prefix github.com/merlindrones/rclgo/pkg/msgs \
  --rclgo-import-path github.com/merlindrones/rclgo \
  --include-package std_msgs \
  --include-package std_srvs \
  --include-package sensor_msgs \
  --include-package geometry_msgs \
  --include-package example_interfaces \
  --include-package test_msgs \
  --include-package action_msgs \
  --include-package builtin_interfaces \
  --include-package unique_identifier_msgs \
  --include-package rcl_interfaces \
  --include-package service_msgs \
  --include-package lifecycle_msgs \
  --include-package rosgraph_msgs \
  --ignore-ros-distro-mismatch

