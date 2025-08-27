#!/usr/bin/env sh

# This script generates the necessary ROS2 messages for rclgo.

echo "This script will generate ROS2 message bindings using rclgo with the following settings:"
echo "  - Root paths: /opt/ros/humble and $(pwd)"
echo "  - Destination path: ./internal/msgs"
echo "  - Message module prefix: github.com/merlindrones/rclgo/internal/msgs"
echo "  - Rclgo import path: github.com/merlindrones/rclgo"
echo "  - Included packages:"
echo "      std_msgs, std_srvs, sensor_msgs, geometry_msgs, example_interfaces,"
echo "      test_msgs, action_msgs, builtin_interfaces, unique_identifier_msgs,"
echo "      rcl_interfaces, service_msgs, lifecycle_msgs"
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
  --root-path "$(pwd)" \
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

