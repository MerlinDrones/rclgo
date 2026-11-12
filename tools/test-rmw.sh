#!/usr/bin/env bash
# This file is part of rclgo
#
# Copyright © 2026 Merlin Drones
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

# Runs the test suite once per installed ROS 2 middleware (RMW), as rcl's
# call_for_each_rmw_implementation does. For Zenoh it starts a router that
# listens on localhost only and stops it afterwards.
#
# Usage: tools/test-rmw.sh [go test flags...]   (default: -count=1 ./...)
# Set RMWS to choose, e.g. RMWS="rmw_fastrtps_cpp rmw_cyclonedds_cpp".
: "${ROS_DISTRO:=jazzy}"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash" # before set -u: it reads unset variables
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

args=("$@")
[ ${#args[@]} -eq 0 ] && args=(-count=1 ./...)

if [ -z "${RMWS:-}" ]; then
	RMWS=""
	for rmw in rmw_fastrtps_cpp rmw_cyclonedds_cpp rmw_connextdds rmw_zenoh_cpp; do
		ros2 pkg prefix "$rmw" >/dev/null 2>&1 && RMWS="$RMWS $rmw"
	done
fi

router=""
stop_router() {
	if [ -n "$router" ]; then
		kill -INT "$router" 2>/dev/null
		sleep 1
		kill -KILL "$router" 2>/dev/null
		wait "$router" 2>/dev/null
		router=""
	fi
}
trap stop_router EXIT

if [ -z "${RMWS// /}" ]; then
	echo "no RMW implementation found; source a ROS 2 setup.bash or set RMWS" >&2
	exit 1
fi
# A test that needs the ros2 tools, a ROS package or rclpy fails instead of
# skipping when they are missing.
export RCLGO_REQUIRE_ROS2="${RCLGO_REQUIRE_ROS2:-1}"

status=0
for rmw in $RMWS; do
	echo "=== RMW_IMPLEMENTATION=$rmw"
	if [ "$rmw" = rmw_zenoh_cpp ]; then
		# Another router on the port would make the sessions join it (and its
		# network) while ours fails to start unnoticed.
		if command -v ss >/dev/null && ss -ltn | grep -qE '[:.]7447[[:space:]]'; then
			echo "port 7447 is already in use (another Zenoh router?); skipping $rmw" >&2
			status=1
			continue
		fi
		# The override applies to the router only; sessions connect to it on
		# localhost by default.
		ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' \
			"$(ros2 pkg prefix rmw_zenoh_cpp)/lib/rmw_zenoh_cpp/rmw_zenohd" >/dev/null 2>&1 &
		router=$!
		if command -v ss >/dev/null; then
			for _ in $(seq 1 50); do
				ss -ltn | grep -q '127.0.0.1:7447' && break
				sleep 0.1
			done
		else
			sleep 1 # no ss to check the router's port
		fi
		if ! kill -0 "$router" 2>/dev/null; then
			echo "the Zenoh router did not start; skipping $rmw" >&2
			router=""
			status=1
			continue
		fi
	fi
	RMW_IMPLEMENTATION=$rmw go test "${args[@]}" || status=1
	stop_router
done
exit $status
