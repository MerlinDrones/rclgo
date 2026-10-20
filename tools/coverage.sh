#!/usr/bin/env bash
# Measures the statement coverage of rclgo's own code by the whole test suite
# and fails if it is below a minimum. Generated message packages (pkg/msgs)
# and internal test helpers are not counted.
#
# Usage: tools/coverage.sh [go test flags]
#   COVERAGE_MIN  minimum total coverage in percent (default 85)
#   COVERAGE_OUT  coverage profile to write (default coverage.out)
set -eo pipefail
if [[ -n "$ROS_DISTRO" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
	source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
set -u
min=${COVERAGE_MIN:-85}
out=${COVERAGE_OUT:-coverage.out}
pkgs=$(go list ./pkg/... ./cmd/... | grep -v -e /pkg/msgs/ -e /internal/ | paste -sd, -)
export RCLGO_REQUIRE_ROS2="${RCLGO_REQUIRE_ROS2:-1}"
# The command tests run a built rclgo-gen binary; with GOCOVERDIR set they
# build it with -cover, and its coverage is merged into the profile.
GOCOVERDIR=$(mktemp -d)
export GOCOVERDIR RCLGO_COVERPKG="$pkgs"
trap 'rm -rf "$GOCOVERDIR"' EXIT
go test -count=1 -coverpkg="$pkgs" -coverprofile="$out" "$@" ./...
if [ -n "$(ls -A "$GOCOVERDIR")" ]; then
	go tool covdata textfmt -i="$GOCOVERDIR" -o "$out.bin"
	tail -n +2 "$out.bin" >>"$out"
	rm -f "$out.bin"
fi
total=$(go tool cover -func="$out" | awk '/^total:/ { sub("%", "", $3); print $3 }')
echo "coverage of rclgo's own code: $total% (minimum $min%)"
if ! awk -v t="$total" -v m="$min" 'BEGIN { exit !(t + 0 >= m + 0) }'; then
	echo "coverage is below the minimum" >&2
	exit 1
fi
