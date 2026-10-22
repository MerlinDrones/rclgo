# shellcheck shell=bash disable=SC2034  # sourced by .githooks/lib.sh
# rclgo is a fork: its long-lived branches are the ROS 2 distros it is pegged to
PROTECTED_BRANCHES="jazzy humble master"
# Only the library module; examples have their own go.mod
GO_MODULES="."
# -short skips the examples tests (they run the example modules as packages
# of the main module, which fails since the examples got their own go.mod)
PUSH_TEST_CMD="go test -short ./..."
