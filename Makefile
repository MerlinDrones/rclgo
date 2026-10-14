GO111MODULE = on

.ONESHELL:
SHELL = /bin/bash

ROS_DISTRO ?= jazzy

all: configure build #default make target

configure:
	echo "" # make configure unimplemented. Unified deployment API between all software components.

build:
	cd cmd/rclgo-gen && go build

install:
	cd cmd/rclgo-gen && go install

.PHONY: test
test:
	export LD_LIBRARY_PATH=/opt/ros/$(ROS_DISTRO)/lib/x86_64-linux-gnu:/opt/ros/$(ROS_DISTRO)/lib:$$LD_LIBRARY_PATH
	RCLGO_REQUIRE_ROS2=1 go test -count=1 -shuffle=on -timeout 30m ./...

# Tests run in a random order (-shuffle=on prints the seed; rerun a failure
# with -shuffle=<seed>) so that they cannot depend on each other's state. A
# test fails instead of skipping when the ROS 2 tools, a ROS package or rclpy
# it needs is missing (RCLGO_REQUIRE_ROS2).
#
# The race detector is the main defence against the concurrency defects this
# library has had; CI runs this target.
.PHONY: test-race
test-race:
	LD_LIBRARY_PATH=/opt/ros/$(ROS_DISTRO)/lib/x86_64-linux-gnu:/opt/ros/$(ROS_DISTRO)/lib:$$LD_LIBRARY_PATH \
		RCLGO_REQUIRE_ROS2=1 go test -race -count=1 -shuffle=on -timeout 45m ./...

# Runs the suite once per installed RMW (Fast DDS, Cyclone DDS, Zenoh, ...).
.PHONY: test-rmw
test-rmw:
	tools/test-rmw.sh -count=1 ./...

# Makes every rcl call that test scenarios reach fail in turn
# (pkg/rclgo/internal/faultinject); RCLGO_FAULT_CALLS sets how many calls of
# each function (default 2) and RCLGO_FAULT_RETS the codes they fail with
# (error, timeout, bad_alloc; default error).
.PHONY: test-faults
test-faults:
	RCLGO_FAULT_SWEEP=1 go test -count=1 -timeout 90m -run TestFaultSweep -v ./pkg/rclgo/internal/faultinject/

# Fails if the suite covers less than COVERAGE_MIN percent of rclgo's own
# code; writes coverage.out.
.PHONY: cover
cover:
	tools/coverage.sh

.PHONY: test-verbose
test-verbose:
	export LD_LIBRARY_PATH=/opt/ros/$(ROS_DISTRO)/lib/x86_64-linux-gnu:/opt/ros/$(ROS_DISTRO)/lib:$$LD_LIBRARY_PATH
	go test -v -count=1 ./...

generate:
	dest_path=internal/msgs

	rm -rf "$$dest_path/"*
	go run ./cmd/rclgo-gen generate \
	    --root-path /usr \
	    --root-path /opt/ros/${ROS_DISTRO} \
	    --dest-path "$(DEST_PATH)" \
		--message-module-prefix "github.com/merlindrones/rclgo/$(DEST_PATH)" \
		--license-header-path ./license-header.txt \
		--include-go-package-deps ./... \
		--cgo-flags-path "" \
		|| exit 1
	rm "$$dest_path/msgs.gen.go" || exit 1
	go run ./cmd/rclgo-gen generate-rclgo \
		--root-path /usr \
		--root-path /opt/ros/${ROS_DISTRO} \
		--license-header-path ./license-header.txt

lint:
	golangci-lint run ./...

