rclgo the ROS2 client library Golang wrapper
============================================

[![Go Reference](https://pkg.go.dev/badge/github.com/merlindrones/rclgo.svg)][docs]

## Roadmap
[Roadmap for ROS2 Parity](ROADMAP.md)

## Getting started

rclgo is used with the Go module system like most Go libraries. It also requires
a ROS 2 installation as well as C bindings to any ROS interface type used with
the library. The ROS core components can be installed by installing the Debian
package `ros-jazzy-ros-core`. C bindings for ROS interfaces are also usually
distributed as Debian packages.

API documentation is available at [pkg.go.dev][docs].

An example module with a publisher and a subscriber can be found in
[examples/publisher_subscriber](examples/publisher_subscriber).

### ROS 2 interface bindings

rclgo requires Go bindings of all the ROS 2 interfaces to exist. rclgo-gen is
used to generate Go bindings to existing interface types installed. The
generated Go bindings depend on the corresponding C bindings to be installed.

rclgo-gen can be installed globally by running

    go install github.com/merlindrones/rclgo/cmd/rclgo-gen@latest

but it is recommended to add rclgo-gen as a dependency to the project to ensure
the version matches that of rclgo. This can be done by adding a file `tools.go`
to the main package of the project containing something similar to the
following:
```go
//go:build tools

package main

import _ "github.com/merlindrones/rclgo/cmd/rclgo-gen"
```
Then run `go mod tidy`. This version of rclgo-gen can be used by running

    go run github.com/merlindrones/rclgo/cmd/rclgo-gen generate -d msgs --include-go-package-deps ./...

in the project directory. The command can be added as a `go generate` comment to
one of the source files in the project, such as `main.go`, as follows:
```go
//go:generate go run github.com/merlindrones/rclgo/cmd/rclgo-gen generate -d msgs --include-go-package-deps ./...
```

### Developing with custom interface types

By default `rclgo-gen generate` looks for interface definitions in
`$AMENT_PREFIX_PATH` and generates Go bindings for all interfaces it finds.
`$AMENT_PREFIX_PATH` contains a list of paths to the ROS 2 underlay and overlays
you have sourced. If you want to use interfaces which are not available in
`$AMENT_PREFIX_PATH`, you can either add the directories to `$AMENT_PREFIX_PATH`
(use the package as an overlay) or pass paths using the `--root-path` option. If
multiple `--root-path` options are passed, the paths are searched in the order
the options are passed. If multiple identically named package and interface name
combinations are found the first one is used. When `--root-path` is used,
`rclgo-gen generate` won't look for interface definitions from
`$AMENT_PREFIX_PATH`. If you want to generate bindings for files using both
`$AMENT_PREFIX_PATH` and custom paths, you can pass
`--root-path="$AMENT_PREFIX_PATH"` in addition to the other paths.

In addition to generating the Go bindings, you must also generate the C
bindings, compile them and make them available to the Go tool. Generated Go
bindings include the `include` and `lib` subdirectories of the root paths used
when generating the bindings in the header and library search paths,
respectively. When building a package using the Go bindings the environment
variables `CGO_CFLAGS` and `CGO_LDFLAGS` can be used to pass additional `-I` and
`-L` options, respectively, if needed.

An example is available in
[examples/custom_message_package](examples/custom_message_package).

### CGO Flags for ROS 2 Development

Building Go programs that use ROS 2 requires setting `CGO_CFLAGS` and
`CGO_LDFLAGS` to include ROS 2 headers and libraries. rclgo provides tools to
generate these flags automatically.

#### For rclgo Development

When working on rclgo itself (using only standard ROS 2 packages), use the
provided `gen_msgs.sh` script:

```bash
./gen_msgs.sh
```

This generates Go message bindings and a `cgo-flags.env` file in the rclgo
directory. Source this file before building or testing:

```bash
source ./cgo-flags.env
go test ./pkg/rclgo/...
```

#### For ROS 2 Workspace Development

When building ROS 2 nodes with custom packages (e.g., `px4_msgs`,
`swarmos_msgs`), you need a `cgo-flags.env` that includes both the base ROS 2
installation and your workspace overlay.

**Prerequisites:**
1. Set the `ROS_WS` environment variable to your workspace directory:
   ```bash
   export ROS_WS=${HOME}/path/to/your/workspace
   ```
   Add this to your `~/.bashrc` or `~/.zshrc` to make it permanent.

2. Build your workspace to populate the `install/` directory:
   ```bash
   cd ${ROS_WS}
   source /opt/ros/humble/setup.bash
   colcon build
   ```

**Generate cgo-flags.env:**

Use the `gen_cgo_flags.sh` script from the rclgo directory:

```bash
# Generate with default packages (px4_msgs, swarmos_msgs)
./gen_cgo_flags.sh

# Or specify custom packages
./gen_cgo_flags.sh px4_msgs my_custom_msgs another_pkg
```

This creates `${ROS_WS}/cgo-flags.env`. Source it before building Go nodes:

```bash
source ${ROS_WS}/cgo-flags.env
cd ${ROS_WS}/src/my_go_node
go build .
```

**Alternative: Manual Command**

You can also use `rclgo-gen generate-cgo-flags` directly:

```bash
go run github.com/merlindrones/rclgo/cmd/rclgo-gen generate-cgo-flags \
  --root-path /opt/ros/humble \
  --root-path ${ROS_WS}/install \
  --output ${ROS_WS}/cgo-flags.env \
  --include-package px4_msgs \
  --include-package swarmos_msgs
```

The `--include-package` flags are optional and help scan for package-specific
C dependencies. The generated file will include paths for all standard ROS 2
packages plus any custom packages found in the workspace.

[docs]: https://pkg.go.dev/github.com/merlindrones/rclgo/pkg/rclgo
