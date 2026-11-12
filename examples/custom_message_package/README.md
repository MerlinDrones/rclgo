# Custom message package example

This is an example of how to use a ROS 2 package containing custom ROS 2 message
definitions in a ROS 2 node implemented using rclgo. `greeting_msgs` contains a
ROS 2 package with a custom message definition that is used by `greeter`, a ROS 2
node implemented using rclgo.

First `greeting_msgs` must be built. Ensure you have sourced your ROS 2
environment and have C build tools, colcon and rosidl C generator installed
(Ubuntu packages build-essential, python3-colcon-common-extensions and
ros-\$ROS_DISTRO-rosidl-generator-c). Switch to `greeting_msgs` directory and build
C bindings for the package by running

    colcon build

Source the package as an overlay by running

    . install/local_setup.sh

Then switch to `greeter` directory. Generate Go bindings by running

    go generate

The generated bindings embed only the paths of the ROS 2 installation
(`/opt/ros/<distro>`), not those of the overlay, so that they build on any
machine. `go generate` also writes the include and library paths of every
package it used, the overlay's included, to `cgo-flags.env`. Load them with

    . ./cgo-flags.env

Now the program can be compiled and tested by running

    go build
    ./greeter

which publishes a welcoming greeting to someone in ROS topic `/greeter/hello`.
