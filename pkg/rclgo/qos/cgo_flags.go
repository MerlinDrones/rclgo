//go:build cgo

package qos

/*
#cgo CFLAGS:  -I/opt/ros/jazzy/include -I/opt/ros/jazzy/include/rmw -I/opt/ros/jazzy/include/rcutils
#cgo CFLAGS:  -I/opt/ros/jazzy/include/rosidl_runtime_c -I/opt/ros/jazzy/include/rosidl_typesupport_interface -I/opt/ros/jazzy/include/rosidl_dynamic_typesupport
#cgo LDFLAGS: -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib
*/
import "C"
