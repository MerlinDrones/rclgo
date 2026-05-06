//go:build cgo

package qos

/*
#cgo CFLAGS:  -I/opt/ros/jazzy/include -I/opt/ros/jazzy/include/rmw -I/opt/ros/jazzy/include/rcutils
#cgo LDFLAGS: -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib
*/
import "C"
