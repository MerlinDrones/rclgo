//go:build cgo

package qos

/*
#cgo CFLAGS:  -I/opt/ros/humble/include -I/opt/ros/humble/include/rmw -I/opt/ros/humble/include/rcutils
#cgo LDFLAGS: -L/opt/ros/humble/lib -Wl,-rpath,/opt/ros/humble/lib
*/
import "C"
