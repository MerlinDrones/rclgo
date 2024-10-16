// This test is not in the package typemap to avoid an import
// cycle.

package test

import (
	"testing"

	std_srvs_srv "github.com/merlindrones/rclgo/internal/msgs/std_srvs/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	. "github.com/smartystreets/goconvey/convey" //nolint:revive
)

func TestGetService(t *testing.T) {
	Convey("Scenario: test service name to type translation", t, func() {
		Convey("Translating the name of an imported message should work", func() {
			srv, ok := typemap.GetService("std_srvs/Empty")
			So(ok, ShouldBeTrue)
			So(srv, ShouldNotBeNil)
			So(srv, ShouldHaveSameTypeAs, std_srvs_srv.EmptyTypeSupport)
		})
		Convey("Translating the name of a non-imported message should not work", func() {
			srv, ok := typemap.GetService("std_srvs/ThisServiceDoesNotExist")
			So(ok, ShouldBeFalse)
			So(srv, ShouldBeNil)
		})
	})
}
