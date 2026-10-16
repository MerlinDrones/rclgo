package rclgo

import (
	"fmt"
	"testing"
)

// Context.Close and Node.Close must finalize every resource before the
// resources it uses: rcl_timer_fini and rcl_action_server_fini read the clock,
// and entities are finalized with their node.
func TestCloseOrder(t *testing.T) {
	newResources := func() []rosResource {
		// Interleave kinds and IDs so that neither insertion order nor ID
		// order alone gives the expected result.
		rs := []rosResource{
			&Clock{}, &guardCondition{}, &WaitSet{}, &Timer{}, &Node{},
			&Publisher{}, &Subscription{}, &Service{}, &Client{},
			&ActionClient{}, &ActionServer{}, &Clock{}, &Timer{}, &Node{},
		}
		for i, r := range rs {
			r.setID(uint64(len(rs) - i))
		}
		return rs
	}
	kinds := func(rs []rosResource) (out []string) {
		for _, r := range rs {
			out = append(out, fmt.Sprintf("%T", r))
		}
		return out
	}
	got := kinds(closeOrder(newResources()))
	want := []string{
		"*rclgo.ActionServer", "*rclgo.ActionClient",
		"*rclgo.Client", "*rclgo.Service", "*rclgo.Subscription", "*rclgo.Publisher",
		"*rclgo.Node", "*rclgo.Node",
		"*rclgo.Timer", "*rclgo.Timer",
		"*rclgo.WaitSet", "*rclgo.guardCondition",
		"*rclgo.Clock", "*rclgo.Clock",
	}
	if fmt.Sprint(got) != fmt.Sprint(want) {
		t.Errorf("close order\n got %v\nwant %v", got, want)
	}
	// Within a kind, resources are closed in creation (ID) order.
	ordered := closeOrder(newResources())
	for i := 1; i < len(ordered); i++ {
		if fmt.Sprintf("%T", ordered[i]) == fmt.Sprintf("%T", ordered[i-1]) && ordered[i].getID() < ordered[i-1].getID() {
			t.Errorf("%T with ID %d closed before ID %d", ordered[i], ordered[i-1].getID(), ordered[i].getID())
		}
	}
}
