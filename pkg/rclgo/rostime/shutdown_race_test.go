package rostime_test

import (
	"context"
	"testing"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
	"github.com/merlindrones/rclgo/pkg/rclgo/rostime"
	"github.com/stretchr/testify/require"
)

// A use_sim_time change whose post-set callbacks are already running when
// the clock is shut down doesn't switch the clock to simulated time again
// (and leave a /clock subscription behind) after Shutdown returned.
func TestShutdownRacingUseSimTimeChange(t *testing.T) {
	node := rclgotest.Node(t, "shutdown_race", "/sim_test")
	m, err := params.NewManager(node)
	require.NoError(t, err)
	// Registered before the clock's own callback, so it runs first and
	// holds the batch of callbacks until Shutdown has returned.
	entered, release := make(chan struct{}), make(chan struct{})
	m.AddPostSetCallback(func(ps []params.Parameter) {
		if len(ps) == 1 && ps[0].Name == "use_sim_time" {
			close(entered)
			<-release
		}
	})
	clock, err := rostime.NewClock(context.Background(), node, m)
	require.NoError(t, err)
	rclgotest.Spin(t, node.Context().Spin)

	set := make(chan error, 1)
	go func() {
		_, err := m.SetValue("use_sim_time", params.Value{Kind: params.KindBool, Bool: true})
		set <- err
	}()
	<-entered
	clock.Shutdown()
	close(release)
	require.NoError(t, <-set)

	require.False(t, clock.IsSimTime(), "the clock switched to simulated time after Shutdown")
	infos, err := node.GetSubscriptionsInfoByTopic("/clock", true)
	require.NoError(t, err)
	for _, info := range infos {
		require.NotEqual(t, node.Name(), info.NodeName, "a /clock subscription was created after Shutdown")
	}
	require.WithinDuration(t, time.Now(), clock.Now(), time.Second, "the clock follows system time")
}
