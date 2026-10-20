package rclgo_test

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/stretchr/testify/require"
)

// Corners of the service protocol, after rclcpp's test_service and
// test_client: several clients, responses out of order, late and duplicate
// responses. Every Send must get the response to its own request.

func isProbe(req *example_interfaces_srv.AddTwoInts_Request) bool { return req.A == 0 && req.B == 0 }

// Two clients of one service each get the answers to their own requests.
func TestServiceTwoClients(t *testing.T) {
	node, _, cli := newAddService(t, "two_clients", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		respondSum(req, send) //nolint:errcheck
	})
	keepAll := qos.NewDefault()
	keepAll.History = qos.HistoryKeepAll
	cli2, err := node.NewClient("two_clients", example_interfaces_srv.AddTwoIntsTypeSupport, &rclgo.ClientOptions{Qos: keepAll})
	require.NoError(t, err)
	requireEventually(t, "second client reaches the service", func() error {
		_, err := callAdd(cli2, 0, 0, 300*time.Millisecond)
		return err
	})
	var wg sync.WaitGroup
	for c, client := range []*rclgo.Client{cli, cli2} {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := int64(1); i <= 20; i++ {
				sum, err := callAdd(client, int64(c)*1000, i, actionTimeout)
				if err != nil || sum != int64(c)*1000+i {
					t.Errorf("client %d request %d: sum %d, err %v", c, i, sum, err)
					return
				}
			}
		}()
	}
	wg.Wait()
}

// A service answering two requests in the reverse order gives each caller its
// own response.
func TestServiceResponsesOutOfOrder(t *testing.T) {
	var (
		mu      sync.Mutex
		pending []func()
	)
	_, _, cli := newAddService(t, "out_of_order", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		if isProbe(req) {
			respondSum(req, send) //nolint:errcheck
			return
		}
		mu.Lock()
		defer mu.Unlock()
		pending = append(pending, func() { respondSum(req, send) }) //nolint:errcheck
		if len(pending) == 2 {
			pending[1]()
			pending[0]()
		}
	})
	sums := make(chan [2]int64, 2)
	for _, b := range []int64{10, 20} {
		go func() {
			sum, err := callAdd(cli, 1, b, actionTimeout)
			if err != nil {
				t.Error(err)
			}
			sums <- [2]int64{b, sum}
		}()
		time.Sleep(50 * time.Millisecond) // the first request arrives first
	}
	for i := 0; i < 2; i++ {
		got := <-sums
		require.Equal(t, 1+got[0], got[1], "the request with b=%d got another request's response", got[0])
	}
}

// A response that arrives after its Send gave up is dropped; the next Send
// gets its own response, not the late one.
func TestServiceLateResponseAfterTimeout(t *testing.T) {
	_, _, cli := newAddService(t, "late_response", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		if req.A == 1 { // the slow request
			go func() {
				time.Sleep(500 * time.Millisecond)
				respondSum(req, send) //nolint:errcheck
			}()
			return
		}
		respondSum(req, send) //nolint:errcheck
	})
	_, err := callAdd(cli, 1, 1, 100*time.Millisecond)
	require.True(t, errors.Is(err, context.DeadlineExceeded), "slow request: %v", err)
	time.Sleep(600 * time.Millisecond) // the late response has arrived
	sum, err := callAdd(cli, 2, 3, actionTimeout)
	require.NoError(t, err)
	require.EqualValues(t, 5, sum)
}

// A handler that answers twice: the caller gets the first answer, the
// duplicate is dropped, and the next Send gets its own response.
func TestServiceDuplicateResponse(t *testing.T) {
	_, _, cli := newAddService(t, "duplicate_response", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		require.NoError(t, respondSum(req, send))
		if !isProbe(req) {
			wrong := example_interfaces_srv.NewAddTwoInts_Response()
			wrong.Sum = -1
			send.SendResponse(wrong) //nolint:errcheck // a second answer, whatever its outcome
		}
	})
	sum, err := callAdd(cli, 4, 5, actionTimeout)
	require.NoError(t, err)
	require.EqualValues(t, 9, sum)
	time.Sleep(200 * time.Millisecond) // the duplicate has arrived
	sum, err = callAdd(cli, 6, 7, actionTimeout)
	require.NoError(t, err)
	require.EqualValues(t, 13, sum, "the next Send got the duplicate response")
}
