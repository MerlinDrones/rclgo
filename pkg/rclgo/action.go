package rclgo

/*
#include <stdlib.h>
#include <rcl_action/rcl_action.h>
*/
import "C"
import (
	"context"
	"crypto/rand"
	"errors"
	"fmt"
	"reflect"
	"runtime"
	"runtime/debug"
	"strings"
	"sync"
	"time"
	"unsafe"

	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

type goalIDMessage interface {
	types.Message
	GetGoalID() *types.GoalID
	SetGoalID(id *types.GoalID)
}

type goalRequestMessage interface {
	goalIDMessage
	GetGoalDescription() types.Message
	SetGoalDescription(goal types.Message)
}

type goalResponseMessage interface {
	GetGoalAccepted() bool
}

type forEach interface {
	CallForEach(f func(interface{}))
}

type GoalStatus int8

//go:generate go run golang.org/x/tools/cmd/stringer -type GoalStatus -linecomment

const (
	GoalUnknown   GoalStatus = iota // Unknown
	GoalAccepted                    // Accepted
	GoalExecuting                   // Executing
	GoalCanceling                   // Canceling
	GoalSucceeded                   // Succeeded
	GoalCanceled                    // Canceled
	GoalAborted                     // Aborted
)

// GoalFailedError is returned by ActionClient.WatchGoal, together with the
// result, when the goal ended in a state other than GoalSucceeded (aborted,
// canceled, or unknown to the server, e.g. because its result expired).
type GoalFailedError struct {
	Status GoalStatus
}

func (e *GoalFailedError) Error() string {
	return "goal did not succeed: " + e.Status.String()
}

// resultStatus returns the Status field of a GetResult response. Every
// generated <Action>_GetResult_Response has one.
func resultStatus(resp types.Message) (GoalStatus, bool) {
	v := reflect.ValueOf(resp)
	if v.Kind() == reflect.Pointer {
		v = v.Elem()
	}
	if v.Kind() != reflect.Struct {
		return 0, false
	}
	f := v.FieldByName("Status")
	if !f.IsValid() || f.Kind() != reflect.Int8 {
		return 0, false
	}
	return GoalStatus(f.Int()), true
}

type FeedbackSender struct{ goal *GoalHandle }

func (s *FeedbackSender) Send(msg types.Message) error {
	return s.goal.server.sendFeedback(s.goal, msg)
}

// GoalHandle is a goal received by an ActionServer.
//
// Lock order for the action server: GoalHandle.resultCond.L, then
// ActionServer.rclServerMu, then ActionServer.goalsMu. Never acquire them in
// another order.
type GoalHandle struct {
	ID          types.GoalID
	Description types.Message

	server       *ActionServer
	cancelContex context.CancelFunc
	requestID    C.rmw_request_id_t
	// handle and info are set when the goal is accepted. Guarded by
	// resultCond.L.
	handle *C.rcl_action_goal_handle_t
	info   C.rcl_action_goal_info_t

	result     types.Message
	resultCond *sync.Cond
}

func newEmptyGoal(s *ActionServer, cancel context.CancelFunc) *GoalHandle {
	return &GoalHandle{
		server:       s,
		cancelContex: cancel,
		info:         C.rcl_action_get_zero_initialized_goal_info(),
		resultCond:   sync.NewCond(&sync.Mutex{}),
	}
}

func (g *GoalHandle) Server() *ActionServer { return g.server }
func (g *GoalHandle) Logger() *Logger       { return g.server.node.logger }

// status returns the state of the goal. g.resultCond.L must be held.
func (g *GoalHandle) status() (GoalStatus, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if g.handle == nil {
		return GoalUnknown, nil
	}
	var status C.schar
	// rcl reads goal handles under rclServerMu. The handle stays valid
	// after the server is closed, so closed is not checked.
	g.server.rclServerMu.Lock()
	rc := C.rcl_action_goal_handle_get_status(g.handle, &status)
	g.server.rclServerMu.Unlock()
	if rc != C.RCL_RET_OK {
		return GoalUnknown, errorsCastC(rc, "failed to get goal status")
	}
	return GoalStatus(status), nil
}

// setState applies event to the goal and publishes the new statuses.
// g.resultCond.L must be held.
func (g *GoalHandle) setState(event C.rcl_action_goal_event_t) error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	s := g.server
	s.rclServerMu.Lock()
	defer s.rclServerMu.Unlock()
	rc := C.rcl_action_update_goal_state(g.handle, event)
	if rc != C.RCL_RET_OK {
		return errorsCastC(rc, "failed to update goal state")
	}
	if !s.closed {
		s.publishStatusesLocked()
	}
	return nil
}

// Accept accepts the goal, sends the goal response and starts executing the
// goal. ExecuteGoal must call Accept before doing any work for the goal. If
// ExecuteGoal returns without calling Accept, the goal is rejected.
func (g *GoalHandle) Accept() (s *FeedbackSender, err error) {
	defer wrapErr("failed to accept goal: %w", &err)
	g.resultCond.L.Lock()
	defer g.resultCond.L.Unlock()
	if g.handle == nil {
		if err := g.server.acceptGoal(g); err != nil {
			return nil, err
		}
		if err := g.server.sendGoalResponse(g, true); err != nil {
			return nil, err
		}
		if err := g.setState(C.GOAL_EVENT_EXECUTE); err != nil {
			return nil, err
		}
	}
	return &FeedbackSender{goal: g}, nil
}

// startCancel moves the goal to CANCELING and cancels the context passed to
// ExecuteGoal.
func (g *GoalHandle) startCancel() {
	g.resultCond.L.Lock()
	defer g.resultCond.L.Unlock()
	if err := g.setState(C.GOAL_EVENT_CANCEL_GOAL); err != nil {
		// The goal finished after rcl accepted the cancel request.
		g.server.logGoalDebug(&g.ID, err)
		return
	}
	g.cancelContex()
}

// waitResult blocks until the goal reaches a terminal state and returns that
// state and the result.
func (g *GoalHandle) waitResult() (GoalStatus, types.Message) {
	g.resultCond.L.Lock()
	defer g.resultCond.L.Unlock()
	for {
		status, err := g.status()
		if err != nil {
			g.server.logGoalError(&g.ID, err)
			return GoalUnknown, nil
		}
		switch status {
		case GoalSucceeded, GoalAborted, GoalCanceled:
			return status, g.result
		default:
			g.resultCond.Wait()
		}
	}
}

type ExecuteGoalFunc = func(context.Context, *GoalHandle) (types.Message, error)

// Action executes the goals of an ActionServer.
//
// ExecuteGoal runs in its own goroutine for each goal. It must call
// goal.Accept to accept the goal; returning before that rejects the goal. The
// context is canceled when a client cancels the goal or the server is closed.
// It is not canceled when Spin stops.
//
// The goal ends in the state given by what ExecuteGoal returns:
//   - SUCCEEDED for a non-nil result and a nil error.
//   - CANCELED if a cancel was requested and ExecuteGoal returns a nil error or
//     an error matching context.Canceled. The returned result, which may be
//     partial, is sent to the client.
//   - ABORTED otherwise.
type Action interface {
	ExecuteGoal(ctx context.Context, goal *GoalHandle) (types.Message, error)
	TypeSupport() types.ActionTypeSupport
}

func NewAction(ts types.ActionTypeSupport, exec ExecuteGoalFunc) Action {
	return &action{typeSupport: ts, executeGoal: exec}
}

type action struct {
	typeSupport types.ActionTypeSupport
	executeGoal ExecuteGoalFunc
}

func (a *action) ExecuteGoal(ctx context.Context, goal *GoalHandle) (types.Message, error) {
	return a.executeGoal(ctx, goal)
}
func (a *action) TypeSupport() types.ActionTypeSupport { return a.typeSupport }

type ActionServerOptions struct {
	GoalServiceQos   qos.Profile
	CancelServiceQos qos.Profile
	ResultServiceQos qos.Profile
	FeedbackTopicQos qos.Profile
	StatusTopicQos   qos.Profile
	// ResultTimeout is how long the result of a finished goal is kept for
	// clients to fetch. Goals are expired while the server is spinning.
	ResultTimeout time.Duration
	Clock         *Clock
}

func NewDefaultActionServerOptions() *ActionServerOptions {
	// rclcpp conventions: service QoS ~= default, status topic is transient-local / keep-last(1)
	status := qos.Profile{
		History:     qos.HistoryKeepLast,
		Depth:       1,
		Reliability: qos.ReliabilityReliable,
		Durability:  qos.DurabilityTransientLocal,
		Liveliness:  qos.LivelinessSystemDefault,
	}
	return &ActionServerOptions{
		GoalServiceQos:   qos.NewDefault(),
		CancelServiceQos: qos.NewDefault(),
		ResultServiceQos: qos.NewDefault(),
		FeedbackTopicQos: qos.NewDefault(),
		StatusTopicQos:   status,
		ResultTimeout:    15 * time.Minute,
	}
}

var errActionServerClosed = errors.New("action server is closed")

type ActionServer struct {
	rosID
	life        lifetime
	node        *Node
	action      Action
	typeSupport types.ActionTypeSupport
	clock       *Clock
	// holdsClock is set while the server holds clock's lifetime.
	holdsClock bool

	// goalCtx is the parent of every goal's context. It is canceled when
	// the server is closed.
	goalCtx    context.Context
	cancelGoal context.CancelFunc

	// rclServerMu guards rclServer and closed. Every rcl call on rclServer
	// is made with it held and only if closed is false.
	rclServer   C.rcl_action_server_t
	rclServerMu sync.Mutex
	closed      bool

	goals   map[types.GoalID]*GoalHandle
	goalsMu sync.RWMutex
}

func (n *Node) NewActionServer(name string, action Action, opts *ActionServerOptions) (*ActionServer, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	// The node must stay usable while the entity is created on it.
	if !n.life.acquire() {
		return nil, closedErr("node")
	}
	defer n.life.release()
	if opts == nil {
		opts = NewDefaultActionServerOptions()
	}
	s := &ActionServer{
		node:        n,
		action:      action,
		typeSupport: action.TypeSupport(),
		clock:       opts.Clock,
		rclServer:   C.rcl_action_get_zero_initialized_server(),
		goals:       make(map[types.GoalID]*GoalHandle),
	}
	if s.clock == nil {
		s.clock = n.context.Clock()
	}
	// The server uses the clock (for goal expiry) until it is finalized.
	if !s.clock.life.acquire() {
		return nil, closedErr("clock")
	}
	s.holdsClock = true
	cname := C.CString(name)
	defer C.free(unsafe.Pointer(cname))
	rclOpts := C.rcl_action_server_options_t{
		allocator: *n.context.rcl_allocator_t,
		result_timeout: C.rcl_duration_t{
			nanoseconds: C.int64_t(opts.ResultTimeout),
		},
	}
	qosToC(opts.GoalServiceQos, &rclOpts.goal_service_qos)
	qosToC(opts.CancelServiceQos, &rclOpts.cancel_service_qos)
	qosToC(opts.ResultServiceQos, &rclOpts.result_service_qos)
	qosToC(opts.FeedbackTopicQos, &rclOpts.feedback_topic_qos)
	qosToC(opts.StatusTopicQos, &rclOpts.status_topic_qos)
	s.clock.jumpMu.Lock()
	rc := C.rcl_action_server_init(
		&s.rclServer,
		n.rcl_node_t,
		s.clock.rcl_clock_t,
		(*C.rosidl_action_type_support_t)(s.typeSupport.TypeSupport()),
		cname,
		&rclOpts,
	)
	s.clock.jumpMu.Unlock()
	if rc != C.RCL_RET_OK {
		s.holdsClock = false
		s.clock.life.release()
		return nil, errorsCastC(rc, "failed to create action server")
	}
	s.goalCtx, s.cancelGoal = context.WithCancel(context.Background())
	if !n.addResource(s) {
		s.Close() //nolint:errcheck // refused before it was added
		return nil, closedErr("node")
	}
	return s, nil
}

// Close finalizes the server and cancels the context of every goal that is
// still executing. Goals that finish after Close cannot report their result.
func (s *ActionServer) Close() error {
	return closeEntity(&s.life, "action server", func() {
		s.rclServerMu.Lock()
		s.closed = true
		s.rclServerMu.Unlock()
		s.node.removeResource(s)
		if s.cancelGoal != nil {
			s.cancelGoal()
		}
	}, &s.node.life, s.fini)
}

func (s *ActionServer) fini() (err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	defer func() {
		if s.holdsClock {
			s.holdsClock = false
			s.clock.life.release()
		}
	}()
	s.rclServerMu.Lock()
	defer s.rclServerMu.Unlock()
	s.clock.jumpMu.Lock()
	rc := C.rcl_action_server_fini(&s.rclServer, s.node.rcl_node_t)
	s.clock.jumpMu.Unlock()
	if rc != C.RCL_RET_OK {
		err = errorsCastC(rc, "failed to finalize action server")
	}
	s.rclServer = C.rcl_action_get_zero_initialized_server()
	return err
}

func (s *ActionServer) Node() *Node { return s.node }

// withServer calls f with rclServerMu held, unless the server is closed.
func (s *ActionServer) withServer(f func() error) error {
	s.rclServerMu.Lock()
	defer s.rclServerMu.Unlock()
	if s.closed {
		return errActionServerClosed
	}
	return f()
}

func isActionServerTakeFailed(err error) bool {
	var takeFailed *ActionServerTakeFailed
	return errors.As(err, &takeFailed)
}

func (s *ActionServer) takeGoalRequest(goal *GoalHandle) error {
	ts := s.typeSupport.SendGoal().Request()
	reqBuf := ts.PrepareMemory()
	defer ts.ReleaseMemory(reqBuf)
	err := s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		rc := C.rcl_action_take_goal_request(&s.rclServer, &goal.requestID, reqBuf)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "failed to take goal request")
		}
		return nil
	})
	if err != nil {
		return err
	}
	req := ts.New().(goalRequestMessage)
	ts.AsGoStruct(req, reqBuf)
	goal.ID = *req.GetGoalID()
	goal.Description = req.GetGoalDescription()
	return nil
}

// acceptGoal registers goal with rcl and the server. goal.resultCond.L must be
// held.
func (s *ActionServer) acceptGoal(goal *GoalHandle) error {
	goal.info.goal_id.uuid = *(*[types.GoalIDLen]C.uchar)(unsafe.Pointer(&goal.ID))
	return s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		s.goalsMu.Lock()
		defer s.goalsMu.Unlock()
		if s.goals[goal.ID] != nil {
			return fmt.Errorf("goal with ID %v has already been accepted", goal.ID)
		}
		handle := C.rcl_action_accept_new_goal(&s.rclServer, &goal.info)
		if handle == nil {
			return errors.New("accepting a goal handle failed: " + errorString())
		}
		goal.handle = (*C.rcl_action_goal_handle_t)(C.malloc(C.sizeof_rcl_action_goal_handle_t))
		*goal.handle = *handle
		runtime.SetFinalizer(goal, func(g *GoalHandle) {
			if g.handle != nil {
				C.rcl_action_goal_handle_fini(g.handle)
				C.free(unsafe.Pointer(g.handle))
				g.handle = nil
			}
		})
		s.goals[goal.ID] = goal
		// rcl stamps its own copy of the goal info with the acceptance
		// time. Read it back so the goal response carries that stamp.
		if rc := C.rcl_action_goal_handle_get_info(goal.handle, &goal.info); rc != C.RCL_RET_OK {
			return errorsCastC(rc, "failed to get goal info")
		}
		s.publishStatusesLocked()
		return nil
	})
}

// sendGoalResponse answers the goal request of goal. goal.resultCond.L must be
// held.
func (s *ActionServer) sendGoalResponse(goal *GoalHandle, accepted bool) error {
	var stamp time.Duration
	if accepted {
		stamp = time.Duration(goal.info.stamp.sec)*time.Second + time.Duration(goal.info.stamp.nanosec)
	} else {
		var err error
		if stamp, err = s.clock.now(); err != nil {
			return err
		}
	}
	resp := s.typeSupport.NewSendGoalResponse(accepted, stamp)
	ts := s.typeSupport.SendGoal().Response()
	buf := ts.PrepareMemory()
	defer ts.ReleaseMemory(buf)
	ts.AsCStruct(buf, resp)
	return s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		rc := C.rcl_action_send_goal_response(&s.rclServer, &goal.requestID, buf)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "failed to send goal response")
		}
		return nil
	})
}

func (s *ActionServer) handleGoalRequest() {
	ctx, cancel := context.WithCancel(s.goalCtx)
	goal := newEmptyGoal(s, cancel)
	if err := s.takeGoalRequest(goal); err != nil {
		cancel()
		if !isActionServerTakeFailed(err) && !errors.Is(err, errActionServerClosed) {
			s.node.Logger().Error(err)
		}
		return
	}
	go s.executeGoal(ctx, goal)
}

func (s *ActionServer) executeGoal(ctx context.Context, goal *GoalHandle) {
	defer goal.cancelContex()
	var (
		result types.Message
		err    error
	)
	func() {
		defer func() {
			if r := recover(); r != nil {
				err = fmt.Errorf("rclgo: panic when executing goal: %v", r)
			}
		}()
		result, err = s.action.ExecuteGoal(ctx, goal)
	}()
	s.finishGoal(goal, result, err)
}

// finishGoal moves goal to the terminal state implied by the values returned
// by ExecuteGoal, or rejects it if it was never accepted.
func (s *ActionServer) finishGoal(goal *GoalHandle, result types.Message, execErr error) {
	goal.resultCond.L.Lock()
	defer goal.resultCond.L.Unlock()
	if goal.handle == nil {
		if execErr == nil {
			s.logGoalError(&goal.ID, "ExecuteGoal returned without calling Accept, rejecting the goal")
		}
		if err := s.sendGoalResponse(goal, false); err != nil {
			s.logGoalDebug(&goal.ID, err)
		}
		return
	}
	status, err := goal.status()
	if err != nil {
		s.logGoalError(&goal.ID, err)
		return
	}
	if status == GoalAccepted {
		// Accept failed after rcl accepted the goal. ABORT is only valid
		// from EXECUTING.
		if err := goal.setState(C.GOAL_EVENT_EXECUTE); err != nil {
			s.logGoalError(&goal.ID, err)
		}
		status = GoalExecuting
	}
	var event C.rcl_action_goal_event_t
	switch {
	case status == GoalCanceling && (execErr == nil || errors.Is(execErr, context.Canceled)):
		event = C.GOAL_EVENT_CANCELED
	case execErr == nil && result != nil:
		event = C.GOAL_EVENT_SUCCEED
	default:
		if execErr == nil {
			s.logGoalError(&goal.ID, "a nil result was returned even though the goal was executed successfully")
		} else {
			s.logGoalDebug(&goal.ID, "aborting: ", execErr)
		}
		event = C.GOAL_EVENT_ABORT
	}
	if err := goal.setState(event); err != nil {
		s.logGoalError(&goal.ID, err)
	}
	goal.result = result
	err = s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		if rc := C.rcl_action_notify_goal_done(&s.rclServer); rc != C.RCL_RET_OK {
			return errorsCastC(rc, "failed to notify that goal is done")
		}
		return nil
	})
	if err != nil {
		s.logGoalDebug(&goal.ID, err)
	}
	goal.resultCond.Broadcast()
}

func (s *ActionServer) expireGoals() error {
	var numExpired C.size_t
	expiredGoals := make([]C.rcl_action_goal_info_t, 10)
	return s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		s.goalsMu.Lock()
		defer s.goalsMu.Unlock()
		for {
			rc := C.rcl_action_expire_goals(
				&s.rclServer,
				unsafe.SliceData(expiredGoals),
				C.size_t(len(expiredGoals)),
				&numExpired,
			)
			if rc != C.RCL_RET_OK {
				return errorsCastC(rc, "failed to expire goals")
			}
			if numExpired == 0 {
				return nil
			}
			for _, goal := range expiredGoals[:int(numExpired)] {
				delete(s.goals, *(*types.GoalID)(unsafe.Pointer(&goal.goal_id.uuid)))
			}
		}
	})
}

func (s *ActionServer) takeResultRequest(header *C.rmw_request_id_t) (goalIDMessage, error) {
	ts := s.typeSupport.GetResult().Request()
	reqBuf := ts.PrepareMemory()
	defer ts.ReleaseMemory(reqBuf)
	err := s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		rc := C.rcl_action_take_result_request(&s.rclServer, header, reqBuf)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "failed to take result request")
		}
		return nil
	})
	if err != nil {
		return nil, err
	}
	req := ts.New().(goalIDMessage)
	ts.AsGoStruct(req, reqBuf)
	return req, nil
}

func (s *ActionServer) getGoal(id *types.GoalID) *GoalHandle {
	s.goalsMu.RLock()
	defer s.goalsMu.RUnlock()
	return s.goals[*id]
}

func (s *ActionServer) sendResultResponse(status GoalStatus, result types.Message, header *C.rmw_request_id_t) error {
	ts := s.typeSupport.GetResult().Response()
	resp := s.typeSupport.NewGetResultResponse(int8(status), result)
	buf := ts.PrepareMemory()
	defer ts.ReleaseMemory(buf)
	ts.AsCStruct(buf, resp)
	return s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		rc := C.rcl_action_send_result_response(&s.rclServer, header, buf)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "failed to send result response")
		}
		return nil
	})
}

func (s *ActionServer) handleResultRequest() {
	var header C.rmw_request_id_t
	req, err := s.takeResultRequest(&header)
	if err != nil {
		if !isActionServerTakeFailed(err) && !errors.Is(err, errActionServerClosed) {
			s.node.Logger().Error(err)
		}
		return
	}
	go func() {
		id := req.GetGoalID()
		status, result := GoalUnknown, types.Message(nil)
		if goal := s.getGoal(id); goal != nil {
			status, result = goal.waitResult()
		}
		if err := s.sendResultResponse(status, result, &header); err != nil {
			s.logGoalError(id, err)
		}
		if err := s.expireGoals(); err != nil && !errors.Is(err, errActionServerClosed) {
			s.logGoalError(id, err)
		}
	}()
}

func (s *ActionServer) handleCancelRequest() {
	ts := s.typeSupport.CancelGoal()
	reqBuf := ts.Request().PrepareMemory()
	var header C.rmw_request_id_t
	err := s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		rc := C.rcl_action_take_cancel_request(&s.rclServer, &header, reqBuf)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "failed to take cancel request")
		}
		return nil
	})
	if err != nil {
		ts.Request().ReleaseMemory(reqBuf)
		if !isActionServerTakeFailed(err) && !errors.Is(err, errActionServerClosed) {
			s.node.Logger().Error(err)
		}
		return
	}
	go func() {
		defer ts.Request().ReleaseMemory(reqBuf)
		respBuf := C.rcl_action_get_zero_initialized_cancel_response()
		defer func() {
			runtime.LockOSThread() // rcl error state is per OS thread; see errorString
			defer runtime.UnlockOSThread()
			if rc := C.rcl_action_cancel_response_fini(&respBuf); rc != C.RCL_RET_OK {
				s.node.Logger().Error(errorsCastC(rc, "failed to finalize cancel response")) //nolint:errcheck
			}
		}()
		err := s.withServer(func() error {
			runtime.LockOSThread() // rcl error state is per OS thread; see errorString
			defer runtime.UnlockOSThread()
			rc := C.rcl_action_process_cancel_request(
				&s.rclServer,
				(*C.rcl_action_cancel_request_t)(reqBuf),
				&respBuf,
			)
			if rc != C.RCL_RET_OK {
				return errorsCastC(rc, "failed to process cancel request")
			}
			return nil
		})
		if errors.Is(err, errActionServerClosed) {
			return
		}
		if err != nil {
			// The client must still get an answer. No goal was moved to
			// CANCELING, so report the request as rejected with an empty
			// list (the data, if any, is freed by the deferred fini).
			s.node.Logger().Error(err)
			respBuf.msg.return_code = C.action_msgs__srv__CancelGoal_Response__ERROR_REJECTED
			respBuf.msg.goals_canceling.size = 0
		} else {
			resp := ts.Response().New()
			ts.Response().AsGoStruct(resp, unsafe.Pointer(&respBuf.msg))
			var toCancel []*GoalHandle
			s.goalsMu.RLock()
			resp.(forEach).CallForEach(func(id interface{}) {
				if goal := s.goals[*id.(*types.GoalID)]; goal != nil {
					toCancel = append(toCancel, goal)
				}
			})
			s.goalsMu.RUnlock()
			for _, goal := range toCancel {
				goal.startCancel()
			}
		}
		err = s.withServer(func() error {
			runtime.LockOSThread() // rcl error state is per OS thread; see errorString
			defer runtime.UnlockOSThread()
			rc := C.rcl_action_send_cancel_response(&s.rclServer, &header, unsafe.Pointer(&respBuf.msg))
			if rc != C.RCL_RET_OK {
				return errorsCastC(rc, "failed to send cancel response")
			}
			return nil
		})
		if err != nil && !errors.Is(err, errActionServerClosed) {
			s.node.Logger().Error(err)
		}
	}()
}

func (s *ActionServer) sendFeedback(goal *GoalHandle, fb types.Message) error {
	msg := s.typeSupport.NewFeedbackMessage(&goal.ID, fb)
	ts := s.typeSupport.FeedbackMessage()
	buf := ts.PrepareMemory()
	defer ts.ReleaseMemory(buf)
	ts.AsCStruct(buf, msg)
	return s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		rc := C.rcl_action_publish_feedback(&s.rclServer, buf)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "failed to send feedback")
		}
		return nil
	})
}

// publishStatusesLocked publishes the status of every goal. rclServerMu must be
// held and the server must not be closed.
func (s *ActionServer) publishStatusesLocked() {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	statuses := C.rcl_action_get_zero_initialized_goal_status_array()
	defer func() {
		if rc := C.rcl_action_goal_status_array_fini(&statuses); rc != C.RCL_RET_OK {
			s.node.Logger().Error(errorsCastC(rc, "failed to finalize status array")) //nolint:errcheck
		}
	}()
	rc := C.rcl_action_get_goal_status_array(&s.rclServer, &statuses)
	if rc != C.RCL_RET_OK {
		s.node.Logger().Error("failed to get status array: ", errorsCast(rc))
		return
	}
	rc = C.rcl_action_publish_status(&s.rclServer, unsafe.Pointer(&statuses.msg))
	if rc != C.RCL_RET_OK {
		s.node.Logger().Error("failed to publish status array: ", errorsCast(rc))
	}
}

func (s *ActionServer) handleReadyEntities(ws *WaitSet) {
	var goalReq, cancelReq, resultReq, expired C.bool
	err := s.withServer(func() error {
		runtime.LockOSThread() // rcl error state is per OS thread; see errorString
		defer runtime.UnlockOSThread()
		rc := C.rcl_action_server_wait_set_get_entities_ready(
			&ws.rcl_wait_set_t,
			&s.rclServer,
			&goalReq,
			&cancelReq,
			&resultReq,
			&expired,
		)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "failed to get ready entities")
		}
		return nil
	})
	if err != nil {
		if !errors.Is(err, errActionServerClosed) {
			s.node.Logger().Error(err)
		}
		return
	}
	if goalReq {
		s.handleGoalRequest()
	}
	if cancelReq {
		s.handleCancelRequest()
	}
	if resultReq {
		s.handleResultRequest()
	}
	if expired {
		if err := s.expireGoals(); err != nil && !errors.Is(err, errActionServerClosed) {
			s.node.logger.Error("failed to expire goals: ", err)
		}
	}
}

func (s *ActionServer) logGoalError(id *types.GoalID, a ...interface{}) {
	s.node.Logger().Error(goalLogMessage(id, a...))
}

func (s *ActionServer) logGoalDebug(id *types.GoalID, a ...interface{}) {
	s.node.Logger().Debug(goalLogMessage(id, a...))
}

func goalLogMessage(id *types.GoalID, a ...interface{}) string {
	var b strings.Builder
	fmt.Fprint(&b, "goal: ")
	if id == nil {
		fmt.Fprint(&b, "<unknown>")
	} else {
		fmt.Fprint(&b, id.String())
	}
	fmt.Fprint(&b, ": ")
	fmt.Fprint(&b, a...)
	return b.String()
}

type FeedbackHandler func(context.Context, types.Message)
type StatusHandler func(context.Context, types.Message)

type ActionClientOptions struct {
	GoalServiceQos   qos.Profile
	CancelServiceQos qos.Profile
	ResultServiceQos qos.Profile
	FeedbackTopicQos qos.Profile
	StatusTopicQos   qos.Profile
}

func NewDefaultActionClientOptions() *ActionClientOptions {
	// The status topic is transient-local with depth 1, matching the server
	// and rclcpp, so a client also receives the current status of goals that
	// were already running when it subscribed.
	status := qos.Profile{
		History:     qos.HistoryKeepLast,
		Depth:       1,
		Reliability: qos.ReliabilityReliable,
		Durability:  qos.DurabilityTransientLocal,
		Liveliness:  qos.LivelinessSystemDefault,
	}
	return &ActionClientOptions{
		GoalServiceQos:   qos.NewDefault(),
		CancelServiceQos: qos.NewDefault(),
		ResultServiceQos: qos.NewDefault(),
		FeedbackTopicQos: qos.NewDefault(),
		StatusTopicQos:   status,
	}
}

type actionClientHandler = func(context.Context, types.Message)

// handlerQueue delivers messages to one feedback or status handler in the
// order they arrived, from at most one goroutine at a time, as rclcpp and rclpy
// deliver them in order on their executor. It stops delivering when stopped or
// when its context ends; a handler call already running is not interrupted.
type handlerQueue struct {
	ctx     context.Context //nolint:containedctx // track lifetimes
	handler actionClientHandler
	logger  *Logger

	mu      sync.Mutex
	pending []types.Message
	running bool
	stopped bool
}

func (q *handlerQueue) push(msg types.Message) {
	q.mu.Lock()
	defer q.mu.Unlock()
	if q.stopped || q.ctx.Err() != nil {
		return
	}
	q.pending = append(q.pending, msg)
	if !q.running {
		q.running = true
		go q.drain()
	}
}

func (q *handlerQueue) drain() {
	for {
		q.mu.Lock()
		if q.stopped || q.ctx.Err() != nil || len(q.pending) == 0 {
			q.running = false
			if q.stopped || q.ctx.Err() != nil {
				q.pending = nil
			}
			q.mu.Unlock()
			return
		}
		msg := q.pending[0]
		q.pending[0] = nil
		q.pending = q.pending[1:]
		q.mu.Unlock()
		q.deliver(msg)
	}
}

// deliver calls the handler. A panicking handler is logged instead of ending
// the process, since it runs on a goroutine of rclgo's.
func (q *handlerQueue) deliver(msg types.Message) {
	defer func() {
		if r := recover(); r != nil {
			q.logger.Errorf("action client handler panicked: %v\n%s", r, debug.Stack()) //nolint:errcheck
		}
	}()
	q.handler(q.ctx, msg)
}

func (q *handlerQueue) stop() {
	q.mu.Lock()
	defer q.mu.Unlock()
	q.stopped = true
	q.pending = nil
}

type actionClientHandlerMap map[uint64]*handlerQueue

func (m actionClientHandlerMap) call(msg types.Message) {
	for _, q := range m {
		q.push(msg.CloneMsg())
	}
}

type actionClientSubs struct {
	perGoal  map[types.GoalID]actionClientHandlerMap
	allGoals actionClientHandlerMap
}

func newActionClientHandlers() actionClientSubs {
	return actionClientSubs{
		perGoal:  make(map[types.GoalID]actionClientHandlerMap),
		allGoals: make(actionClientHandlerMap),
	}
}

type ActionClient struct {
	rosID
	life lifetime
	node *Node

	typeSupport types.ActionTypeSupport
	rclClient   C.rcl_action_client_t
	rclClientMu sync.Mutex

	goalSender   requestSender
	resultSender requestSender
	cancelSender requestSender

	nextSubscriberID uint64
	feedbackSubs     actionClientSubs
	statusSubs       actionClientSubs

	// lastStatus is the latest status of every goal the server reported,
	// delivered to a status watcher when it subscribes. Guarded by
	// rclClientMu.
	lastStatus map[types.GoalID]types.Message
}

func (n *Node) NewActionClient(name string, ts types.ActionTypeSupport, opts *ActionClientOptions) (*ActionClient, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	// The node must stay usable while the entity is created on it.
	if !n.life.acquire() {
		return nil, closedErr("node")
	}
	defer n.life.release()
	if opts == nil {
		opts = NewDefaultActionClientOptions()
	}
	c := &ActionClient{
		node:         n,
		typeSupport:  ts,
		rclClient:    C.rcl_action_get_zero_initialized_client(),
		feedbackSubs: newActionClientHandlers(),
		statusSubs:   newActionClientHandlers(),
		lastStatus:   make(map[types.GoalID]types.Message),
	}
	c.goalSender = newRequestSender(requestSenderTransport{
		SendRequest:  c.sendGoalRequest,
		TakeResponse: c.takeGoalResponse,
		TypeSupport:  ts.SendGoal(),
		Logger:       n.Logger(),
	})
	c.resultSender = newRequestSender(requestSenderTransport{
		SendRequest:  c.sendResultRequest,
		TakeResponse: c.takeResultResponse,
		TypeSupport:  ts.GetResult(),
		Logger:       n.Logger(),
	})
	c.cancelSender = newRequestSender(requestSenderTransport{
		SendRequest:  c.sendCancelRequest,
		TakeResponse: c.takeCancelResponse,
		TypeSupport:  ts.CancelGoal(),
		Logger:       n.Logger(),
	})
	cname := C.CString(name)
	defer C.free(unsafe.Pointer(cname))
	rclOpts := C.rcl_action_client_options_t{
		allocator: *n.context.rcl_allocator_t,
	}
	qosToC(opts.GoalServiceQos, &rclOpts.goal_service_qos)
	qosToC(opts.CancelServiceQos, &rclOpts.cancel_service_qos)
	qosToC(opts.ResultServiceQos, &rclOpts.result_service_qos)
	qosToC(opts.FeedbackTopicQos, &rclOpts.feedback_topic_qos)
	qosToC(opts.StatusTopicQos, &rclOpts.status_topic_qos)
	rc := C.rcl_action_client_init(
		&c.rclClient,
		n.rcl_node_t,
		(*C.rosidl_action_type_support_t)(ts.TypeSupport()),
		cname,
		&rclOpts,
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCastC(rc, "failed to create action client")
	}
	if !n.addResource(c) {
		c.Close() //nolint:errcheck // refused before it was added
		return nil, closedErr("node")
	}
	return c, nil
}

func (c *ActionClient) Close() error {
	return closeEntity(&c.life, "action client", func() {
		c.node.removeResource(c)
		// Wake callers waiting for a response right away.
		c.cancelSender.Close() //nolint:errcheck
		c.goalSender.Close()   //nolint:errcheck
		c.resultSender.Close() //nolint:errcheck
	}, &c.node.life, c.fini)
}

func (c *ActionClient) fini() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var err error
	c.rclClientMu.Lock()
	defer c.rclClientMu.Unlock()
	rc := C.rcl_action_client_fini(&c.rclClient, c.node.rcl_node_t)
	if rc != C.RCL_RET_OK {
		err = errors.Join(err, errorsCastC(rc, "failed to finalize action client"))
	}
	// typeSupport stays set: methods called after Close use it to build their
	// request before the rcl call reports ErrClosed.
	return err
}

func (c *ActionClient) Node() *Node { return c.node }

// WatchGoal sends goal, calls onFeedback (if not nil) with its feedback, in
// order, and waits for its result. If the goal is rejected it returns an error;
// if it ends in a state other than GoalSucceeded it returns the result and a
// *GoalFailedError with the final status.
func (c *ActionClient) WatchGoal(ctx context.Context, goal types.Message, onFeedback FeedbackHandler) (result types.Message, goalID *types.GoalID, retErr error) {
	req, err := c.newSendGoalRequest(goal)
	if err != nil {
		return nil, nil, err
	}
	if onFeedback != nil {
		var cancel context.CancelFunc
		ctx, cancel = context.WithCancel(ctx)
		defer cancel()
		unsub := c.subscribe(ctx, &c.feedbackSubs, req.GetGoalID(), onFeedback)
		defer unsub()
	}
	// Registered before the goal is sent: if ctx ends while the goal request
	// is in flight, the server may already have accepted the goal, and it
	// must not be left running. Canceling a goal the server never accepted
	// is harmless. The cancel request is sent without waiting for its
	// response (like rclcpp's async_cancel_goal): Spin may have stopped with
	// the same ctx, and then no response would be taken.
	defer func() {
		if ctx.Err() != nil {
			cancelReq := c.typeSupport.CancelGoal().Request().New()
			cancelReq.(goalIDMessage).SetGoalID(req.GetGoalID())
			if err := c.cancelSender.sendNoWait(cancelReq); err != nil && !errors.Is(err, ErrClosed) {
				retErr = errors.Join(err, retErr)
			}
		}
	}()
	resp, err := c.SendGoalRequest(ctx, req)
	if err != nil {
		return nil, req.GetGoalID(), err
	}
	if !resp.(goalResponseMessage).GetGoalAccepted() {
		return nil, req.GetGoalID(), errors.New("goal was rejected")
	}
	result, err = c.GetResult(ctx, req.GetGoalID())
	if err != nil {
		return result, req.GetGoalID(), err
	}
	if status, ok := resultStatus(result); ok && status != GoalSucceeded {
		return result, req.GetGoalID(), &GoalFailedError{Status: status}
	}
	return result, req.GetGoalID(), nil
}

func (c *ActionClient) SendGoal(ctx context.Context, goal types.Message) (types.Message, *types.GoalID, error) {
	msg, err := c.newSendGoalRequest(goal)
	if err != nil {
		return nil, nil, err
	}
	resp, err := c.SendGoalRequest(ctx, msg)
	return resp, msg.GetGoalID(), err
}

func (c *ActionClient) newSendGoalRequest(goal types.Message) (goalRequestMessage, error) {
	msg := c.typeSupport.SendGoal().Request().New().(goalRequestMessage)
	goalID := msg.GetGoalID()
	if _, err := rand.Read(goalID[:]); err != nil {
		return nil, fmt.Errorf("failed to generate goal ID: %v", err)
	}
	msg.SetGoalDescription(goal)
	return msg, nil
}

func (c *ActionClient) SendGoalRequest(ctx context.Context, request types.Message) (types.Message, error) {
	resp, _, err := c.goalSender.Send(ctx, request)
	return resp, err
}

func (c *ActionClient) sendGoalRequest(req unsafe.Pointer) (C.int64_t, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !c.life.acquire() {
		return 0, closedErr("action client")
	}
	defer c.life.release()
	var seqNum C.int64_t
	rc := C.rcl_action_send_goal_request(&c.rclClient, req, &seqNum)
	if rc != C.RCL_RET_OK {
		return 0, errorsCastC(rc, "failed to send goal request")
	}
	return seqNum, nil
}

func (c *ActionClient) takeGoalResponse(resp unsafe.Pointer) (C.int64_t, interface{}, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var header C.rmw_request_id_t
	switch rc := C.rcl_action_take_goal_response(&c.rclClient, &header, resp); rc {
	case C.RCL_RET_OK:
		return header.sequence_number, nil, nil
	case C.RCL_RET_ACTION_CLIENT_TAKE_FAILED:
		return 0, nil, errTakeFailed
	default:
		return 0, nil, errorsCastC(rc, "failed to take goal response")
	}
}

func (c *ActionClient) GetResult(ctx context.Context, goalID *types.GoalID) (types.Message, error) {
	msg := c.typeSupport.GetResult().Request().New()
	msg.(goalIDMessage).SetGoalID(goalID)
	resp, _, err := c.resultSender.Send(ctx, msg)
	return resp, err
}

func (c *ActionClient) sendResultRequest(req unsafe.Pointer) (C.int64_t, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !c.life.acquire() {
		return 0, closedErr("action client")
	}
	defer c.life.release()
	var seqNum C.int64_t
	rc := C.rcl_action_send_result_request(&c.rclClient, req, &seqNum)
	if rc != C.RCL_RET_OK {
		return 0, errorsCastC(rc, "failed to send result request")
	}
	return seqNum, nil
}

func (c *ActionClient) takeResultResponse(resp unsafe.Pointer) (C.int64_t, interface{}, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var header C.rmw_request_id_t
	switch rc := C.rcl_action_take_result_response(&c.rclClient, &header, resp); rc {
	case C.RCL_RET_OK:
		return header.sequence_number, nil, nil
	case C.RCL_RET_ACTION_CLIENT_TAKE_FAILED:
		return 0, nil, errTakeFailed
	default:
		return 0, nil, errorsCastC(rc, "failed to take result response")
	}
}

func (c *ActionClient) CancelGoal(ctx context.Context, request types.Message) (types.Message, error) {
	resp, _, err := c.cancelSender.Send(ctx, request)
	return resp, err
}

func (c *ActionClient) sendCancelRequest(req unsafe.Pointer) (C.int64_t, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !c.life.acquire() {
		return 0, closedErr("action client")
	}
	defer c.life.release()
	var seqNum C.int64_t
	rc := C.rcl_action_send_cancel_request(&c.rclClient, req, &seqNum)
	if rc != C.RCL_RET_OK {
		return 0, errorsCastC(rc, "failed to send cancel request")
	}
	return seqNum, nil
}

func (c *ActionClient) takeCancelResponse(resp unsafe.Pointer) (C.int64_t, interface{}, error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var header C.rmw_request_id_t
	switch rc := C.rcl_action_take_cancel_response(&c.rclClient, &header, resp); rc {
	case C.RCL_RET_OK:
		return header.sequence_number, nil, nil
	case C.RCL_RET_ACTION_CLIENT_TAKE_FAILED:
		return 0, nil, errTakeFailed
	default:
		return 0, nil, errorsCastC(rc, "failed to take cancel response")
	}
}

func (c *ActionClient) WatchFeedback(ctx context.Context, goalID *types.GoalID, handler FeedbackHandler) <-chan error {
	unsub := c.subscribe(ctx, &c.feedbackSubs, goalID, handler)
	errc := make(chan error, 1)
	go func() {
		defer unsub()
		<-ctx.Done()
		errc <- ctx.Err()
	}()
	return errc
}

func (c *ActionClient) subscribe(ctx context.Context, subs *actionClientSubs, id *types.GoalID, handler actionClientHandler) (unsubscribe func()) {
	c.rclClientMu.Lock()
	defer c.rclClientMu.Unlock()
	q := &handlerQueue{ctx: ctx, handler: handler, logger: c.node.Logger()}
	// A status watcher first gets the latest known status, so that it also
	// learns the state of goals whose status does not change again.
	if subs == &c.statusSubs {
		for goalID, status := range c.lastStatus {
			if id == nil || goalID == *id {
				q.push(status.CloneMsg())
			}
		}
	}
	if id == nil {
		subID := c.nextSubscriberID
		c.nextSubscriberID++
		subs.allGoals[subID] = q
		return func() {
			q.stop()
			c.rclClientMu.Lock()
			defer c.rclClientMu.Unlock()
			delete(subs.allGoals, subID)
		}
	}
	goalID := *id
	handlers := subs.perGoal[goalID]
	if handlers == nil {
		handlers = make(actionClientHandlerMap)
		subs.perGoal[goalID] = handlers
	}
	subID := c.nextSubscriberID
	c.nextSubscriberID++
	handlers[subID] = q
	return func() {
		q.stop()
		c.rclClientMu.Lock()
		defer c.rclClientMu.Unlock()
		if handlers := subs.perGoal[goalID]; handlers != nil {
			if _, ok := handlers[subID]; ok {
				delete(handlers, subID)
				if len(handlers) == 0 {
					delete(subs.perGoal, goalID)
				}
			}
		}
	}
}

func (c *ActionClient) handleFeedback() {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	ts := c.typeSupport.FeedbackMessage()
	buf := ts.PrepareMemory()
	defer ts.ReleaseMemory(buf)
	switch rc := C.rcl_action_take_feedback(&c.rclClient, buf); rc {
	case C.RCL_RET_OK:
		msg := ts.New().(goalIDMessage)
		ts.AsGoStruct(msg, buf)
		c.feedbackSubs.allGoals.call(msg)
		c.feedbackSubs.perGoal[*msg.GetGoalID()].call(msg)
	case C.RCL_RET_ACTION_CLIENT_TAKE_FAILED:
	default:
		c.node.Logger().Error(errorsCastC(rc, "failed to take feedback"))
	}
}

func (c *ActionClient) WatchStatus(ctx context.Context, goalID *types.GoalID, handler StatusHandler) <-chan error {
	unsub := c.subscribe(ctx, &c.statusSubs, goalID, handler)
	errc := make(chan error, 1)
	go func() {
		defer unsub()
		<-ctx.Done()
		errc <- ctx.Err()
	}()
	return errc
}

func (c *ActionClient) handleStatus() {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	ts := c.typeSupport.GoalStatusArray()
	buf := ts.PrepareMemory()
	defer ts.ReleaseMemory(buf)
	switch rc := C.rcl_action_take_status(&c.rclClient, buf); rc {
	case C.RCL_RET_OK:
		msg := ts.New()
		ts.AsGoStruct(msg, buf)
		// The status array lists every goal the server still knows.
		clear(c.lastStatus)
		msg.(forEach).CallForEach(func(info interface{}) {
			msg := info.(goalIDMessage)
			c.lastStatus[*msg.GetGoalID()] = msg
			c.statusSubs.perGoal[*msg.GetGoalID()].call(msg)
			c.statusSubs.allGoals.call(msg)
		})
	case C.RCL_RET_ACTION_CLIENT_TAKE_FAILED:
	default:
		c.node.Logger().Error(errorsCastC(rc, "failed to take status"))
	}
}

func (c *ActionClient) handleReadyEntities(ws *WaitSet) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	var feedback, status, goalResp, cancelResp, resultResp C.bool
	c.rclClientMu.Lock()
	defer c.rclClientMu.Unlock()
	rc := C.rcl_action_client_wait_set_get_entities_ready(
		&ws.rcl_wait_set_t,
		&c.rclClient,
		&feedback,
		&status,
		&goalResp,
		&cancelResp,
		&resultResp,
	)
	if rc != C.RCL_RET_OK {
		c.node.Logger().Error(errorsCastC(rc, "failed to get ready entities"))
		return
	}
	if feedback {
		c.handleFeedback()
	}
	if status {
		c.handleStatus()
	}
	if goalResp {
		c.goalSender.HandleResponse()
	}
	if cancelResp {
		c.cancelSender.HandleResponse()
	}
	if resultResp {
		c.resultSender.HandleResponse()
	}
}

func wrapErr(format string, err *error, a ...interface{}) {
	if *err != nil {
		args := make([]interface{}, 0, 1+len(a))
		args = append(args, *err)
		args = append(args, a...)
		*err = fmt.Errorf(format, args...)
	}
}
