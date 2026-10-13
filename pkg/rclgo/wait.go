package rclgo

/*
#include <rcl/wait.h>
#include <rcl_action/wait.h>
*/
import "C"

import (
	"context"
	"errors"
	"runtime"
	"sync"
	"time"
	"unsafe"
)

// errWaitSetClosed is returned by Run when the wait set or its context is
// closed while it runs.
var errWaitSetClosed = errors.New("wait set was closed")

type WaitSet struct {
	rosID
	Subscriptions   []*Subscription
	Timers          []*Timer
	Services        []*Service
	Clients         []*Client
	ActionClients   []*ActionClient
	ActionServers   []*ActionServer
	guardConditions []*guardCondition
	rcl_wait_set_t  C.rcl_wait_set_t
	cancelWait      *guardCondition
	context         *Context

	// source, if set, is the resource store the entity lists are rebuilt
	// from whenever refresh is triggered (see followResources).
	source  *rosResourceStore
	refresh *guardCondition

	life lifetime

	// stopRun, while Run is running, makes it return.
	stopMu  sync.Mutex
	stopRun context.CancelFunc

	// skipped records entities left to another running wait set, so that
	// each is reported once.
	skipped map[*lifetime]bool
}

func NewWaitSet() (*WaitSet, error) {
	c := DefaultContext()
	if c == nil {
		return nil, errInitNotCalled
	}
	return c.NewWaitSet()
}

func (c *Context) NewWaitSet() (ws *WaitSet, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	// The context must stay usable while the entity is created on it.
	if !c.life.acquire() {
		return nil, closedErr("context")
	}
	defer c.life.release()
	const (
		subscriptionsCount   = 0
		guardConditionsCount = 0
		timersCount          = 0
		clientsCount         = 0
		servicesCount        = 0
		eventsCount          = 0
	)
	ws = &WaitSet{
		context:        c,
		Subscriptions:  []*Subscription{},
		Timers:         []*Timer{},
		Services:       []*Service{},
		Clients:        []*Client{},
		rcl_wait_set_t: C.rcl_get_zero_initialized_wait_set(),
	}
	defer onErr(&err, ws.Close)
	var rc C.rcl_ret_t = C.rcl_wait_set_init(
		&ws.rcl_wait_set_t,
		subscriptionsCount,
		guardConditionsCount,
		timersCount,
		clientsCount,
		servicesCount,
		eventsCount,
		c.rcl_context_t,
		*c.rcl_allocator_t,
	)
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}
	ws.cancelWait, err = c.newGuardCondition()
	if err != nil {
		return nil, err
	}
	ws.addGuardConditions(ws.cancelWait)
	if !c.addResource(ws) {
		return nil, closedErr("context")
	}
	return ws, nil
}

// Context returns the context s belongs to.
func (w *WaitSet) Context() *Context {
	return w.context
}

func (w *WaitSet) AddSubscriptions(subs ...*Subscription) {
	w.Subscriptions = append(w.Subscriptions, subs...)
}

func (w *WaitSet) AddTimers(timers ...*Timer) {
	w.Timers = append(w.Timers, timers...)
}

func (w *WaitSet) AddServices(services ...*Service) {
	w.Services = append(w.Services, services...)
}

func (w *WaitSet) AddClients(clients ...*Client) {
	w.Clients = append(w.Clients, clients...)
}

func (w *WaitSet) AddActionServers(servers ...*ActionServer) {
	w.ActionServers = append(w.ActionServers, servers...)
}

func (w *WaitSet) AddActionClients(clients ...*ActionClient) {
	w.ActionClients = append(w.ActionClients, clients...)
}

func (w *WaitSet) addGuardConditions(guardConditions ...*guardCondition) {
	w.guardConditions = append(w.guardConditions, guardConditions...)
}

// followResources makes w wait for the resources in store and rebuild its
// entity lists whenever a resource is added to or removed from store or the
// stores of its nodes.
func (w *WaitSet) followResources(store *rosResourceStore) (err error) {
	w.refresh, err = w.context.newGuardCondition()
	if err != nil {
		return err
	}
	w.addGuardConditions(w.refresh)
	w.source = store
	// Register before the first sync so that a resource added in between
	// still triggers a refresh.
	w.context.addSpinner(w)
	w.syncResources()
	return nil
}

// syncResources replaces the entity lists of w with the resources currently in
// w.source.
func (w *WaitSet) syncResources() {
	w.Subscriptions = w.Subscriptions[:0]
	w.Timers = w.Timers[:0]
	w.Services = w.Services[:0]
	w.Clients = w.Clients[:0]
	w.ActionServers = w.ActionServers[:0]
	w.ActionClients = w.ActionClients[:0]
	w.addResources(w.source)
}

func (w *WaitSet) addResources(res *rosResourceStore) {
	res.mutex.Lock()
	defer res.mutex.Unlock()
	for _, res := range res.resources {
		switch res := res.(type) {
		case *Subscription:
			// A subscription without a callback is read by its owner with
			// TakeMessage. Waiting on it would make Spin consume or spin
			// on messages the owner is polling for.
			if res.Callback != nil {
				w.AddSubscriptions(res)
			}
		case *Timer:
			if res.Callback != nil {
				w.AddTimers(res)
			}
		case *Service:
			w.AddServices(res)
		case *Client:
			w.AddClients(res)
		case *ActionServer:
			w.AddActionServers(res)
		case *ActionClient:
			w.AddActionClients(res)
		case *guardCondition: // Guard conditions are handled specially
		case *Node:
			w.addResources(&res.rosResourceStore)
		}
	}
}

/*
Run causes the current goroutine to block on this given WaitSet.
WaitSet executes the given timers and subscriptions and calls their callbacks on new events.

Entities closed while Run is running are dropped, and their rcl handles are
finalized once Run no longer uses them. Run returns an error when the wait set
or its context is closed.
*/
func (w *WaitSet) Run(ctx context.Context) (err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if ctx == nil {
		return errors.New("context must not be nil")
	}
	if !w.life.acquire() {
		return errWaitSetClosed
	}
	defer w.life.release()
	// After releasing its entities, wake the other spinning wait sets so
	// they pick up the entities this one serviced.
	defer w.context.resourcesChanged()
	release := w.acquireEntities()
	defer func() { release() }()
	if !w.hasGuardCondition(w.cancelWait) {
		return errWaitSetClosed
	}
	errs := make(chan error, 1)
	defer func() {
		err = errors.Join(err, <-errs)
	}()
	errctx, cancel := context.WithCancel(ctx)
	defer cancel()
	w.stopMu.Lock()
	w.stopRun = cancel
	w.stopMu.Unlock()
	defer func() {
		w.stopMu.Lock()
		w.stopRun = nil
		w.stopMu.Unlock()
	}()
	if w.life.isClosed() {
		// Closed between acquiring and publishing stopRun.
		cancel()
	}
	// runDone closes when the wait loop has returned (deferred calls run in
	// reverse order: before cancel and before errs is read).
	runDone := make(chan struct{})
	defer close(runDone)
	go func() {
		defer close(errs)
		<-errctx.Done()
		// rcl_wait waits without a timeout, so the trigger is the only way to
		// wake it: retry a failed trigger until the loop has returned.
		for logged := false; ; logged = true {
			err := w.cancelWait.Trigger()
			// ErrClosed: the guard condition is finalized, which happens only
			// after the wait loop has let go of it, so nothing waits on it.
			if err == nil || errors.Is(err, ErrClosed) {
				return
			}
			if !logged {
				GetLogger("rclgo").Error("waking the wait set failed, retrying: ", err) //nolint:errcheck
			}
			select {
			case <-runDone:
				errs <- err
				return
			case <-time.After(10 * time.Millisecond):
			}
		}
	}()
	refresh := false
	for {
		if refresh {
			// Acquire the new set before releasing the old one, so that
			// entities this wait set services stay owned by it.
			previous := release
			w.syncResources()
			release = w.acquireEntities()
			previous()
			refresh = false
		}
		// Closing the wait set (or its context) also closes its own
		// guard conditions; a refresh then drops cancelWait, and its
		// trigger would go unseen. Check before every wait.
		if w.life.isClosed() || !w.hasGuardCondition(w.cancelWait) {
			if err := ctx.Err(); err != nil {
				return err
			}
			return errWaitSetClosed
		}
		if err := w.initEntities(); err != nil {
			return err
		}
		if rc := C.rcl_wait(&w.rcl_wait_set_t, -1); rc != C.RCL_RET_OK {
			return errorsCast(rc)
		}
		guardConditions := unsafe.Slice(w.rcl_wait_set_t.guard_conditions, len(w.guardConditions))
		for i := range w.guardConditions {
			switch {
			case guardConditions[i] == nil:
			case guardConditions[i] == w.cancelWait.rclGuardCondition:
				if errctx.Err() == nil {
					// Left triggered by an earlier Run that ended for
					// another reason; this Run was not asked to stop.
					break
				}
				if err := ctx.Err(); err != nil {
					return err
				}
				return errWaitSetClosed
			case w.refresh != nil && guardConditions[i] == w.refresh.rclGuardCondition:
				refresh = true
			}
		}
		timers := unsafe.Slice(w.rcl_wait_set_t.timers, len(w.Timers))
		// An entity may be closed by a callback that ran earlier in
		// this loop; its handle stays valid until the next refresh, but
		// it must not be serviced any more.
		for i, t := range w.Timers {
			if timers[i] != nil && !t.life.isClosed() {
				if err := t.call(); err != nil {
					// Canceled after rcl_wait returned: nothing to do.
					if !errors.Is(err, errTimerCanceled) {
						GetLogger("rclgo").Error("timer not called: ", err) //nolint:errcheck
					}
					continue
				}
				if t.Callback != nil {
					t.Callback(t)
				}
			}
		}
		subs := unsafe.Slice(w.rcl_wait_set_t.subscriptions, len(w.Subscriptions))
		for i, s := range w.Subscriptions {
			if subs[i] != nil && s.Callback != nil && !s.life.isClosed() {
				s.Callback(s)
			}
		}
		svcs := unsafe.Slice(w.rcl_wait_set_t.services, len(w.Services))
		for i, s := range w.Services {
			if svcs[i] != nil && !s.life.isClosed() {
				s.handleRequest()
			}
		}
		clients := unsafe.Slice(w.rcl_wait_set_t.clients, len(w.Clients))
		for i, c := range w.Clients {
			if clients[i] != nil && !c.life.isClosed() {
				c.sender.HandleResponse()
			}
		}
		for _, s := range w.ActionServers {
			if !s.life.isClosed() {
				s.handleReadyEntities(w)
			}
		}
		for _, c := range w.ActionClients {
			if !c.life.isClosed() {
				c.handleReadyEntities(w)
			}
		}
	}
}

// acquireEntities registers w as a user of its entities, so that closing them
// defers their finalization, and drops the entities that are already closed
// from its lists. The returned function releases them.
//
// An entity that another running wait set already services is skipped, so
// that it is serviced once; the first wait set keeps it until it stops, and
// the skip is logged once per entity.
func (w *WaitSet) acquireEntities() (release func()) {
	var held []*lifetime
	// Only entities skipped in this pass are remembered, so the map doesn't
	// grow as entities come and go.
	skipped := map[*lifetime]bool{}
	defer func() { w.skipped = skipped }()
	keep := func(l *lifetime, describe func() string) bool {
		ok, otherOwner := l.acquireFor(w)
		if ok {
			held = append(held, l)
			return true
		}
		if otherOwner {
			if !w.skipped[l] {
				GetLogger("rclgo").Warn(describe() + //nolint:errcheck
					" is already serviced by another running Spin or wait set; only that one services it")
			}
			skipped[l] = true
		}
		return false
	}
	w.Subscriptions = filterEntities(w.Subscriptions, func(s *Subscription) bool {
		return keep(&s.life, func() string { return "subscription to " + s.TopicName })
	})
	w.Timers = filterEntities(w.Timers, func(t *Timer) bool { return keep(&t.life, func() string { return "a timer" }) })
	w.Services = filterEntities(w.Services, func(s *Service) bool { return keep(&s.life, func() string { return "a service" }) })
	w.Clients = filterEntities(w.Clients, func(c *Client) bool { return keep(&c.life, func() string { return "a client" }) })
	w.ActionClients = filterEntities(w.ActionClients, func(c *ActionClient) bool {
		return keep(&c.life, func() string { return "an action client" })
	})
	w.ActionServers = filterEntities(w.ActionServers, func(s *ActionServer) bool {
		return keep(&s.life, func() string { return "an action server" })
	})
	w.guardConditions = filterEntities(w.guardConditions, func(g *guardCondition) bool {
		return keep(&g.life, func() string { return "a guard condition" })
	})
	return func() {
		for _, l := range held {
			l.releaseFor(w)
		}
	}
}

func filterEntities[T any](entities []T, keep func(T) bool) []T {
	kept := entities[:0]
	for _, e := range entities {
		if keep(e) {
			kept = append(kept, e)
		}
	}
	return kept
}

func (w *WaitSet) hasGuardCondition(g *guardCondition) bool {
	for _, gc := range w.guardConditions {
		if gc == g {
			return true
		}
	}
	return false
}

// stop makes Run return if it is running.
func (w *WaitSet) stop() {
	w.stopMu.Lock()
	defer w.stopMu.Unlock()
	if w.stopRun != nil {
		w.stopRun()
	}
}

func (w *WaitSet) initEntities() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	if !C.rcl_wait_set_is_valid(&w.rcl_wait_set_t) {
		return errorsCastC(C.RCL_RET_WAIT_SET_INVALID, "rcl_wait_set_is_valid() failed")
	}
	var rc C.rcl_ret_t = C.rcl_wait_set_clear(&w.rcl_wait_set_t)
	if rc != C.RCL_RET_OK {
		return errorsCastC(rc, "rcl_wait_set_clear() failed")
	}
	rc = C.rcl_wait_set_resize(
		&w.rcl_wait_set_t,
		C.size_t(len(w.Subscriptions)+2*len(w.ActionClients)),
		C.size_t(len(w.guardConditions)),
		C.size_t(len(w.Timers)+len(w.ActionServers)),
		C.size_t(len(w.Clients)+3*len(w.ActionClients)),
		C.size_t(len(w.Services)+3*len(w.ActionServers)),
		w.rcl_wait_set_t.size_of_events,
	)
	if rc != C.RCL_RET_OK {
		return errorsCastC(rc, "rcl_wait_set_resize() failed")
	}
	for _, sub := range w.Subscriptions {
		rc = C.rcl_wait_set_add_subscription(&w.rcl_wait_set_t, sub.rcl_subscription_t, nil)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "rcl_wait_set_add_subscription() failed")
		}
	}
	for _, timer := range w.Timers {
		rc = C.rcl_wait_set_add_timer(&w.rcl_wait_set_t, timer.rcl_timer_t, nil)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "rcl_wait_set_add_timer() failed")
		}
	}
	for _, service := range w.Services {
		rc = C.rcl_wait_set_add_service(&w.rcl_wait_set_t, service.rclService, nil)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "rcl_wait_set_add_service() failed")
		}
	}
	for _, client := range w.Clients {
		rc = C.rcl_wait_set_add_client(&w.rcl_wait_set_t, client.rclClient, nil)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "rcl_wait_set_add_client() failed")
		}
	}
	for _, guardCondition := range w.guardConditions {
		rc = C.rcl_wait_set_add_guard_condition(&w.rcl_wait_set_t, guardCondition.rclGuardCondition, nil)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "rcl_wait_set_add_guard_condition() failed")
		}
	}
	for _, server := range w.ActionServers {
		rc = C.rcl_action_wait_set_add_action_server(&w.rcl_wait_set_t, &server.rclServer, nil)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "rcl_wait_set_add_action_server() failed")
		}
	}
	for _, client := range w.ActionClients {
		rc = C.rcl_action_wait_set_add_action_client(&w.rcl_wait_set_t, &client.rclClient, nil, nil)
		if rc != C.RCL_RET_OK {
			return errorsCastC(rc, "rcl_wait_set_add_action_client() failed")
		}
	}
	return nil
}

// Close stops Run if it is running and finalizes the wait set once Run has
// returned.
func (w *WaitSet) Close() error {
	c := w.context
	return closeEntity(&w.life, "wait set", func() {
		w.stop()
		c.removeSpinner(w)
		c.removeResource(w)
	}, &c.life, w.fini)
}

func (w *WaitSet) fini() (err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	rc := C.rcl_wait_set_fini(&w.rcl_wait_set_t)
	if rc != C.RCL_RET_OK {
		err = errors.Join(err, errorsCast(rc))
	}
	var closeError closeError
	for _, g := range []*guardCondition{w.cancelWait, w.refresh} {
		if g == nil {
			continue
		}
		if gerr := g.Close(); gerr != nil && !errors.As(gerr, &closeError) {
			err = errors.Join(err, gerr)
		}
	}
	return err
}
