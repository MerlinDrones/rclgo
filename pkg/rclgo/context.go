/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo

/*
#include <rcl/context.h>
#include <rcl/init_options.h>
#include <rcl/init.h>
#include <rcl/time.h>
#include <rcutils/allocator.h>
*/
import "C"

import (
	"context"
	"errors"
	"fmt"
	"io"
	"math"
	"runtime"
	"sort"
	"sync"
	"sync/atomic"
	"unsafe"
)

// Setting this as the domain ID causes ROS to use the value of ROS_DOMAIN_ID
// environment variable as the actual domain ID or zero if ROS_DOMAIN_ID not
// defined.
const DefaultDomainID = math.MaxUint

type rosID uint64

func (r *rosID) getID() uint64 {
	return uint64(*r)
}

func (r *rosID) setID(x uint64) {
	*r = rosID(x)
}

type rosResource interface {
	io.Closer
	getID() uint64
	setID(x uint64)
}

// rosResourceStore manages ROS resources. When Close is called, all resources in
// the store are Closed. The zero value is ready for use.
type rosResourceStore struct {
	mutex     sync.Mutex
	resources map[uint64]rosResource
	idCounter uint64

	// onChange, if set, is called after a resource that Spin services is
	// added to or removed from the store. It is called without the store's
	// mutex held.
	onChange func()

	// closed is set when closeAll starts. Resources created afterwards are
	// refused, so that none outlives the node or context being closed.
	closed bool
}

// addResource adds r to the store. It returns false, without adding r, once
// the store is being closed.
func (s *rosResourceStore) addResource(r rosResource) bool {
	s.mutex.Lock()
	if s.closed {
		s.mutex.Unlock()
		return false
	}
	if s.resources == nil {
		s.resources = make(map[uint64]rosResource)
		// The counter starts at one to allow removing zero-initialized
		// resources.
		s.idCounter = 1
	}
	r.setID(s.idCounter)
	s.resources[s.idCounter] = r
	s.idCounter++
	s.mutex.Unlock()
	s.changed(r)
	return true
}

func (s *rosResourceStore) removeResource(r rosResource) {
	s.mutex.Lock()
	delete(s.resources, r.getID())
	s.mutex.Unlock()
	s.changed(r)
}

func (s *rosResourceStore) changed(r rosResource) {
	if s.onChange == nil {
		return
	}
	switch r.(type) {
	case *Subscription, *Timer, *Service, *Client, *ActionServer, *ActionClient, *Node:
		s.onChange()
	}
}

// closeAll closes the resources in the store in dependency order (see
// closeOrder), skipping those for which skip returns true. Resources that are
// already closed are ignored.
func (s *rosResourceStore) closeAll(skip func(rosResource) bool) (err error) {
	s.mutex.Lock()
	s.closed = true
	resources := make([]rosResource, 0, len(s.resources))
	for _, r := range s.resources {
		if skip == nil || !skip(r) {
			resources = append(resources, r)
		}
	}
	s.mutex.Unlock()
	var closeError closeError
	for _, r := range closeOrder(resources) {
		if cerr := r.Close(); cerr != nil && !errors.As(cerr, &closeError) {
			err = errors.Join(err, cerr)
		}
	}
	return err
}

// closeOrder sorts resources so that every resource comes before the
// resources it uses: action servers and clients, then the other node
// entities, nodes, timers, wait sets, guard conditions and finally clocks
// (timers and action servers use a clock). Resources of the same kind are
// closed in creation order.
func closeOrder(resources []rosResource) []rosResource {
	rank := func(r rosResource) int {
		switch r.(type) {
		case *ActionServer, *ActionClient:
			return 0
		case *Subscription, *Service, *Client, *Publisher:
			return 1
		case *Node:
			return 2
		case *Timer:
			return 3
		case *WaitSet:
			return 5
		case *guardCondition:
			return 6
		case *Clock:
			return 7
		default:
			return 4
		}
	}
	sort.SliceStable(resources, func(i, j int) bool {
		ri, rj := rank(resources[i]), rank(resources[j])
		if ri != rj {
			return ri < rj
		}
		return resources[i].getID() < resources[j].getID()
	})
	return resources
}

var (
	// defaultContextMu guards defaultContext, which Init sets and Uninit
	// clears.
	defaultContextMu sync.Mutex
	defaultContext   *Context
	errInitNotCalled = errors.New("Init has not been called")
)

// DefaultContext returns the global default context or nil if Init has not yet
// been called.
func DefaultContext() *Context {
	defaultContextMu.Lock()
	defer defaultContextMu.Unlock()
	return defaultContext
}

// Init is like InitWithOpts except that it always uses default options.
func Init(args *Args) (err error) {
	return InitWithOpts(args, nil)
}

// InitWithOpts initializes the global default context and logging system if
// they have not been initialized yet. Calling InitWithOpts multiple times after
// a successful (returning nil) call is a no-op.
//
// A nil args is treated as an empty argument list.
//
// If opts is nil, default options are used.
func InitWithOpts(args *Args, opts *ContextOptions) (err error) {
	defaultContextMu.Lock()
	defer defaultContextMu.Unlock()
	if defaultContext == nil {
		defaultContext, err = NewContextWithOpts(args, opts)
	}
	return
}

// Uninit uninitializes the default context if it has been initialized. Calling
// Uninit multiple times without calling Init in between the calls is a no-op.
// Uninit should be called before program termination if Init has been called
// successfully.
func Uninit() (err error) {
	defaultContextMu.Lock()
	c := defaultContext
	defaultContext = nil
	defaultContextMu.Unlock()
	if c != nil {
		err = c.Close()
	}
	// Finalize logging system to flush all buffered logs
	if loggingErr := FiniLogging(); loggingErr != nil {
		err = errors.Join(err, loggingErr)
	}
	return
}

// Spin starts and waits for all ROS resources in DefaultContext() that need
// waiting such as nodes and subscriptions. Spin returns when an error occurs or
// ctx is canceled.
func Spin(ctx context.Context) error {
	c := DefaultContext()
	if c == nil {
		return errInitNotCalled
	}
	return c.Spin(ctx)
}

// ContextOptions can be used to configure a Context.
type ContextOptions struct {
	// The type of the default clock created for the Context.
	ClockType ClockType

	// The DDS domain ID of the Context. Should be in range [0, 101] or
	// DefaultDomainID.
	DomainID uint
}

// NewDefaultContextOptions returns the default options for a Context.
func NewDefaultContextOptions() *ContextOptions {
	return &ContextOptions{
		ClockType: ClockTypeROSTime,
		DomainID:  DefaultDomainID,
	}
}

// Context manages resources for a set of RCL entities.
type Context struct {
	rcl_allocator_t *C.rcutils_allocator_t
	rcl_context_t   *C.rcl_context_t
	defaultClock    *Clock
	clockMu         sync.RWMutex // guards clock
	clock           *Clock

	rosResourceStore
	life lifetime

	// spinners are the wait sets that follow a resource store (see
	// WaitSet.followResources). They are told to rebuild their entity lists
	// whenever a resource is added to or removed from the context or one of
	// its nodes.
	spinnersMu sync.Mutex
	spinners   map[*WaitSet]struct{}

	// leakedNodes counts nodes that could not be finalized. rcl and the
	// middleware still use the context for them, so it is not finalized
	// either; its memory is leaked instead of being freed while in use.
	leakedNodes atomic.Int32
}

// NewContext calls NewContextWithOpts with default options except for
// ClockType, which is set to the value passed to this function.
//
// If clockType == 0, ClockTypeROSTime is used.
func NewContext(clockType ClockType, rclArgs *Args) (*Context, error) {
	opts := NewDefaultContextOptions()
	if clockType == 0 {
		opts.ClockType = ClockTypeROSTime
	} else {
		opts.ClockType = clockType
	}
	return NewContextWithOpts(rclArgs, opts)
}

/*
NewContextWithOpts initializes a new RCL context.

A nil rclArgs is treated as en empty argument list.

If logging has not yet been initialized, NewContextWithOpts will initialize it
automatically using rclArgs for logging configuration.

If opts is nil, default options are used.
*/
func NewContextWithOpts(rclArgs *Args, opts *ContextOptions) (ctx *Context, err error) {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	ctx = &Context{}
	ctx.rosResourceStore.onChange = ctx.resourcesChanged
	defer onErr(&err, ctx.Close)

	if rclArgs == nil {
		rclArgs, _, err = ParseArgs(nil)
		if err != nil {
			return nil, err
		}
	}
	if opts == nil {
		opts = NewDefaultContextOptions()
	}

	ctx.rcl_allocator_t = (*C.rcl_allocator_t)(C.malloc(C.sizeof_rcl_allocator_t))
	*ctx.rcl_allocator_t = C.rcl_get_default_allocator()
	ctx.rcl_context_t = (*C.rcl_context_t)(C.malloc(C.sizeof_rcl_context_t))
	*ctx.rcl_context_t = C.rcl_get_zero_initialized_context()

	if err := rclInitLogging(rclArgs, false); err != nil {
		return nil, err
	}

	rcl_init_options_t := C.rcl_get_zero_initialized_init_options()
	rc := C.rcl_init_options_init(&rcl_init_options_t, *ctx.rcl_allocator_t)
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}
	// rcl_init copies the options into the context.
	defer func() {
		if rc := C.rcl_init_options_fini(&rcl_init_options_t); rc != C.RCL_RET_OK {
			// Only memory is lost; the context is usable.
			GetLogger("rclgo").Error(errorsCastC(rc, "failed to finalize init options")) //nolint:errcheck
		}
	}()
	rc = C.rcl_init_options_set_domain_id(&rcl_init_options_t, C.size_t(opts.DomainID))
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}

	rc = C.rcl_init(rclArgs.argc(), rclArgs.argv(), &rcl_init_options_t, ctx.rcl_context_t)
	runtime.KeepAlive(rclArgs)
	if rc != C.RCL_RET_OK {
		return nil, errorsCast(rc)
	}

	ctx.defaultClock, err = ctx.NewClock(opts.ClockType)
	if err != nil {
		return nil, err
	}
	ctx.clock = ctx.defaultClock

	return ctx, nil
}

// Close stops every wait set running on the context, closes all resources of
// the context and finalizes it. Resources that a running wait set still uses
// are finalized when it lets go of them, and the context after them.
func (c *Context) Close() error {
	if !c.life.markClosed() {
		return closeErr("context")
	}
	c.stopWaitSets()
	isClock := func(r rosResource) bool { _, ok := r.(*Clock); return ok }
	err := c.rosResourceStore.closeAll(isClock)
	return errors.Join(err, c.life.finalize(nil, "context", c.fini))
}

// stopWaitSets makes every running wait set of the context return.
func (c *Context) stopWaitSets() {
	c.mutex.Lock()
	var waitSets []*WaitSet
	for _, r := range c.resources {
		if ws, ok := r.(*WaitSet); ok {
			waitSets = append(waitSets, ws)
		}
	}
	c.mutex.Unlock()
	for _, ws := range waitSets {
		ws.stop()
	}
}

// fini finalizes the clocks and the rcl context. It runs after every other
// resource of the context has been finalized.
func (c *Context) fini() error {
	runtime.LockOSThread() // rcl error state is per OS thread; see errorString
	defer runtime.UnlockOSThread()
	errs := c.rosResourceStore.closeAll(nil)
	if n := c.leakedNodes.Load(); n > 0 && c.rcl_context_t != nil {
		// rcl_context_fini would free the middleware context under the
		// nodes' feet (rcl frees it even when the middleware refuses).
		c.rcl_context_t = nil
		c.rcl_allocator_t = nil
		return errors.Join(errs, fmt.Errorf("context not finalized because %d of its nodes could not be; its memory is leaked", n))
	}
	if c.rcl_context_t != nil {
		if rc := C.rcl_shutdown(c.rcl_context_t); rc != C.RCL_RET_OK {
			errs = errors.Join(errs, errorsCastC(rc, fmt.Sprintf("C.rcl_shutdown(%+v)", c.rcl_context_t)))
		} else if rc := C.rcl_context_fini(c.rcl_context_t); rc != C.RCL_RET_OK {
			errs = errors.Join(errs, errorsCastC(rc, "rcl_context_fini failed"))
		}
		C.free(unsafe.Pointer(c.rcl_context_t))
		c.rcl_context_t = nil
	}
	if c.rcl_allocator_t != nil {
		C.free(unsafe.Pointer(c.rcl_allocator_t))
		c.rcl_allocator_t = nil
	}
	return errs
}

// Clock returns the clock that timers and action servers created on c use.
func (c *Context) Clock() *Clock {
	c.clockMu.RLock()
	defer c.clockMu.RUnlock()
	return c.clock
}

// SetClock sets the clock for timers and action servers created on c from now
// on; nil restores the context's default clock. Existing ones keep theirs.
func (c *Context) SetClock(newClock *Clock) {
	c.clockMu.Lock()
	defer c.clockMu.Unlock()
	if newClock == nil {
		c.clock = c.defaultClock
	} else {
		c.clock = newClock
	}
}

// Spin starts and waits for all ROS resources in the context that need waiting
// such as nodes and subscriptions. Resources created while Spin is running are
// picked up as well. Spin returns when an error occurs or ctx is canceled.
func (c *Context) Spin(ctx context.Context) error {
	return spinErr("context", c.spin(ctx, &c.rosResourceStore))
}

// spin waits for and handles the resources in store, following changes to it,
// until an error occurs or ctx is canceled.
func (c *Context) spin(ctx context.Context, store *rosResourceStore) (err error) {
	ws, err := c.NewWaitSet()
	if err != nil {
		return err
	}
	defer func() {
		var closed closeError
		if cerr := ws.Close(); cerr != nil && !errors.As(cerr, &closed) {
			err = errors.Join(err, cerr)
		}
	}()
	if err := ws.followResources(store); err != nil {
		return err
	}
	return ws.Run(ctx)
}

// resourcesChanged tells every spinning wait set to rebuild its entity lists
// and recompute its timeout.
func (c *Context) resourcesChanged() {
	c.spinnersMu.Lock()
	defer c.spinnersMu.Unlock()
	for ws := range c.spinners {
		// A failed trigger only delays the refresh until the wait set
		// wakes for another reason; a finalized one belongs to a wait set
		// that no longer runs.
		if err := ws.refresh.Trigger(); err != nil && !errors.Is(err, ErrClosed) {
			GetLogger("rclgo").Error("waking the wait set failed: ", err) //nolint:errcheck
		}
	}
}

func (c *Context) addSpinner(ws *WaitSet) {
	c.spinnersMu.Lock()
	defer c.spinnersMu.Unlock()
	if c.spinners == nil {
		c.spinners = make(map[*WaitSet]struct{})
	}
	c.spinners[ws] = struct{}{}
}

func (c *Context) removeSpinner(ws *WaitSet) {
	c.spinnersMu.Lock()
	defer c.spinnersMu.Unlock()
	delete(c.spinners, ws)
}
