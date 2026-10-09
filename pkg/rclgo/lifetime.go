package rclgo

import (
	"fmt"
	"sync"
)

// lifetime keeps the rcl handle of an entity alive while wait sets use it.
//
// Closing an entity marks it closed at once. Its handle is finalized right away
// if nothing uses it, or otherwise when the last user releases it. This lets an
// entity be closed while Spin is waiting on it, from any goroutine including
// the entity's own callback, without freeing memory Spin still uses.
//
// The zero value is ready for use.
type lifetime struct {
	mu      sync.Mutex
	users   int
	closed  bool
	pending func()

	// owner is the wait set that services the entity while it runs. Another
	// running wait set skips the entity, so each message or request is
	// handled once. ownerHolds counts owner's holds; ownership ends with the
	// last one.
	owner      *WaitSet
	ownerHolds int
}

// acquire registers a user of the handle. It fails once the entity is closed.
func (l *lifetime) acquire() bool {
	l.mu.Lock()
	defer l.mu.Unlock()
	if l.closed {
		return false
	}
	l.users++
	return true
}

// acquireFor registers w as a user that services the entity. It fails if the
// entity is closed or another wait set services it (otherOwner reports the
// latter).
func (l *lifetime) acquireFor(w *WaitSet) (ok, otherOwner bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	if l.closed {
		return false, false
	}
	if l.owner != nil && l.owner != w {
		return false, true
	}
	l.owner = w
	l.ownerHolds++
	l.users++
	return true, false
}

// releaseFor undoes acquireFor.
func (l *lifetime) releaseFor(w *WaitSet) {
	l.mu.Lock()
	if l.owner == w {
		l.ownerHolds--
		if l.ownerHolds == 0 {
			l.owner = nil
		}
	}
	l.releaseLocked()
}

// hold registers a user even if the entity is closed. A child whose
// finalization is deferred holds its parent so that the parent is finalized
// after it.
func (l *lifetime) hold() {
	l.mu.Lock()
	l.users++
	l.mu.Unlock()
}

// release unregisters a user. The last user runs the deferred finalizer, if
// any.
func (l *lifetime) release() {
	l.mu.Lock()
	l.releaseLocked()
}

// releaseLocked is release with l.mu held; it unlocks l.mu.
func (l *lifetime) releaseLocked() {
	l.users--
	var run func()
	if l.users == 0 {
		run, l.pending = l.pending, nil
	}
	l.mu.Unlock()
	if run != nil {
		run()
	}
}

func (l *lifetime) isClosed() bool {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.closed
}

// markClosed marks the entity closed. It returns false if it already was.
func (l *lifetime) markClosed() bool {
	l.mu.Lock()
	defer l.mu.Unlock()
	if l.closed {
		return false
	}
	l.closed = true
	return true
}

// finalize runs fini and returns its error if the handle has no users.
// Otherwise fini runs when the last user releases the handle; parent, if not
// nil, is held until then, and the error of fini is logged as being about
// kind.
func (l *lifetime) finalize(parent *lifetime, kind string, fini func() error) error {
	l.mu.Lock()
	if l.users == 0 {
		l.mu.Unlock()
		return fini()
	}
	if parent != nil {
		parent.hold()
	}
	l.pending = func() {
		if err := fini(); err != nil {
			GetLogger("rclgo").Error(fmt.Sprintf("deferred close of %s failed: %v", kind, err)) //nolint:errcheck
		}
		if parent != nil {
			parent.release()
		}
	}
	l.mu.Unlock()
	return nil
}

// closeEntity closes an entity: it marks it closed, calls remove (which takes
// the entity out of its resource store, so spinning wait sets drop it) and then
// finalizes it with fini, deferred while wait sets still use it.
func closeEntity(l *lifetime, kind string, remove func(), parent *lifetime, fini func() error) error {
	if !l.markClosed() {
		return closeErr(kind)
	}
	if remove != nil {
		remove()
	}
	return l.finalize(parent, kind, fini)
}
