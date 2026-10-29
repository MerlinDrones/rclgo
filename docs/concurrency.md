# Concurrency between Go and rcl

rclgo calls rcl, the C library underneath rclcpp and rclpy, directly through cgo. This page explains why concurrency
was the largest source of defects in rclgo up to `jazzy` before #55–#61 (v0.6.0 and the commits after it), and how
#56, #57 and #58 fixed each one. "Before" below means that code; "after" means the code with all of #55–#61.

## Why it was a big problem

rcl was written for callers that manage threads carefully:

- one executor thread per node;
- many calls documented as **not thread-safe** for the same node (creating and finalizing entities);
- the last error message is stored **per OS thread**;
- logging state (logger levels, the output handler) is **process-global**, and rclcpp protects it with a global
  logging mutex.

rclgo gave Go code direct access to rcl but added almost no protection of its own. In Go that is a bad combination:

- Go programs call into rclgo from many goroutines at once. It is normal to close a subscription while `Spin` is
  still waiting on it, or from a goroutine other than the one using it.
- The Go scheduler can move a goroutine to a different OS thread between two cgo calls.
- Go's memory safety and race detector don't cover C memory. A race inside rcl doesn't produce a Go error. It
  crashes the whole process with SIGSEGV, corrupts rcl's internal state, or quietly gives a wrong result.
- These failures depend on timing. They seldom show up on an idle machine and appear under load. Concurrent node
  creation and destruction, for example, crashed 0 of 20 runs unpinned on a quiet host and 14 of 30 when pinned to
  two CPUs (measured on a development build without the fix in 3).

A common pattern triggers several of these at once: for each request, create a subscription on a long-lived node,
poll it with `TakeMessage` from a separate goroutine, and close it when the request ends, while `Context.Spin` runs
on the same node.

## Summary

| # | Problem | Issue | Fixed in |
|---|---|---|---|
| 1 | Close freed memory other goroutines were still using | #12, #24 | #56 (follow-ups in #57, #58) |
| 2 | Entity init/fini on one node ran concurrently | #50 | #57 |
| 3 | Process-global logging and node state were unprotected | #36, #53 | #57, #58 |
| 4 | rcl error text could come from another goroutine | | #57 |
| 5 | Spin ignored entities created after it started | #3 | #56 |
| 6 | Timers and action servers raced on a clock's jump callbacks | | #56 |
| 7 | A node that failed to finalize was freed while rcl and DDS still used it | #26 | #57 |

## 1. Close vs concurrent use: use-after-free (#12, #24)

**Before:**
- `Subscription.Close` called `rcl_subscription_fini` and freed the handle straight away.
- `Subscription.TakeMessage` passed the handle to `rcl_take` without any check.
- So if one goroutine was inside `rcl_take` while another called Close, `rcl_take` read freed memory. A running
  wait set had the same problem with every entity it waited on. Publishers, services, clients and timers worked the
  same way.
- `Context.Close` finalized resources in Go map order, so it could free a clock while a timer was still reading it.

**After:** each entity has a reference-counted `lifetime` (`pkg/rclgo/lifetime.go`), on the same "free later" model
rclpy uses.
- Every call takes a reference first (`acquire`) and releases it when done; `TakeMessage` is an example.
- Close only marks the entity closed. The rcl handle is finalized right away if nothing uses it, or when the last
  user releases it (`lifetime.finalize`). So Close is safe from any goroutine, including the entity's own callback.
- A call on a closed entity returns an error wrapping `rclgo.ErrClosed` instead of crashing.
- An entity whose finalization is delayed holds its parent, so a node is never freed before its children.
- `Context.Close` stops every running wait set first, then finalizes from a locked snapshot in dependency order:
  action servers and clients, other node entities, nodes, timers, wait sets, guard conditions, and clocks last
  together with the rcl context.
- A node that can't be finalized is deliberately leaked rather than freed (see 7).

Commits: "fix: make closing entities and contexts safe while Spin is running", "fix: make user calls safe against a
concurrent Close" (#56); "fix: clocks outlive the timers and action servers that use them; no cancel wait in
WatchGoal" (#57).

## 2. Creating and finalizing entities on the same node (#50)

**Before:** rcl documents `rcl_publisher_init/fini`, `rcl_subscription_init/fini`, `rcl_service_init/fini` and the
like as not thread-safe for the same node, because they share the node's type-description cache. rclgo didn't lock
around them. Result: "Failed to unregister type, hash not present in map" (`node_type_cache.c`) or SIGSEGV.

**After:** a per-node mutex, `Node.entityMu`, is held around every rcl call that creates or finalizes an entity on
that node.

Commit: "fix: serialize creating and finalizing entities on the same node" (#57). Test:
`TestEntityCreationRacingClose`, measured on a development build without the fix: failed 10 of 10 runs on Cyclone DDS
(some with SIGSEGV) and 3–6 of 10 on Fast DDS; it passes with the fix, including with `-race` on 2 CPUs.

## 3. Process-global logging and node state (#36, #53)

**Before:** `rcl_node_init`/`rcl_node_fini`, the rosout publisher's init/fini and `rcutils_log` change
process-global tables. `rcutils_log`, for example, caches each logger's effective level in a shared table the first
time that logger logs. rclgo had only a Go `sync.Mutex` for its own variables and never held it around these C calls.
Measured on development builds without the respective fix:
- Nodes created and closed concurrently crashed 14 of 30 `-race` runs pinned to two CPUs (SIGSEGV in
  `rcl_node_fini`; `TestNodesCreatedAndClosedConcurrently`).
- Logging from many goroutines while levels changed failed 9 of 10 runs (rarely a SIGSEGV in `rcutils_log`, more
  often a silently dropped message; `TestLoggingWhileLevelsChange`).

**After:** one **recursive** C pthread mutex, `rclgo_logging_mutex` in `pkg/rclgo/logging.go`, the equivalent of
rclcpp's global logging mutex, is held around:
- `rcl_node_init`/`rcl_node_fini` and the rosout publisher's init/fini;
- the whole `rcutils_log` call (`rcutils_log_wrapper`) and the level queries (`Level`, `EffectiveLevel`,
  `IsEnabledFor`).

It is recursive because rcl logs while creating a node, and that log call takes the same lock again on the same
thread. Both tests above then passed 30 of 30 runs.

A custom output handler that panics would unwind through the C frames and leave this mutex locked, hanging every later
log call. The Go handler wrapper therefore recovers the panic, prints it with its stack, and returns
(`TestLoggingHandlerPanic`).

Commits: "fix(logging): serialize rcl logging state like rclcpp's global logging mutex", "fix(node): serialize
rcl_node_init and rcl_node_fini with the logging mutex" (#57); "fix(logging): recover panics in the logging output
handler", "fix(logging): hold the logging mutex for the whole of rcutils_log and the level queries" (#58).

## 4. rcl's error state is per OS thread

**Before:** rcl stores its last error message per OS thread. Go can move a goroutine to another thread between the
failing C call and the `rcutils_get_error_string` call that reads the message, so the message could be lost or
belong to another goroutine. rclgo never pinned goroutines to threads.

**After:** every function that makes an rcl call and then reads the error state pins its goroutine to its OS thread
with `runtime.LockOSThread` for its whole duration (rationale at `errorString` in `pkg/rclgo/error.go`; more than 80
call sites).

Commit: "fix: keep rcl error text with the goroutine that caused it; reject short CDR input" (#57).

## 5. Spin ignored entities created after it started (#3)

This is less about memory safety and more about Go code changing the set of entities while Spin runs.

**Before:** `Context.Spin` built one wait set from the resources that existed at that moment and never looked again.
A subscription created after Spin started never had its callback called.

**After:**
- Creating an entity triggers a "refresh" guard condition on each running Spin; the wait set rebuilds its lists under
  the resource store's mutex. Wait sets built by hand keep their explicit lists.
- Each entity is serviced by only one running wait set at a time (`lifetime.acquireFor`); a second Spin skips it with
  a warning. This matches rclcpp's rule that a node belongs to one executor at a time.
- Subscriptions and timers created with a nil callback are left to their owner to poll.

Bindings generated by an older `rclgo-gen` wrapped a nil callback in a function that called it, which now panics on
the first message. Regenerate them (see [migrating-from-v0.6.md](migrating-from-v0.6.md)).

Commits: "fix(spin): service resources created after Spin has started", "fix(spin): service each entity from one
running wait set at a time", "fix(spin): leave nil-callback subscriptions and timers to their owner" (#56).

## 6. A clock's jump callbacks

rcl keeps a clock's time-jump callbacks in an unsynchronized array; timers and action servers on a ROS-time clock
register into it. A per-Clock mutex, `Clock.jumpMu`, now serializes those calls.

Commit: "fix: make closing entities and contexts safe while Spin is running" (#56).

## 7. The intentionally leaked node (#26)

This happens only after a failure, but it is the same root problem: other threads (rcl's logging, the DDS
middleware's own listener threads) still hold pointers into memory that rclgo frees. Those aren't Go threads, so the
crash has no Go traceback.

**Before:** `Node.Close` carried on regardless of failed steps.
1. If `rcl_logging_rosout_fini_publisher_for_node` failed, rcl kept the node's entry in its rosout map, keyed by a
   pointer to the node's logger name. rclgo still finalized and freed the `rcl_node_t`, so the key and the
   publisher's node dangled. A later node with the same name found the stale entry ("Publisher already registered
   for node name") and logged through it: intermittent SIGSEGV in `rcl_logging_multiple_output_handler`.
2. If `rcl_node_fini` failed, the rmw node stayed alive, yet `Context.Close` still ran `rcl_shutdown` and
   `rcl_context_fini`. rcl frees its context implementation, which embeds the `rmw_context_t`, even though
   `rmw_context_fini` refuses ("Finalizing a context with active nodes"). Fast DDS's listener thread kept using that
   context and later crashed in `rmw_fastrtps_shared_cpp::_take`.

Found by fault injection (`make test-faults` makes rcl calls fail by link-time wrapping); the crash appeared within a
few dozen further contexts in the same process.

**After:** if either step fails, freeing is the wrong move, so rclgo leaks on purpose.
- `Node.fini`: on a failed rosout fini or `rcl_node_fini`, rclgo drops its pointer without freeing it, counts the node
  in the context's `leakedNodes`, and returns "node … not finalized; its memory is leaked: …".
- `Context.fini`: if any node was leaked, it skips `rcl_shutdown`/`rcl_context_fini` and leaks the context too,
  returning "context not finalized because N of its nodes could not be; its memory is leaked".

The trade-off is a bounded memory leak after a failure that is already abnormal, instead of a use-after-free that
crashes the process later and somewhere unrelated. `Close` returns both errors, so the failure isn't silent.

Commit: "fix: handle failing rcl calls on error paths; generate error types per run" (#57).

## How the fixes are tested

- Every fix has a test that fails without it; the stress tests above give the before/after numbers.
- `make test-race`, `make test-repeat` (GOMAXPROCS 1 and 4), goleak, and `make test-faults` (see
  [testing.md](testing.md)).
- `make test-rmw` runs the suites under every installed RMW (Fast DDS, Cyclone DDS, Connext, Zenoh).

## Design notes

The fixes copy rclcpp and rclpy rather than inventing new rules: a global logging mutex, rcl's rule for the same node,
"free later" closing, and one executor per entity. The one Go-specific addition is pinning goroutines to their OS
thread (4), because neither rclcpp nor rclpy has a scheduler that moves work between threads.
