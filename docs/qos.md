# QoS (Quality of Service) in rclgo

ROS 2 lets you tune communication behavior per topic/service/action via **QoS**.
In rclgo this lives in `pkg/rclgo/qos` and centers on a lightweight `Profile` you can customize or build from presets.

---

## Profile

```go
// pkg/rclgo/qos/profile.go (summary)
type Profile struct {
    History   HistoryPolicy   // KeepLast or KeepAll
    Depth     int             // queue depth when History=KeepLast
    Reliability ReliabilityPolicy   // Reliable or BestEffort
    Durability  DurabilityPolicy    // Volatile or TransientLocal
    Deadline   time.Duration        // message period constraint (ns)
    Lifespan   time.Duration        // how long a message stays valid (ns)
    Liveliness LivelinessPolicy     // Automatic | ManualByNode | ManualByTopic
    LivelinessLeaseDuration time.Duration // (ns)
    AvoidROSNamespaceConventions bool
}
```

### Enums

* `HistoryPolicy`: `HistoryUnknown`, `HistoryKeepLast`, `HistoryKeepAll`
* `ReliabilityPolicy`: `ReliabilityUnknown`, `ReliabilityReliable`, `ReliabilityBestEffort`
* `DurabilityPolicy`: `DurabilityUnknown`, `DurabilityVolatile`, `DurabilityTransientLocal`
* `LivelinessPolicy`: `LivelinessUnknown`, `LivelinessAutomatic`, `LivelinessManualByNode`, `LivelinessManualByTopic`

> Durations are `time.Duration` and represented as nanoseconds in the underlying RMW struct. If you don’t need a constraint, leave them at `0`.

---

## Presets

The package provides convenience constructors that mirror common ROS 2 profiles:

```go
qos.NewClockProfile()
// KeepLast(1), BestEffort, Volatile, LivelinessAutomatic

qos.NewSensorDataProfile()
// KeepLast(5), BestEffort, Volatile, LivelinessAutomatic

qos.NewParameterEventsProfile()
// KeepAll, Reliable, TransientLocal, LivelinessAutomatic
```

Use a preset as a base, then tweak what you need:

```go
p := qos.NewSensorDataProfile()
p.Depth = 10            // bump queue depth
p.Reliability = qos.ReliabilityReliable
```

---

## Typical usage

You pass a `qos.Profile` when creating publishers/subscriptions/services.
(Exact option names vary by helper; the idea is the same—provide a `qos.Profile`.)

```go
// Publisher example
p := qos.NewSensorDataProfile()
pub, err := node.NewPublisher(ctx, std_msgs.StringTypeSupport, "/camera/imu",
    rclgo.WithPublisherQos(p))
if err != nil { /* handle */ }

// Subscription example
subQ := qos.NewSensorDataProfile()
sub, err := node.NewSubscription(ctx, std_msgs.StringTypeSupport, "/camera/imu",
    rclgo.WithSubscriptionQos(subQ), callback)
```

Services/actions typically use a reliable, volatile profile with a small depth; use your project’s helpers or build directly from `qos.Profile`.

---

## Interop with RMW (C layer)

You shouldn’t need this in normal code, but for completeness:

* `Profile.AsCStruct(*C.rmw_qos_profile_t)` writes a profile into the RMW struct.
* `(*Profile).FromCStruct(*C.rmw_qos_profile_t)` reads back from the RMW struct.

These bridge functions are what rclgo calls under the hood when it constructs publishers, subscriptions, services, and actions.

---

## Choosing the right QoS

A quick cheat sheet:

* **Sensor streams (cameras, IMU, LIDAR):** `SensorData` (BestEffort, small KeepLast) to minimize latency and drop old frames under load.
* **Status/telemetry where message loss is ok:** BestEffort + small depth.
* **Commands/config where delivery matters:** Reliable + KeepLast(depth≥1).
* **Parameter events / latched-like behavior:** Reliable + TransientLocal (+ KeepAll).
* **Clock/time sync topics:** `Clock` (BestEffort, KeepLast(1)).

If you mix profiles across publishers/subscribers on the same topic, ROS 2 uses compatibility rules. A common pitfall is pairing a BestEffort subscriber with a Reliable publisher across lossy networks—prefer matching policies unless you have a reason not to.

---

## IDE & cgo notes (reference)

* Tests and builds that touch QoS interop require **cgo** and the ROS 2 headers/libs visible to the toolchain.
* If you’re not using `pkg-config`, ensure your IDE sets:

    * `CGO_ENABLED=1`
    * `CC=/usr/bin/gcc`, `CXX=/usr/bin/g++`
    * `C_INCLUDE_PATH=/opt/ros/humble/include`
    * `LIBRARY_PATH` and `LD_LIBRARY_PATH` to include `/opt/ros/humble/lib` and (if present) `/opt/ros/humble/lib/x86_64-linux-gnu`
    * `GOCACHE` to a writable path (e.g., `$USER_HOME$/.cache/go-build` in GoLand with “process path macros” enabled)
* Our tests avoid `pkg-config` by setting `PKG_CONFIG=/bin/true` and feeding include/library paths directly.

---

## Troubleshooting

* **Subscriber not receiving data:** Check that publisher/subscriber reliability and durability are compatible; mismatched reliability over a lossy link is a frequent culprit.
* **Messages “missing”:** With `BestEffort`, drops under load are expected; switch to `Reliable` or increase `Depth`.
* **Late joiners see nothing:** Use `TransientLocal` on the publisher if you need late subscribers to get the last value (e.g., parameter or latched-like topics).
* **IDE builds fail but terminal passes:** your IDE likely isn’t inheriting the environment used in your shell (include/lib paths, `GOCACHE`). See the cgo notes above.

---

## API surface at a glance

```go
// Profile fields & enums (see above)

// Presets
func NewClockProfile() Profile
func NewSensorDataProfile() Profile
func NewParameterEventsProfile() Profile

// Interop (used internally)
func (p Profile) AsCStruct(dst *C.rmw_qos_profile_t)
func (p *Profile) FromCStruct(src *C.rmw_qos_profile_t)
```
