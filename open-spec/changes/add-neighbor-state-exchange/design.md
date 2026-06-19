# Design: Neighbor State Exchange

## Overview

Add a deterministic communication system that sits between drone state and swarm control.

Each update publishes immutable snapshots of current drone state, filters receiver deliveries by range, applies deterministic packet loss, schedules surviving deliveries, and makes delivered snapshots visible after a fixed latency. The swarm controller reads neighbor snapshots from this communication system when calculating cohesion, alignment, separation, altitude target averages, and neighbor diagnostics.

## Core Types

Add a new communication module in `internal/sim/`, likely `communication.go`.

Expected types:

- `CommConfig`: explicit communication tuning.
- `PacketLossMode`: explicit loss mode with an invalid zero value.
- `NeighborSnapshot`: immutable delivered state from one sender.
- `CommSystem`: owns simulation communication time, pending deliveries, and receiver neighbor tables.
- `CommDiagnostics`: deterministic communication metrics for tests and later HUD use.

`CommConfig` fields:

- `Range float64`
- `Latency float64`
- `MaxAge float64`
- `LossMode PacketLossMode`
- `PacketLossRate float64`
- `LossSeed uint64`

`NeighborSnapshot` fields:

- `SenderID int`
- `Position Vec3`
- `Velocity Vec3`
- `IsArmed bool`
- `FlightMode FlightMode`
- `PublishTime float64`

## Construction

Communication construction must validate all required configuration.

Invalid configuration returns explicit errors:

- `Range <= 0`
- `Latency < 0`
- `MaxAge <= 0`
- `MaxAge < Latency`
- missing or invalid `LossMode`
- `LossModeNone` with nonzero `PacketLossRate` or nonzero `LossSeed`
- `LossModeProbabilistic` with `PacketLossRate < 0` or `PacketLossRate > 1`
- `LossModeProbabilistic` with `LossSeed == 0`

Nil drones are rejected by swarm construction before communication state is created. No constructor should invent production defaults.

`PacketLossMode` should make the zero value invalid, for example:

```text
Invalid -> omitted or unusable configuration
None -> no packet loss; rate and seed must be zero
Probabilistic -> deterministic seeded packet loss
```

## Identity

The initial implementation uses the drone slice index as the stable simulation-local sender ID.

This is acceptable only while drones are not inserted, removed, or reordered after swarm construction. Published snapshots and diagnostics must expose the ID explicitly so a later dynamic-drone model has a clear migration path.

## Update Order

For each fixed simulation update:

1. Communication advances by `dt`.
2. Expired delivered snapshots are removed.
3. Current drone states are published.
4. For each sender-to-receiver pair, range is checked at publish time.
5. In-range delivery attempts are passed through the packet loss model.
6. Surviving delivery attempts are scheduled.
7. Deliveries whose scheduled time has arrived are inserted into receiver neighbor tables.
8. Swarm control reads receiver neighbor tables and applies control outputs.
9. Drone physics updates.

This means a snapshot published during a given update is not usable until at least its configured latency has elapsed.

## Delivery Semantics

Range is evaluated at publish time using three-dimensional distance.

If sender and receiver are in range at publish time, a delivery is scheduled for:

```text
publish_time + Latency
```

If sender and receiver are out of range at publish time, no delivery is scheduled.

Receiver motion after scheduling does not cancel the delivery. Later versions can introduce link revalidation if needed, but this change keeps delivery deterministic and simple.

## Packet Loss Semantics

Packet loss is evaluated after range filtering and before delivery scheduling.

`LossModeNone` delivers every in-range delivery attempt.

`LossModeProbabilistic` evaluates each sender-to-receiver delivery attempt independently against `PacketLossRate`. Dropped packets are not scheduled for delivery and do not update receiver neighbor tables.

Packet loss must be deterministic for a given `LossSeed`, update sequence, and drone order. Tests must not depend on wall-clock time or global random state.

Loss is directional. A snapshot from drone A to drone B can be dropped while the same publish event from drone A to drone C survives.

## Staleness Semantics

Each receiver keeps only the latest delivered snapshot per sender.

A snapshot is visible while:

```text
current_time - PublishTime <= MaxAge
```

Expired snapshots are removed before swarm behavior reads neighbors.

## Swarm Integration

`Swarm` should stop scanning all other drones as live neighbors for behavior calculations.

The controller can still access the controlled drone's own live state and can still apply torque and altitude target changes to that drone. For every other drone, behavior uses `NeighborSnapshot` data from the communication layer.

Recommended API shape:

- `NewCommSystem(drones []*Drone, cfg CommConfig) (*CommSystem, error)`
- `(*CommSystem) Update(dt float64, drones []*Drone)`
- `(*CommSystem) NeighborsFor(receiverID int) []NeighborSnapshot`
- `(*CommSystem) Diagnostics() CommDiagnostics`

`Update` must panic for `dt <= 0`, matching current `Swarm.Update` behavior.

The swarm constructor should require an explicit communication dependency or explicit communication config. A nil communication system should fail fast rather than silently falling back to direct live state.

## Configuration Wiring

Keep swarm movement tuning and communication tuning separate.

Recommended project-level helper:

- Existing `swarmConfig()` continues returning motion/control config.
- Add `commConfig()` returning explicit simulator communication config.

Tests should use local helper configs rather than production defaults.

## Diagnostics

Expose:

- Average visible communication neighbors.
- Oldest visible snapshot age.
- Pending delivery count.
- Delivery attempt count.
- Dropped packet count.

Swarm HUD can continue showing existing spread and neighbor metrics initially. HUD changes are not required for the first implementation unless they are needed to avoid misleading labels.

## Tests

Add headless unit tests that do not depend on wall-clock time.

Required behavior tests:

- Invalid configuration errors.
- Snapshot invisible before latency.
- Snapshot visible after latency.
- Snapshot preserves publish-time position and velocity.
- Out-of-range snapshot not delivered.
- Loss mode `None` does not drop in-range snapshots.
- Probabilistic loss at rate `1.0` drops all in-range snapshots.
- Probabilistic loss with the same nonzero seed produces the same drop/delivery sequence.
- Snapshot expires after `MaxAge`.
- Swarm behavior and neighbor count use communication snapshots instead of direct live state.

Also keep `go test ./...` green by updating stale constructor call sites, including `cmd/headless/main.go`.

## Risks

Changing neighbor data from live state to stale snapshots can alter swarm stability. The first implementation should preserve existing swarm gains and broaden tests only enough to catch major instability, not retune the controller prematurely.

The current repository already has a failing `go test ./...` due to a stale `cmd/headless` call to `NewSwarm`. Implementation must address that so communication test failures are not hidden behind an unrelated build failure.
