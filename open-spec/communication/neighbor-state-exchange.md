# Neighbor State Exchange

## Status

Draft for implementation planning.

## Purpose

The simulator should model drone-to-drone communication as local neighbor state exchange rather than direct, perfect access to every drone's live state.

The first communication layer exists to create a clean, deterministic boundary between swarm behavior and shared drone state. It should be small enough to test thoroughly before adding packet loss, bandwidth limits, routing, or mission-level messages.

## Scope

This spec covers only neighbor state exchange:

- Each drone publishes its own compact state snapshot.
- Other drones receive snapshots only from drones within communication range.
- Received snapshots are delayed by a fixed configured latency.
- Swarm behavior consumes received neighbor snapshots, not direct live drone pointers, for neighbor position and velocity data.
- Stale snapshots remain visible until they expire by configured maximum age.

This spec does not cover:

- Packet loss.
- Bandwidth limits.
- Encryption.
- Routing or multi-hop relays.
- Ground-control messages.
- Mission commands.
- Consensus protocols.
- Sensor noise.

## Model

The communication model is deterministic and fixed-step friendly.

Each simulation update has two communication phases:

1. Publish: each drone produces a state snapshot from its current state.
2. Deliver: snapshots whose delivery time has arrived become visible to receiving drones.

The swarm controller then reads each drone's received neighbor table for local coordination.

Snapshots are immutable after publication. A drone receiving a snapshot sees the sender state as it was at publish time, not as it is at delivery time.

## Snapshot Data

The first snapshot type should include only fields needed by current swarm behavior and basic diagnostics:

- Sender ID.
- Position.
- Velocity.
- Armed state.
- Flight mode.
- Publish time.

Additional fields require a spec update before implementation.

## Configuration

Communication configuration must be explicit:

- `Range`: maximum sender-to-receiver distance at publish time.
- `Latency`: fixed delay before a published snapshot is delivered.
- `MaxAge`: maximum snapshot age retained in a receiver's neighbor table.

Invalid configuration must return explicit errors:

- `Range <= 0`.
- `Latency < 0`.
- `MaxAge <= 0`.
- `MaxAge < Latency`.

No implicit production defaults should be created inside the communication constructor.

## Identity

Each drone participating in communication needs a stable simulation-local ID.

The initial implementation can assign IDs from the drone slice index inside the communication/swarm setup, but the assigned ID must be explicit in published snapshots and diagnostics. Reordering drones after initialization is out of scope.

## Range Semantics

Range is evaluated at publish time using three-dimensional distance between sender and receiver positions.

If two drones are in range when the snapshot is published, the snapshot is scheduled for delivery even if they move out of range before delivery.

If two drones are out of range when the snapshot is published, no delivery is scheduled for that publish event.

## Staleness Semantics

Each receiver stores the most recent delivered snapshot per sender.

A snapshot is usable while:

```text
current_time - publish_time <= MaxAge
```

Expired snapshots are removed before swarm behavior reads neighbor state.

If a drone has no unexpired received snapshots, it has zero communication neighbors.

## Swarm Boundary

The swarm controller should stop calculating neighbors from direct access to other drones' live positions and velocities.

Allowed direct drone access:

- The controlled drone's own state.
- Applying torque or altitude targets to the controlled drone.
- Publishing the controlled drone's own snapshot.

Neighbor data must come from received snapshots.

## Error Handling

Construction errors should be returned with context.

Runtime programmer errors should fail fast:

- Non-positive communication update `dt` must panic, matching the current swarm update style.
- Nil drones in the configured drone list should be rejected during construction.
- Nil communication systems should not be silently ignored by swarm setup.

## Diagnostics

The communication layer should expose deterministic diagnostics:

- Average visible communication neighbors.
- Oldest visible snapshot age.
- Pending delivery count.

HUD integration can use these diagnostics later, but the first implementation should prioritize tests over rendering changes.

## Test Expectations

The first implementation should include headless unit tests for:

- Invalid communication configuration fails with explicit errors.
- In-range snapshots are not visible before latency elapses.
- In-range snapshots become visible after latency elapses.
- Delivered snapshots preserve publish-time position and velocity.
- Out-of-range snapshots are not delivered.
- Expired snapshots are removed after `MaxAge`.
- Swarm neighbor count uses communication snapshots rather than live all-drone access.

Tests should use fixed `dt` values and avoid wall-clock time.
