# Proposal: Add Neighbor State Exchange

## What

Add a deterministic neighbor-state communication layer for drones.

The change introduces a communication boundary where each drone publishes its own state snapshot and receives delayed snapshots from nearby drones. Swarm behavior will consume these received snapshots instead of reading every other drone's live position and velocity directly.

## Why

The current swarm controller implements local swarm rules, but it still has perfect central access to all drone state. That makes the behavior easier to write, but it hides the communication constraints that matter for realistic swarm simulation.

This change creates the first testable communication abstraction without taking on full radio realism. It focuses on range, fixed latency, deterministic packet loss, stale state, explicit configuration, and deterministic tests.

## Goals

- Model neighbor state exchange with explicit range, latency, and maximum age.
- Model packet loss with an explicit deterministic loss configuration.
- Keep behavior deterministic under fixed-step simulation.
- Make missing or invalid communication configuration fail fast with clear errors.
- Preserve current swarm control intent while changing neighbor data access from live state to received snapshots.
- Expose diagnostics useful for tests and later HUD integration.

## Non-Goals

- Bandwidth limits.
- Encryption.
- Multi-hop routing.
- Ground-control messages.
- Mission commands.
- Consensus protocols.
- Sensor noise.

## Success Criteria

- A drone cannot see another drone's published state until the configured latency has elapsed.
- Packet loss can drop in-range snapshots before delivery scheduling.
- Packet loss behavior is deterministic for a given explicit seed.
- Delivered snapshots preserve publish-time position and velocity.
- Out-of-range drones do not schedule deliveries.
- Expired snapshots are removed and no longer count as communication neighbors.
- Swarm neighbor calculations use communication snapshots rather than direct all-drone live state.
- `go test ./...` is expected to pass after implementation, including the existing `cmd/headless` constructor mismatch.

## Affected Areas

- `internal/sim/` for the communication model and swarm integration.
- `internal/sim/simulator.go` for explicit communication configuration wiring.
- `cmd/headless/main.go` for constructor compatibility and explicit config wiring.
- `test/` for deterministic communication and swarm integration tests.

## Reference Spec

See `open-spec/communication/neighbor-state-exchange.md`.
