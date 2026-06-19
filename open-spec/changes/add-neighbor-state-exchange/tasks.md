# Tasks: Add Neighbor State Exchange

## Implementation

- [x] Add `internal/sim/communication.go` with `CommConfig`, `PacketLossMode`, `NeighborSnapshot`, `CommSystem`, and `CommDiagnostics`.
- [x] Implement explicit `CommConfig` validation with clear errors and no implicit production defaults.
- [x] Implement deterministic publish, range filtering, packet loss, delayed delivery, snapshot replacement, and expiration logic.
- [x] Add a `NeighborsFor(receiverID int)` API that returns received snapshots without exposing mutable internal storage.
- [x] Integrate communication into `Swarm` so neighbor position and velocity data comes from snapshots instead of direct live drone reads.
- [x] Keep swarm control access to each controlled drone's own live state for torque and altitude target application.
- [x] Add explicit simulator communication wiring in `internal/sim/simulator.go`.
- [x] Fix stale `NewSwarm` call sites, including `cmd/headless/main.go`.

## Tests

- [x] Add communication config validation tests.
- [x] Add tests proving snapshots are invisible before fixed latency elapses.
- [x] Add tests proving snapshots become visible after fixed latency elapses.
- [x] Add tests proving delivered snapshots preserve publish-time position and velocity.
- [x] Add tests proving out-of-range drones do not schedule deliveries.
- [x] Add tests proving loss mode `None` does not drop in-range snapshots.
- [x] Add tests proving probabilistic loss at rate `1.0` drops all in-range snapshots.
- [x] Add tests proving probabilistic loss with the same nonzero seed produces the same drop/delivery sequence.
- [x] Add tests proving snapshots expire after `MaxAge`.
- [x] Add swarm integration tests proving neighbor counts and behavior use communication snapshots rather than live all-drone state.

## Verification

- [x] Run `go fmt ./...`.
- [x] Run `go test ./...`.
- [x] Run `go vet ./...`.
- [x] Confirm the existing `cmd/headless` build failure is resolved.

## Out of Scope

- [x] Do not add bandwidth modeling.
- [x] Do not add routing or multi-hop relays.
- [x] Do not add mission or ground-control messages.
- [x] Do not retune swarm control gains unless tests show instability caused by the communication boundary.
