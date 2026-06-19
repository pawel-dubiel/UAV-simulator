# Tasks: Visualize And Drive Swarm Coordination

## Implementation

- [x] Add swarm-level diagnostics that expose communication metrics without leaking mutable communication internals.
- [x] Add communication link snapshots for rendering visible receiver-to-sender neighbor relationships.
- [x] Add selected-drone objective state to `Swarm`.
- [x] Add explicit objective tuning fields to `SwarmConfig` with fail-fast validation.
- [x] Integrate selected-drone objective into swarm acceleration only when a drone has a received selected-drone snapshot.
- [x] Wire selected drone changes from `Simulator` into `Swarm` objective state.
- [x] Add 3D line rendering support to `Renderer`.
- [x] Render communication links before drones in normal and interpolated render paths.
- [x] Add HUD communication diagnostics beside existing swarm debug information.

## Tests

- [x] Add tests for `SwarmConfig` objective validation.
- [x] Add tests proving objective disabled by `GoalWeight == 0` has no direct selected-drone seek effect.
- [x] Add tests proving a drone without a selected-drone snapshot does not seek the selected drone directly.
- [x] Add tests proving a drone with a selected-drone snapshot receives selected-drone objective influence.
- [x] Add tests proving communication links reflect visible snapshots.
- [x] Add tests proving expired snapshots are omitted from communication links.
- [x] Add tests proving swarm diagnostics expose communication metrics.

## Verification

- [x] Run `go fmt ./...`.
- [x] Run `go test ./...`.
- [x] Run `go vet ./...`.
- [x] Run `go run . -headless -steps=120 -ups=120`.
- [ ] Manually run the simulator and confirm communication links/HUD are visible.
- [ ] Manually move the selected drone and confirm nearby swarm members visibly respond.

## Out of Scope

- [x] Do not convert to true per-drone agents.
- [x] Do not add bandwidth modeling.
- [x] Do not add routing or multi-hop relays.
- [x] Do not add mission planning.
- [x] Do not add obstacle avoidance.
