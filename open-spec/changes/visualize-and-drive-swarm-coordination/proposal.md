# Proposal: Visualize And Drive Swarm Coordination

## What

Add visible swarm communication feedback and a simple selected-drone swarm objective.

This change combines three related improvements:

- Draw communication links between drones that currently see each other through received neighbor snapshots.
- Show communication diagnostics in the HUD.
- Add a swarm objective that makes the group visibly react to the selected drone while still respecting snapshot-based neighbor communication.

## Why

The simulator now has a deterministic communication substrate, but the behavior is hard to see. Drones publish and consume neighbor snapshots, yet the current swarm rules mostly preserve local cohesion and can look static when drones start close together.

The next slice should make communication and coordination observable before moving to a larger per-drone-agent architecture. The user should be able to see whether drones are connected, whether communication is stale or dropping packets, and whether the swarm responds to a moving selected drone.

## Goals

- Make drone-to-drone communication visible in the renderer.
- Surface communication metrics in the HUD.
- Add a visible swarm objective tied to the selected drone.
- Keep the objective based on received snapshots, not direct live global state.
- Preserve deterministic headless tests for communication and swarm behavior.
- Keep packet loss and latency visible enough to diagnose behavior.

## Non-Goals

- Do not convert the architecture to true per-drone agents yet.
- Do not add bandwidth modeling.
- Do not add routing or multi-hop relays.
- Do not add mission planning.
- Do not add obstacle avoidance.
- Do not retune physics broadly.

## Success Criteria

- The rendered simulator shows communication links between drones with active received neighbor snapshots.
- HUD displays visible neighbor count, pending packet count, dropped packet count, and oldest snapshot age.
- Moving the selected drone creates a visible response in nearby swarm members.
- Only drones with a received selected-drone snapshot seek the selected drone directly.
- Drones without a selected-drone snapshot continue using local cohesion, alignment, and separation.
- Existing communication tests remain deterministic.
- `go test ./...` and `go vet ./...` pass after implementation.

## Affected Areas

- `internal/sim/communication.go` for richer diagnostics or link snapshot access.
- `internal/sim/swarm.go` for selected-drone objective logic and public diagnostics.
- `internal/sim/renderer.go` for line rendering.
- `internal/sim/simulator.go` for selected-drone objective wiring, rendering links, and HUD diagnostics.
- `test/` for headless behavior tests.

## Prior Work

This builds on `open-spec/changes/add-neighbor-state-exchange/`.
