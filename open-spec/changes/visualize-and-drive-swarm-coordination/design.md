# Design: Visualize And Drive Swarm Coordination

## Overview

The current communication system creates received neighbor snapshots, but the simulator does not expose them visually and the swarm has no strong group objective. This change adds observability and a small, communication-aware objective.

Conceptually:

```text
Drone state
  -> CommSystem snapshots
  -> Swarm neighbor logic
       ├─ local boids behavior
       └─ selected-drone objective when visible by snapshot
  -> Renderer/HUD diagnostics
```

The goal is not to create final swarm intelligence. The goal is to make communication and coordination obvious enough to inspect.

## Communication Link Visualization

Render a line for each receiver-to-sender relationship represented by a currently visible `NeighborSnapshot`.

Recommended semantics:

- A link means "receiver currently has a non-expired snapshot from sender."
- Draw links using snapshot sender position to receiver live position.
- Color by snapshot age:
  - fresh: green/cyan
  - near expiry: yellow
  - expired links are not drawn
- Optionally vary alpha by age.

Because current `Renderer` only draws drones and ground, add a small dynamic line renderer path rather than overloading cube geometry.

Expected rendering shape:

```text
Renderer
  ├─ RenderGround()
  ├─ RenderDrone()
  └─ RenderLines([]LineVertex)
```

Line rendering should be separate from `UIRenderer` because these are 3D world-space links, not screen-space HUD elements.

## HUD Diagnostics

Expose communication diagnostics through `Swarm`, then display them in the existing HUD block.

Current HUD already shows:

```text
SWARM N 20  SPREAD 7M  NEI 3
```

Add a second line:

```text
COMM VIS 2.8  PEND 14  DROP 3/120  AGE 80MS
```

Where:

- `VIS`: average visible communication neighbors.
- `PEND`: pending deliveries.
- `DROP`: dropped packets over delivery attempts.
- `AGE`: oldest visible snapshot age.

Keep labels short enough for the current 360px panel.

## Selected-Drone Objective

Add a visible swarm objective tied to the selected drone.

The important constraint: non-selected drones should not use direct live selected-drone state unless they are the selected drone. They may seek the selected drone only if their communication table contains a visible snapshot from the selected drone.

Behavior:

- The selected drone remains pilot-controlled.
- Other drones keep existing local cohesion, alignment, separation, and altitude averaging.
- If a drone has a received snapshot from the selected drone, add a goal-seeking acceleration toward that snapshot position.
- If it does not have that snapshot, it only follows local neighbor behavior.

This makes communication effects visible:

```text
selected drone moves
      │
      ▼
drones with selected snapshot seek it
      │
      ▼
other drones respond through local neighbor cohesion/alignment
```

Packet loss, latency, range, and staleness will naturally affect how strongly and how quickly the swarm responds.

## Configuration

Keep configuration explicit.

Add swarm objective tuning to `SwarmConfig` rather than hiding it as implicit behavior:

- `GoalWeight`: acceleration weight for selected-drone objective.
- `GoalRadius`: deadband radius around the selected-drone snapshot where direct seek fades or stops.

Invalid configuration should fail fast:

- `GoalWeight < 0`
- `GoalRadius < 0`

If `GoalWeight == 0`, the selected-drone objective is disabled. This gives tests and future scenarios an explicit off switch.

## API Shape

Potential additions:

- `(*Swarm) SetObjectiveDrone(id int)`
- `(*Swarm) ClearObjective()`
- `(*Swarm) Diagnostics() SwarmDiagnostics`
- `(*Swarm) CommunicationLinks() []CommunicationLink`

Possible support types:

```text
SwarmDiagnostics
  Spread
  AverageCommunicationNeighbors
  PendingDeliveries
  DeliveryAttempts
  DroppedPackets
  OldestSnapshotAge

CommunicationLink
  ReceiverID
  SenderID
  ReceiverPosition
  SenderSnapshotPosition
  SnapshotAge
  MaxAge
```

`CommunicationLinks` should expose copies, not mutable internal state.

## Rendering Integration

In `render` and `renderInterpolated`:

1. Render ground.
2. Render communication links.
3. Render drones.
4. Render HUD.

Rendering links before drones keeps drones visually dominant.

Interpolated drone positions create a small mismatch with snapshot positions. That is acceptable because links represent communication state, not exact physical rods.

## Testing

Headless tests should cover behavior and diagnostics without OpenGL:

- Swarm diagnostics expose communication metrics.
- Objective is disabled when `GoalWeight == 0`.
- Drone without a selected-drone snapshot does not seek selected drone directly.
- Drone with a selected-drone snapshot receives a goal-seeking command or changed attitude target.
- Communication links reflect visible snapshots and omit expired snapshots.

Renderer link drawing can be kept thin and verified through construction-level tests where practical. Full visual QA can be manual unless a browser/OpenGL capture flow is added later.

## Risks

The selected-drone objective can fight local separation if too strong. Keep it bounded by existing `MaxAccel` and make it additive with local rules rather than overriding them.

HUD space is limited. Prefer short labels and avoid adding another large panel.

Drawing all links is O(N * visible neighbors). For 20 drones this is fine. If swarm size grows, add a debug toggle or link cap later.

## Deferred Architecture

True per-drone agents remain a later change.

This change should avoid moving controller ownership into `Drone` or `DroneAgent`. It should expose enough behavior to decide whether that larger refactor is justified.
