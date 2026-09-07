# SYNQ Architecture

## Core principle

SYNQ is a decentralized multi-robot system. There is no global right-of-way service, central reservation manager, or traffic controller. Every robot makes its next movement decision locally using its own model and messages received from peers.

## Robot-owned components

Every robot conceptually owns the following components:

| Component | Responsibility |
| --- | --- |
| `LocalWorldModel` | Maintains the robot's local view of map, obstacles, peers, and uncertainty. |
| `LocalPlanner` | Produces and revises a path toward the assigned goal. |
| `IntentManager` | Publishes and tracks short-horizon movement intentions. |
| `PeerCommunication` | Sends, receives, timestamps, and validates peer messages. |
| `ConflictDetector` | Detects predicted cell, edge-swap, corridor, and safety conflicts. |
| `ConflictResolver` | Applies deterministic local rules to choose yield, proceed, wait, or replan. |
| `PriorityManager` | Calculates deterministic priority and stable tie-breaks. |
| `SafetyManager` | Enforces collision prevention and human-safety constraints. |
| `DeadlockManager` | Identifies stalled cycles and selects deterministic recovery actions. |
| `StateMachine` | Represents operational modes such as idle, moving, yielding, waiting, and faulted. |
| `Metrics` | Records local decisions, delays, replans, conflicts, and safety events. |

## System boundaries

```text
Task source / dashboard ── assigns task ──> Robot
Dashboard <────────────── observes state ── Simulator / Robots

Robot A <──── peer messages ────> Robot B <──── peer messages ────> Robot C
  local decision                    local decision                    local decision

Simulator: stores ground truth, applies selected actions, advances ticks,
           and reports physical violations. It does not select right-of-way.
```

## Planned package responsibilities

- `config/`: validated, deterministic scenario and simulation configuration.
- `models/`: shared dataclasses, enums, and immutable message/domain contracts.
- `planning/`: local path planning algorithms and route representations.
- `coordination/`: robot-local intent, conflict, priority, and deadlock logic.
- `networking/`: simulated peer transport and later dashboard transport adapters.
- `simulation/`: warehouse world, clock, robot actuation, and visualization integration.
- `safety/`: robot-local safety policies and human-zone constraints.
- `tasks/`: task models, assignment inputs, and robot task lifecycle.
- `metrics/`: event schemas, aggregation, and deterministic reporting.
- `dashboard/`: later read-only observability plus task creation interface.

## Invariants to preserve

1. Safety decisions override priority decisions.
2. Two active robots cannot occupy one cell during a tick.
3. Deterministic inputs, seed, and peer-message ordering produce deterministic results.
4. Robot motion proposals originate in robot-local code, never the simulator or dashboard.

## Decentralized intent and conflict flow

Each `RobotAgent` owns its `IntentManager`, `ConflictDetector`, and
`ConflictResolver`. The intent manager converts at most five upcoming path cells
into timestamped predicted occupancies. State messages broadcast that immutable
intent together with pose, task priority, waiting time, and heartbeat.

Each peer message is stored in the receiving robot's own `LocalWorldModel`.
Conflict detection compares that robot's own intent with only those received
snapshots. It detects same-tick cell occupancy, head-on edge swaps, trajectory
intersections at different predicted times, and opposing traversal of narrow
corridor cells. Conflict records normalize robot IDs and per-robot timing so two
robots given the same messages construct equal results.

The robot-local deterministic resolver ranks safety eligibility first, followed
by task priority, accumulated waiting time, and finally lexical robot ID. Battery
is telemetry and is not a right-of-way input. The simulator neither calls the
detector nor resolver; it continues to apply selected actions and reject invalid
physical occupancy only.
## Conflict behavior integration

Before proposing a path movement, each `RobotAgent` runs its own detector against
its `LocalWorldModel` and resolves each peer conflict locally. The normalized
winner is cached for that peer while the conflict episode remains active, so
accumulating wait time cannot make both robots alternate right-of-way mid-pass.
A winning robot continues its path. A losing robot advertises a hold intent and
enters `WAITING`; for an immediate head-on move or when its occupied cell blocks
the winner's predicted route, it may select a deterministic locally safe side
cell, enter `YIELDING`, and then wait there. Once its local conflict disappears,
it resumes the retained path or replans from the yield cell.

`FleetSimulation` does not call conflict detection or resolution and cannot set a
robot's movement state. It drains and publishes messages around each tick,
validates the independently proposed moves, advances positions, and records
physical collision and occupancy history. Conflict counts, blocked ticks,
wait/yield events, and resumptions belong to each robot's own metrics object.
## Distributed corridor ownership

Narrow corridors are explicit `CorridorRegion` records containing interior
cells, outside entry cells, and optional passing bays. Every `RobotAgent` owns a
`CorridorProtocol`. Approaching robots broadcast requests and locally evaluate
the same leased-token messages using safety eligibility, task priority, waiting
time, and robot ID. A single valid lease preserves ownership until the owner
leaves; requests and tokens disappear from its next state message after exit.
Tokens are also ignored when expired or when the owner's heartbeat exceeds the
configured offline timeout. The losing entrant waits in an outside passing bay.
The message bus only broadcasts these immutable records, and `FleetSimulation`
only distributes static corridor geometry.

## Deadlock recovery

Every robot also owns a `DeadlockManager` with a configurable seconds-to-ticks
threshold (six seconds by default). A deadlock requires a locally observed
mutual wait-for relationship: both robots are waiting past the threshold and
each advertises that it is blocked by the other. Both perspectives normalize
the pair, choose the lower robot ID to proceed and the higher ID to back off,
and select the same nearest passing bay. Robots log `DEADLOCK_DETECTED`,
`DEADLOCK_RESOLUTION`, and `DEADLOCK_CLEARED` locally.

Waiting time is considered before the robot-ID tie-break when a free corridor is
contested, so a repeatedly delayed equal-priority robot gains preference. A
remaining starvation limitation is that strict HIGH task priority can repeatedly
beat NORMAL work. Also, a live owner that keeps refreshing its lease while making
no progress inside a corridor is not yet expired by a progress watchdog; the
current offline timeout only handles missing heartbeats.
## Peer liveness and task reassignment

Every active `RobotAgent` publishes its heartbeat in `PeerStateMessage`. Each peer stores `peer_last_seen` in its own `LocalWorldModel` and applies the shared `HEARTBEAT_TIMEOUT` against its local simulation clock. On timeout, that observer marks the peer offline and removes the peer's state, intent/path reservation, corridor requests, and corridor tokens. Conflict and corridor decisions therefore automatically reconsider the remaining fresh peer data. `FleetSimulation.disable_robot` is fault injection only; it does not mark that robot offline in any peer model.

`DeterministicTaskDispatcher` is logically separate from motion coordination. It may create, release, and assign work after consuming robot-reported offline status. It cannot select paths, resolve conflicts, grant corridor ownership, or command yielding. Assignment uses distance to the task goal followed by robot ID as a stable tie-break. In short: **task dispatch is not traffic coordination**.
## Offline robot physical occupancy

Heartbeat timeout is a communication-state change, not physical removal. When the simulation disables a robot, it keeps that robot at its final cell and exposes the cell as environment occupancy to every active robot. Each agent stores that observation in its own `LocalWorldModel`, invalidates any remaining path that crosses it, and locally enters `REPLANNING`. A successful A* replan avoids the occupied cell; an unavailable route enters `UNREACHABLE` and remains stopped. The simulator only validates that no selected move enters physical occupancy; it never chooses a route or right-of-way.

Liveness events are intentionally local observations. R1 and R3 may each emit one `HEARTBEAT_TIMEOUT` and one `ROBOT_OFFLINE` event for R2. Each event carries `observer_robot_id` and `peer_robot_id`, so a future dashboard can distinguish local observations from fleet summaries.

The shared task registry is allowed for prototype task bookkeeping only. `DeterministicTaskDispatcher` releases a task only when its status is `ASSIGNED` or `IN_PROGRESS`, then immediately changes it to `RELEASED`; subsequent peer observations cannot satisfy that transition again. It assigns one deterministic winner and changes status back to `ASSIGNED`. It has no traffic-control API.