# Demo Requirements

## Minimum final prototype demonstration

The final prototype must demonstrate all of the following:

1. At least **three active autonomous mobile robots** in one warehouse scenario.
2. A **dynamic warehouse** containing changing obstacles, human/safety events, or both.
3. **Decentralized coordination**: each robot selects its own movement from local state and peer messages.
4. No central component grants right-of-way, reserves traffic, or determines movements.
5. Collision safety: two active robots never share a cell, including under conflict scenarios.
6. Deterministic conflict resolution with visible/reported reasons and tie-breakers.
7. At least one demonstrated conflict, yield/replan decision, and recovery from constrained movement.
8. Recorded metrics for throughput, conflicts, waiting, replanning, and safety stops.

## Evidence to prepare

- A seeded scenario configuration and single command to run it.
- A view of each robot's state, route, current intent, and local decision reason.
- A message/event trace showing peer-to-peer coordination.
- Automated tests for core safety and coordination behaviours.
- A short explanation that distinguishes simulation world ownership from traffic decision ownership.

## Explicit non-goals for the simulator and dashboard

- The simulator may reject invalid simultaneous occupancy, but may not choose who proceeds.
- The dashboard may create tasks and display telemetry, but may not coordinate, prioritize, or resolve traffic.
