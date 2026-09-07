# Engineering Rules

These rules are permanent for all contributors and future Codex runs.

1. **Coordination is decentralized.** Each robot makes its own coordination and movement decisions from its local state and peer messages. Do not introduce a central traffic controller.
2. **Simulation does not make traffic decisions.** It maintains world ground truth, applies already-selected actions, detects invalid physical states, and advances time only.
3. **Dashboard does not make traffic decisions.** It may observe the system and create tasks, but cannot grant right-of-way or select robot motions.
4. **Safety outranks priority.** A robot must choose a safe stop or yield over a higher-priority movement whenever needed.
5. **No shared occupancy.** Two active robots must never occupy the same grid cell at the same simulation tick.
6. **Conflict resolution is deterministic.** Given identical local state and messages, robots must reach the same resolution. Tie-breakers must be explicit and stable (for example, robot ID).
7. **Verify claims.** Never claim code works without running relevant tests or validation and reporting the result.
8. **Preserve working implementations.** Extend or refactor proven code carefully; do not rewrite it unnecessarily.
9. **Keep the prototype simple and deterministic.** Prefer understandable deterministic algorithms suitable for an SIH demonstration over premature complexity.
10. **Test behavioural features.** Every major behavioural feature requires focused automated tests, including normal and safety-related cases.

## Implementation discipline

- Keep robot-owned decision logic out of `simulation/` and `dashboard/`.
- Use dataclasses and enums for explicit domain state where they improve clarity.
- Keep simulation time and random seeds controllable for repeatable tests and demos.
- Document any change to inter-robot message format or conflict-resolution rule.
- Maintain Windows compatibility and Python 3.11+ support.
