# Implementation Plan

This roadmap deliberately postpones feature implementation until the preceding foundations are tested.

1. **Phase 1 — Models and configuration:** Create domain dataclasses/enums, map/scenario configuration, IDs, and deterministic simulation settings.
2. **Phase 2 — Warehouse and A*:** Implement grid warehouse representation, static obstacles, A* local routing, and planner tests.
3. **Phase 3 — Robot simulator:** Add tick-based robot entities, motion application, occupancy validation, and a headless deterministic runner.
4. **Phase 4 — Local world models:** Give each robot independently maintained peer/obstacle knowledge with controlled observation latency.
5. **Phase 5 — Peer communication:** Implement message contracts and a simulated decentralized transport with delay, loss, and ordering controls.
6. **Phase 6 — Intents:** Add short-horizon intent publication, receipt, expiry, and local intent tracking.
7. **Phase 7 — Conflict detection and resolution:** Detect cell and edge conflicts; implement deterministic local yield/proceed/replan rules.
8. **Phase 8 — Corridor and deadlock:** Model narrow corridors, detect wait cycles, and recover through deterministic backoff or replanning.
9. **Phase 9 — Human safety:** Add human/dynamic-obstacle zones and robot-local safety stop behaviour.
10. **Phase 10 — Dynamic replanning:** Replan locally around newly observed obstacles, delayed peers, and route invalidation.
11. **Phase 11 — Failure detection and task reassignment:** Detect peer failure through decentralized evidence and support task recovery without a traffic brain.
12. **Phase 12 — Metrics:** Capture local and fleet-level throughput, wait time, conflict, replan, safety, and communication metrics.
13. **Phase 13 — Dashboard:** Build FastAPI/WebSocket observability and task-creation interfaces without coordination authority.
14. **Phase 14 — Full deterministic demo:** Package a repeatable three-or-more-robot dynamic warehouse scenario and evidence views.
15. **Phase 15 — Stress testing and documentation:** Run repeatable stress scenarios, capture results, and complete operator/developer documentation.

## Exit criteria for every phase

- The feature respects the architecture and invariants in `AGENTS.md`.
- Focused `pytest` coverage includes expected behaviour and relevant failure/safety cases.
- A deterministic invocation reproduces the result.
- Documentation is updated if a public contract or coordination rule changes.
