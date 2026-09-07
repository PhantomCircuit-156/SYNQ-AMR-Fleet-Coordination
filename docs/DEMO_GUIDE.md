# SYNQ SIH Judge Demo Guide

## Start

From `C:\SYNQ` on Windows:

```powershell
.\.venv\Scripts\python.exe scripts\run_dashboard.py
```

Open `http://127.0.0.1:8000`. Select **Full SIH Demo**, press **Load**, enable **Presentation Mode**, then press **Start / Resume**. Full Demo runs at a deliberately narrated one logical tick per second at 1×; use 2× or 4× for a shorter walkthrough.

## 15-second explanation

> SYNQ has no central traffic controller. Every AMR plans locally, shares a five-step future occupancy intent, predicts conflicts from its own local world model, deterministically resolves the same winner, and then moves. The dashboard only observes and injects missions or environment events.

## Full SIH Demo phases

1. Three independent AMRs receive warehouse missions.
2. Each publishes temporal intent (`+1` through `+5`).
3. Both sides of an overlap detect it locally and derive the same winner.
4. The winner passes; the yielding robot resumes.
5. A HIGH-priority payload mission is allocated to a capacity-eligible AMR.
6. A pallet blocks an active route; the affected robot replans locally.
7. R2 is disabled, remains a physical obstacle, misses heartbeats, and its unfinished task is released/reassigned after peer-local timeout observations.
8. Remaining work completes with the collision invariant still at zero.

Use **Pause**, **Step +1**, or **Next Phase** while explaining. Next Phase advances normal simulation ticks until the narration changes; it never chooses traffic behavior.

## Engineering mode

Engineering Mode exposes explicit tools: **Select**, **Set Pickup**, **Set Dropoff**, **Add Obstacle**, **Remove**, and **Inspect Cell**. Create a mission by selecting distinct walkable pickup/dropoff cells, task/priority/payload types, payload ID, weight, and fragile metadata. Capacity-ineligible AMRs are excluded deterministically. The mission card shows real phase/path progress, ETA ticks, payload, and the actual allocation reason.

Click an AMR or fleet card to populate the read-only inspector. It shows mission, motion, local intent/decision, winner/reason, corridor ownership, wait/replan counters, battery, payload, capacity, and peer liveness. Operators cannot steer robots or command yield/right-of-way.

Dynamic objects can be labeled BOX, PALLET, FORKLIFT, or BLOCKED_AISLE; all are fixed occupied cells in this prototype. Map zoom/fit and browser fullscreen affect visualization only.

## Recommended supporting scenarios

- **Intersection Conflict:** pause on the red prediction marker and compare R1/R2 local decisions.
- **Head-On Conflict:** show edge-swap prediction and local side yield.
- **Narrow Corridor:** show distributed leased ownership.
- **Deadlock Recovery:** show deterministic backoff events.
- **Robot Failure:** manually fail R2 and observe two independent timeout/offline observations.
- **Offline Robot Blockage:** show that the disabled AMR remains physical occupancy while another AMR replans.

## Honest scope

Human safety is intentionally deferred in this pass. The repository contains earlier safety value objects, but there is no judge-facing Human Crossing scenario because RED/YELLOW/GREEN behavior is not yet integrated and fully tested. No ROI or efficiency-improvement claims are displayed; Operations Overview uses only current simulation metrics.