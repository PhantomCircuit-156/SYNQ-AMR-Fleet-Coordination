"""Local SIH visualization API. It observes simulation decisions; it never makes them."""
from __future__ import annotations
from contextlib import asynccontextmanager
import asyncio
from dataclasses import dataclass, field
from threading import RLock
from typing import Any
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from models.grid import Cell
from models.robot_state import RobotState
from simulation.coordination_scenarios import build_two_robot_intersection_scenario
from simulation.corridor_scenarios import build_opposing_corridor_scenario, build_recoverable_deadlock_scenario
from simulation.failure_scenarios import build_robot_failure_scenario
from simulation.offline_blockage_scenarios import build_offline_robot_blockage_scenario
from simulation.scenarios import build_basic_three_robot_scenario
from tasks.lifecycle import DeterministicTaskDispatcher, FleetTask, TaskStatus

SCENARIOS = ("basic", "intersection", "corridor", "deadlock", "robot_failure", "offline_robot_blockage")

class TaskRequest(BaseModel):
    destination_x: int = Field(ge=0)
    destination_y: int = Field(ge=0)

@dataclass(slots=True)
class DashboardEngine:
    """Single authoritative simulation clock for a local dashboard session."""
    simulation: Any = field(init=False)
    scenario_name: str = field(default="basic", init=False)
    zones: tuple[Any, ...] = field(default=(), init=False)
    _lock: RLock = field(default_factory=RLock, init=False)
    _events: list[dict[str, Any]] = field(default_factory=list, init=False)
    _event_keys: set[tuple[Any, ...]] = field(default_factory=set, init=False)
    _previous_states: dict[str, str] = field(default_factory=dict, init=False)
    _previous_conflicts: dict[str, tuple[int, int]] = field(default_factory=dict, init=False)
    _failure_at: int | None = field(default=None, init=False)
    _failure_triggered: bool = field(default=False, init=False)
    _task_number: int = field(default=0, init=False)

    def __post_init__(self) -> None: self.reset("basic")

    def reset(self, scenario_name: str = "basic") -> None:
        if scenario_name not in SCENARIOS: raise ValueError(f"Unknown scenario: {scenario_name}")
        with self._lock:
            self.scenario_name=scenario_name; self._events=[]; self._event_keys=set(); self._previous_states={}; self._previous_conflicts={}; self._failure_triggered=False; self._failure_at=None
            if scenario_name == "basic":
                scenario, self.simulation = build_basic_three_robot_scenario(); self.zones=scenario.zones
            elif scenario_name == "intersection": self.simulation=build_two_robot_intersection_scenario().simulation; self.zones=()
            elif scenario_name == "corridor": self.simulation=build_opposing_corridor_scenario(); self.zones=()
            elif scenario_name == "deadlock": self.simulation=build_recoverable_deadlock_scenario(); self.zones=()
            elif scenario_name == "robot_failure": self.simulation=build_robot_failure_scenario().simulation; self.zones=(); self._failure_at=3
            else: self.simulation=build_offline_robot_blockage_scenario().simulation; self.zones=(); self._failure_at=2
            self._collect_events()

    def tick(self) -> None:
        with self._lock:
            if self._failure_at is not None and not self._failure_triggered and self.simulation.tick_count >= self._failure_at:
                self.simulation.disable_robot("R2"); self._failure_triggered=True
            active = any(robot.enabled and robot.state not in {RobotState.IDLE, RobotState.TASK_COMPLETE, RobotState.UNREACHABLE} for robot in self.simulation.robots)
            tasks_pending = self.simulation.task_dispatcher is not None and any(task.status is not TaskStatus.COMPLETED for task in self.simulation.task_dispatcher.tasks.values())
            if active or tasks_pending or self._failure_at is not None and not self._failure_triggered:
                self.simulation.step()
            self._collect_events()

    def create_task(self, destination: Cell) -> dict[str, Any]:
        with self._lock:
            if not self.simulation.grid.is_walkable(destination) or destination in self.simulation.physical_obstacles:
                raise ValueError("Destination is not currently walkable.")
            dispatcher=self.simulation.task_dispatcher
            if dispatcher is None:
                dispatcher=DeterministicTaskDispatcher(); self.simulation.task_dispatcher=dispatcher
            candidates=[robot for robot in self.simulation.robots if robot.enabled and robot.state in {RobotState.IDLE,RobotState.TASK_COMPLETE}]
            if not candidates: raise RuntimeError("No idle robot is available for a new task.")
            self._task_number += 1; task=FleetTask(f"DASH-{self._task_number}",destination); dispatcher.add_task(task)
            self._emit({"type":"TASK_CREATED","task_id":task.task_id,"destination":self._cell(destination),"timestamp":self.simulation.tick_count})
            winner=min(candidates,key=lambda robot:(abs(robot.position.x-destination.x)+abs(robot.position.y-destination.y),robot.robot_id))
            dispatcher.assign_initial(task.task_id,winner)
            self._emit({"type":"TASK_ASSIGNED","task_id":task.task_id,"robot_id":winner.robot_id,"timestamp":self.simulation.tick_count})
            return self._task(task)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            metrics=self._metrics()
            return {"scenario":self.scenario_name,"time":self.simulation.tick_count,"grid":{"width":self.simulation.grid.width,"height":self.simulation.grid.height,"obstacles":[self._cell(c) for c in sorted(self.simulation.grid.obstacles)],"zones":[{"name":z.name,"left":z.left,"top":z.top,"width":z.width,"height":z.height} for z in self.zones],"dynamic_obstacles":[self._cell(c) for c in sorted(self.simulation.physical_obstacles)]},"robots":[self._robot(r) for r in self.simulation.robots],"tasks":[self._task(t) for t in self._tasks()],"metrics":metrics,"events":list(self._events[-150:])}

    def _robot(self, robot: Any) -> dict[str, Any]:
        task_id=next((task.task_id for task in self._tasks() if task.assigned_robot_id==robot.robot_id and task.status is not TaskStatus.COMPLETED),None)
        return {"robot_id":robot.robot_id,"pose":self._cell(robot.position),"state":robot.state.name,"goal":self._cell(robot.goal) if robot.goal else None,"task_id":task_id,"priority":getattr(robot.priority,"name",str(robot.priority)),"battery":robot.battery,"planned_path":[self._cell(c) for c in robot.path],"intent":[self._cell(step.cell) for step in robot.intent.steps] if robot.intent else [],"online":robot.enabled,"waiting_time":robot.waiting_ticks,"replan_count":robot.conflict_metrics.replans_succeeded}
    def _tasks(self): return tuple(self.simulation.task_dispatcher.tasks.values()) if self.simulation.task_dispatcher else ()
    def _task(self, task: FleetTask) -> dict[str, Any]: return {"task_id":task.task_id,"destination":self._cell(task.goal),"status":task.status.value,"assigned_robot_id":task.assigned_robot_id,"release_count":task.release_count}
    @staticmethod
    def _cell(cell: Cell) -> dict[str,int]: return {"x":cell.x,"y":cell.y}
    def _metrics(self) -> dict[str, int]:
        robots=self.simulation.robots; cm=[r.conflict_metrics for r in robots]; tasks=self._tasks()
        return {"active_robots":sum(r.enabled for r in robots),"offline_robots":sum(not r.enabled for r in robots),"tasks_created":len(tasks),"tasks_completed":sum(t.status is TaskStatus.COMPLETED for t in tasks),"active_tasks":sum(t.status not in {TaskStatus.COMPLETED,TaskStatus.PENDING} for t in tasks),"conflicts_detected":sum(m.predicted_conflicts for m in cm),"conflicts_resolved":sum(m.conflicts_cleared for m in cm),"deadlocks_detected":sum(m.deadlocks_detected for m in cm),"deadlocks_resolved":sum(m.deadlocks_resolved for m in cm),"replans":sum(m.replans_succeeded for m in cm),"safety_stops":0,"offline_events":sum(len(r.local_world_model.liveness_events) for r in robots),"task_reassignments":sum(1 for e in self.simulation.task_dispatcher.events if e.event_type.value=="TASK_REASSIGNED") if self.simulation.task_dispatcher else 0,"collisions":self.simulation.collision_count}
    def _emit(self,event:dict[str,Any]) -> None:
        key=tuple(sorted((k,str(v)) for k,v in event.items()))
        if key not in self._event_keys: self._event_keys.add(key); self._events.append(event)
    def _collect_events(self) -> None:
        for robot in self.simulation.robots:
            state=robot.state.name; previous=self._previous_states.get(robot.robot_id)
            if previous != state:
                event_type={"MOVING":"PATH_PLANNED","REPLANNING":"REPLAN_TRIGGERED","YIELDING":"ROBOT_YIELDING","TASK_COMPLETE":"TASK_COMPLETE"}.get(state)
                if event_type: self._emit({"type":event_type,"robot_id":robot.robot_id,"timestamp":self.simulation.tick_count})
                self._previous_states[robot.robot_id]=state
            metrics = robot.conflict_metrics
            previous_conflicts, previous_cleared = self._previous_conflicts.get(robot.robot_id, (0, 0))
            if metrics.predicted_conflicts > previous_conflicts:
                self._emit({"type":"CONFLICT_DETECTED", "robot_id":robot.robot_id, "timestamp":self.simulation.tick_count})
                for peer_id, winner_id in robot._winner_by_peer.items():
                    if winner_id is not None:
                        self._emit({"type":"RIGHT_OF_WAY_GRANTED", "robot_id":winner_id, "peer_robot_id":peer_id, "timestamp":self.simulation.tick_count})
            if metrics.conflicts_cleared > previous_cleared:
                self._emit({"type":"CONFLICT_RESOLVED", "robot_id":robot.robot_id, "timestamp":self.simulation.tick_count})
            self._previous_conflicts[robot.robot_id] = (metrics.predicted_conflicts, metrics.conflicts_cleared)
            for event in robot.local_world_model.liveness_events:
                self._emit({"type":event.event_type.value,"observer_robot_id":event.observer_robot_id,"offline_robot_id":event.peer_robot_id,"timestamp":event.timestamp})
            for event in robot.deadlock_events:
                self._emit({"type":event.event_type.value,"participants":list(event.participants),"timestamp":event.timestamp})
        if self.simulation.task_dispatcher:
            for event in self.simulation.task_dispatcher.events:
                self._emit({"type":event.event_type.value,"task_id":event.task_id,"robot_id":event.robot_id,"timestamp":event.timestamp})

def create_app(engine: DashboardEngine | None = None, auto_tick: bool = True) -> FastAPI:
    engine=engine or DashboardEngine()
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        task=None
        if auto_tick:
            async def loop():
                while True: engine.tick(); await asyncio.sleep(0.25)
            task=asyncio.create_task(loop())
        yield
        if task: task.cancel()
    app=FastAPI(title="SYNQ SIH26123 Dashboard",lifespan=lifespan)
    app.state.engine=engine
    static_dir=__import__("pathlib").Path(__file__).with_name("static")
    app.mount("/static",StaticFiles(directory=static_dir),name="static")
    @app.get("/")
    def index(): return FileResponse(static_dir / "index.html")
    @app.get("/api/status")
    def status(): return engine.snapshot()
    @app.get("/api/robots")
    def robots(): return engine.snapshot()["robots"]
    @app.get("/api/tasks")
    def tasks(): return engine.snapshot()["tasks"]
    @app.get("/api/metrics")
    def metrics(): return engine.snapshot()["metrics"]
    @app.get("/api/events")
    def events(): return engine.snapshot()["events"]
    @app.post("/api/tasks")
    def task(request: TaskRequest):
        try: return engine.create_task(Cell(request.destination_x,request.destination_y))
        except (ValueError,RuntimeError) as error: raise HTTPException(409, str(error))
    @app.post("/api/scenarios/{scenario_name}")
    def scenario(scenario_name:str):
        if scenario_name not in SCENARIOS: raise HTTPException(404,"Unknown scenario")
        engine.reset(scenario_name); return {"scenario":scenario_name}
    @app.post("/api/reset")
    def reset(): engine.reset("basic"); return {"scenario":"basic"}
    @app.websocket("/ws")
    async def websocket(websocket:WebSocket):
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(engine.snapshot()); await asyncio.sleep(0.25)
        except WebSocketDisconnect: pass
    return app

app=create_app()