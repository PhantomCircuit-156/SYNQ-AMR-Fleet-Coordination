"""Robot agent with local planning and decentralized conflict behavior."""
from __future__ import annotations
from dataclasses import dataclass, field
from coordination.conflict import Conflict, ConflictDetector, ConflictType
from coordination.corridor import CorridorProtocol
from coordination.deadlock import DeadlockEvent, DeadlockEventType, DeadlockManager
from coordination.intent import Intent, IntentManager
from coordination.local_world import LocalWorldModel
from coordination.resolution import ConflictResolution, ConflictResolver
from metrics.robot_conflicts import RobotConflictMetrics
from models.corridor import CorridorRegion, CorridorRequest, CorridorToken
from models.grid import Cell, WarehouseGrid
from models.messages import PeerStateMessage
from models.priority import TaskPriority
from models.robot_state import RobotState
from networking.peer_bus import InProcessPeerMessageBus
from planning.a_star import AStarPlanner

@dataclass(slots=True)
class RobotAgent:
    """A robot that owns planning, state, knowledge, and coordination decisions."""
    robot_id: str
    position: Cell
    planner: AStarPlanner = field(default_factory=AStarPlanner)
    state: RobotState = field(default=RobotState.IDLE, init=False)
    goal: Cell | None = field(default=None, init=False)
    path: tuple[Cell, ...] = field(default=(), init=False)
    velocity: tuple[int, int] = field(default=(0, 0), init=False)
    battery: float = field(default=100.0, init=False)
    priority: TaskPriority = field(default=TaskPriority.NORMAL, init=False)
    intent: Intent | None = field(default=None, init=False)
    waiting_ticks: int = field(default=0, init=False)
    safety_clear: bool = field(default=True, init=False)
    enabled: bool = field(default=True, init=False)
    local_world_model: LocalWorldModel = field(init=False)
    intent_manager: IntentManager = field(init=False)
    conflict_detector: ConflictDetector = field(init=False)
    conflict_resolver: ConflictResolver = field(init=False)
    corridor_protocol: CorridorProtocol = field(init=False)
    deadlock_manager: DeadlockManager = field(init=False)
    conflict_metrics: RobotConflictMetrics = field(default_factory=RobotConflictMetrics, init=False)
    corridor_requests: tuple[CorridorRequest, ...] = field(default=(), init=False)
    corridor_tokens: tuple[CorridorToken, ...] = field(default=(), init=False)
    _path_index: int = field(default=0, init=False)
    _last_timestamp: int = field(default=0, init=False)
    _pending_yield_destination: Cell | None = field(default=None, init=False)
    _active_conflict_signatures: set[tuple[object, ...]] = field(default_factory=set, init=False)
    _winner_by_peer: dict[str, str | None] = field(default_factory=dict, init=False)
    _resolution_reason_by_peer: dict[str, str] = field(default_factory=dict, init=False)
    _waiting_for_corridor: str | None = field(default=None, init=False)
    _corridor_owner_by_id: dict[str, str | None] = field(default_factory=dict, init=False)
    _blocked_by_peer_ids: tuple[str, ...] = field(default=(), init=False)
    _processed_deadlock_events: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        self.local_world_model = LocalWorldModel(self.robot_id)
        self.intent_manager = IntentManager()
        self.conflict_detector = ConflictDetector(self.robot_id)
        self.conflict_resolver = ConflictResolver(self.robot_id)
        self.corridor_protocol = CorridorProtocol(self.robot_id)
        self.deadlock_manager = DeadlockManager(self.robot_id)
        self._refresh_own_state(0)

    def disable(self) -> None:
        """Simulate loss of this robot; it stops moving and publishing."""
        self.enabled = False
        self.state = RobotState.OFFLINE
        self.velocity = (0, 0)
        self.intent = None
        self.corridor_requests = ()
        self.corridor_tokens = ()
        self._refresh_own_state(self._last_timestamp)

    def evaluate_peer_liveness(self, current_time: int) -> tuple[str, ...]:
        """Expire stale peers independently using this robot's local clock."""
        self._last_timestamp = current_time
        offline = self.local_world_model.expire_stale_peers(current_time)
        for peer_id in offline:
            self._winner_by_peer.pop(peer_id, None)
            self._resolution_reason_by_peer.pop(peer_id, None)
            self._active_conflict_signatures = {
                signature for signature in self._active_conflict_signatures
                if peer_id not in signature
            }
            for corridor_id, owner_id in tuple(self._corridor_owner_by_id.items()):
                if owner_id == peer_id:
                    del self._corridor_owner_by_id[corridor_id]
        return offline

    def observe_dynamic_obstacles(self, obstacles: set[Cell]) -> None:
        """Receive environment occupancy observations; route choice remains local."""
        self.local_world_model.known_dynamic_obstacles = set(obstacles)

    def _planning_grid(self) -> WarehouseGrid:
        static_map = self.local_world_model.static_map
        if static_map is None: raise RuntimeError("Robot has no static map knowledge.")
        return WarehouseGrid(static_map.width, static_map.height, frozenset(set(static_map.obstacles) | self.local_world_model.known_dynamic_obstacles))

    def _remaining_path_is_blocked(self) -> bool:
        return any(cell in self.local_world_model.known_dynamic_obstacles for cell in self.path[self._path_index + 1:])
    def attach_static_map(self, grid: WarehouseGrid) -> None: self.local_world_model.set_static_map(grid)
    def attach_corridor_regions(self, regions: tuple[CorridorRegion, ...]) -> None: self.local_world_model.set_corridor_regions(regions)

    def restore_waiting_snapshot(self, blocked_by: tuple[str, ...], waiting_ticks: int, timestamp: int = 0) -> None:
        if self.state is not RobotState.MOVING or not self.path: raise RuntimeError("A planned moving task is required before restoring WAITING.")
        if waiting_ticks < 0: raise ValueError("Waiting ticks cannot be negative.")
        self.state = RobotState.WAITING; self.velocity = (0, 0); self.waiting_ticks = waiting_ticks
        self._blocked_by_peer_ids = tuple(sorted(set(blocked_by))); self._last_timestamp = timestamp
        self._refresh_intent(timestamp); self._refresh_own_state(timestamp)

    def assign_goal(self, goal: Cell) -> None:
        if self.state not in {RobotState.IDLE, RobotState.TASK_COMPLETE}: raise RuntimeError(f"Robot {self.robot_id} cannot accept a goal while {self.state.name}.")
        self.goal = goal; self.path = (); self._path_index = 0; self.velocity = (0, 0); self.intent = None; self.waiting_ticks = 0
        self._winner_by_peer.clear(); self._resolution_reason_by_peer.clear(); self._active_conflict_signatures.clear(); self._waiting_for_corridor = None; self._blocked_by_peer_ids = (); self.corridor_requests = (); self.corridor_tokens = ()
        self.state = RobotState.PLANNING; self._refresh_own_state(self._last_timestamp)

    def decide_next_move(self, grid: WarehouseGrid | None = None) -> Cell | None:
        if not self.enabled: return None
        if self.local_world_model.static_map is None:
            if grid is None: raise RuntimeError("Robot has no static map knowledge.")
            self.attach_static_map(grid)
        if self.state in {RobotState.IDLE, RobotState.TASK_COMPLETE, RobotState.UNREACHABLE}: return None
        if self.goal is None: raise RuntimeError("A non-idle robot must have a goal.")
        if self.state in {RobotState.PLANNING, RobotState.REPLANNING}:
            replanning = self.state is RobotState.REPLANNING
            path = self.planner.plan(self._planning_grid(), self.position, self.goal)
            if path is None:
                self.state = RobotState.UNREACHABLE; self.velocity = (0, 0); self.conflict_metrics.no_route_stops += 1
            else:
                self.path = path; self._path_index = 0; self.state = RobotState.TASK_COMPLETE if len(path) == 1 else RobotState.MOVING
                if replanning: self.conflict_metrics.replans_succeeded += 1
            self._refresh_intent(self._last_timestamp); self._refresh_own_state(self._last_timestamp); return None
        if self._remaining_path_is_blocked():
            self.state = RobotState.REPLANNING; self.velocity = (0, 0); self.conflict_metrics.replans_requested += 1
            self._refresh_intent(self._last_timestamp); self._refresh_own_state(self._last_timestamp); return None
        for evaluator in (self._evaluate_deadlock_before_motion, self._evaluate_corridor_before_motion, self._evaluate_conflicts_before_motion):
            blocked, move = evaluator()
            if move is not None or blocked: return move
        return self.path[self._path_index + 1] if self.state is RobotState.MOVING else None

    def confirm_move(self, destination: Cell) -> None:
        if self.state is RobotState.YIELDING:
            if destination != self._pending_yield_destination: raise ValueError("Yield movement differs from the robot's local decision.")
            self.velocity = (destination.x-self.position.x, destination.y-self.position.y); self.position = destination
            self.path = (); self._path_index = 0; self._pending_yield_destination = None; self.state = RobotState.WAITING
            self._refresh_intent(self._last_timestamp); self._refresh_own_state(self._last_timestamp); return
        if self.state is not RobotState.MOVING: raise RuntimeError("A robot can only confirm movement while MOVING or YIELDING.")
        expected = self.path[self._path_index + 1]
        if destination != expected: raise ValueError(f"Unexpected move confirmation: expected {expected}, got {destination}")
        self.velocity = (destination.x-self.position.x, destination.y-self.position.y); self.position = destination; self._path_index += 1
        if self._path_index == len(self.path)-1: self.state = RobotState.TASK_COMPLETE
        self._refresh_intent(self._last_timestamp); self._refresh_own_state(self._last_timestamp)

    def publish_state(self, bus: InProcessPeerMessageBus, timestamp: int) -> PeerStateMessage:
        if not self.enabled: raise RuntimeError("An offline robot cannot publish a heartbeat.")
        self._last_timestamp = timestamp; self._refresh_intent(timestamp); self._refresh_corridor_advertisements(timestamp)
        message = self._build_state_message(timestamp); self.local_world_model.update_own_state(message); bus.publish(message); return message
    def receive_peer_messages(self, bus: InProcessPeerMessageBus) -> int: return sum(self.local_world_model.receive_peer_state(m) for m in bus.drain(self.robot_id))
    def detect_conflicts(self) -> tuple[Conflict, ...]: return self.conflict_detector.detect(self.local_world_model)
    def resolve_conflict(self, conflict: Conflict) -> ConflictResolution: return self.conflict_resolver.resolve_from_local_world(conflict, self.local_world_model)
    def known_winner_for(self, peer_id: str) -> str | None: return self._winner_by_peer.get(peer_id)
    def coordination_decisions(self) -> tuple[dict[str, str | None], ...]:
        """Expose this robot's cached local outcomes for read-only observability."""
        return tuple({"peer_robot_id": peer_id, "winner_robot_id": winner_id, "local_decision": "PROCEED" if winner_id == self.robot_id else "YIELD", "reason": self._resolution_reason_by_peer.get(peer_id, "UNKNOWN")} for peer_id, winner_id in sorted(self._winner_by_peer.items()))
    def active_conflict_markers(self) -> tuple[dict[str, object], ...]:
        """Expose normalized active conflicts without granting dashboard control."""
        return tuple({"conflict_type": signature[0].name, "robot_ids": signature[1], "cell": signature[2]} for signature in sorted(self._active_conflict_signatures, key=str))
    def known_corridor_owner_for(self, corridor_id: str) -> str | None: return self._corridor_owner_by_id.get(corridor_id)
    @property
    def deadlock_events(self) -> tuple[DeadlockEvent, ...]: return tuple(self.deadlock_manager.events)

    def _evaluate_deadlock_before_motion(self) -> tuple[bool, Cell | None]:
        decision = self.deadlock_manager.evaluate(self.local_world_model, self.local_world_model.corridor_regions, self._last_timestamp); self._sync_deadlock_metrics()
        if decision is None: return False, None
        peer = next(r for r in decision.participants if r != self.robot_id); self._winner_by_peer[peer] = decision.proceed_robot_id; self._blocked_by_peer_ids = ()
        if decision.backoff_robot_id == self.robot_id:
            destination = self._step_toward_passing_bay(decision.passing_bay)
            if destination is not None:
                self.state = RobotState.YIELDING; self._pending_yield_destination = destination; self.conflict_metrics.yield_events += 1; self._refresh_own_state(self._last_timestamp)
                return True, destination
            return True, None
        if self.state in {RobotState.WAITING, RobotState.YIELDING}: self._waiting_for_corridor = None; self._resume_after_conflict()
        return True, None

    def _evaluate_corridor_before_motion(self) -> tuple[bool, Cell | None]:
        corridor = self._relevant_corridor()
        if corridor is None: self.corridor_tokens = (); return False, None
        ownership = self.corridor_protocol.evaluate(corridor, self.local_world_model, self._last_timestamp); self._corridor_owner_by_id[corridor.corridor_id] = ownership.owner_id
        if ownership.owner_id == self.robot_id:
            had = any(t.corridor_id == corridor.corridor_id for t in self.corridor_tokens)
            self.corridor_tokens = (self.corridor_protocol.issue_or_refresh_token(corridor.corridor_id, self._last_timestamp),)
            if not had: self.conflict_metrics.corridor_tokens_acquired += 1
            if self._waiting_for_corridor == corridor.corridor_id:
                self._waiting_for_corridor = None; self._blocked_by_peer_ids = (); self._resume_after_conflict(); return True, None
            return False, None
        self.corridor_tokens = (); next_cell = self._next_path_cell()
        must_wait = self._waiting_for_corridor == corridor.corridor_id or (self.position not in corridor.cells and next_cell is not None and next_cell in corridor.cells)
        if not must_wait or self.position in corridor.cells: return False, None
        self._waiting_for_corridor = corridor.corridor_id; self._blocked_by_peer_ids = (ownership.owner_id,) if ownership.owner_id else (); self.waiting_ticks += 1
        self.conflict_metrics.blocked_ticks += 1; self.conflict_metrics.corridor_wait_ticks += 1
        destination = self._step_toward_nearest_bay(corridor)
        if destination is not None:
            self.state = RobotState.YIELDING; self._pending_yield_destination = destination; self.conflict_metrics.yield_events += 1; self._refresh_own_state(self._last_timestamp); return True, destination
        if self.state is not RobotState.WAITING: self.conflict_metrics.wait_events += 1
        self.state = RobotState.WAITING; self.velocity = (0, 0); self._refresh_intent(self._last_timestamp); self._refresh_own_state(self._last_timestamp); return True, None
    def _relevant_corridor(self) -> CorridorRegion | None:
        regions = self.local_world_model.corridor_regions
        if self._waiting_for_corridor is not None: return next((r for r in regions if r.corridor_id == self._waiting_for_corridor), None)
        nxt = self._next_path_cell(); remaining = set(self.path[self._path_index+1:]) if self.path else set()
        return next((r for r in regions if self.position in r.cells or nxt in r.cells or bool(remaining & r.cells)), None)
    def _next_path_cell(self) -> Cell | None: return self.path[self._path_index+1] if self.path and self._path_index+1 < len(self.path) else None
    def _step_toward_nearest_bay(self, corridor: CorridorRegion) -> Cell | None:
        if self.position in corridor.passing_bays or not corridor.passing_bays: return None
        bay = min(corridor.passing_bays, key=lambda c: (abs(c.x-self.position.x)+abs(c.y-self.position.y), c)); return self._step_toward_passing_bay(bay)
    def _step_toward_passing_bay(self, bay: Cell | None) -> Cell | None:
        grid = self.local_world_model.static_map
        if bay is None or grid is None or bay == self.position: return None
        route = self.planner.plan(self._planning_grid(), self.position, bay)
        if route is None or len(route) < 2: return None
        destination = route[1]
        return None if destination in {s.pose for s in self.local_world_model.peer_states.values()} else destination

    def _refresh_corridor_advertisements(self, timestamp: int) -> None:
        requests=[]; remaining=self.path[self._path_index+1:] if self.path else ()
        for corridor in self.local_world_model.corridor_regions:
            corridor_path=tuple(c for c in remaining if c in corridor.cells)
            if not (self._waiting_for_corridor == corridor.corridor_id or self.position in corridor.cells or corridor_path): continue
            if corridor_path: entry, exit_cell = corridor_path[0], corridor_path[-1]
            else:
                entry=min(sorted(corridor.cells),key=lambda c:(abs(c.x-self.position.x)+abs(c.y-self.position.y),c)); exit_cell=entry
            requests.append(CorridorRequest(corridor.corridor_id,self.robot_id,timestamp,entry,exit_cell))
        self.corridor_requests=tuple(sorted(requests,key=lambda r:r.corridor_id)); active={r.corridor_id for r in requests}
        self.corridor_tokens=tuple(t for t in self.corridor_tokens if t.corridor_id in active and t.is_valid_at(timestamp))

    def _is_explicit_corridor_conflict(self, conflict: Conflict) -> bool:
        cells={conflict.cell} if conflict.cell is not None else set()
        for transition in conflict.transitions: cells.update((transition.source,transition.destination))
        return any(bool(cells & corridor.cells) for corridor in self.local_world_model.corridor_regions)
    def _sync_deadlock_metrics(self) -> None:
        for event in self.deadlock_manager.events[self._processed_deadlock_events:]:
            if event.event_type is DeadlockEventType.DEADLOCK_DETECTED: self.conflict_metrics.deadlocks_detected += 1
            elif event.event_type is DeadlockEventType.DEADLOCK_RESOLUTION: self.conflict_metrics.deadlocks_resolved += 1
            elif event.event_type is DeadlockEventType.DEADLOCK_CLEARED: self.conflict_metrics.deadlocks_cleared += 1
        self._processed_deadlock_events=len(self.deadlock_manager.events)

    def _evaluate_conflicts_before_motion(self) -> tuple[bool, Cell | None]:
        conflicts=tuple(c for c in self.detect_conflicts() if not self._is_explicit_corridor_conflict(c)); signatures={self._conflict_signature(c) for c in conflicts}
        for conflict in conflicts:
            if self._conflict_signature(conflict) not in self._active_conflict_signatures: self.conflict_metrics.record_conflict(conflict.conflict_type.name)
        self.conflict_metrics.conflicts_cleared += len(self._active_conflict_signatures-signatures); self._active_conflict_signatures=signatures
        by_peer={}
        for conflict in conflicts:
            peer=next(r for r in conflict.robot_ids if r != self.robot_id); by_peer.setdefault(peer,[]).append(conflict)
        for peer in tuple(self._winner_by_peer):
            if peer not in by_peer: del self._winner_by_peer[peer]; self._resolution_reason_by_peer.pop(peer, None)
        losing=[]
        for peer in sorted(by_peer):
            if peer not in self._winner_by_peer:
                result=self.resolve_conflict(by_peer[peer][0]); self._winner_by_peer[peer]=result.winner_id; self._resolution_reason_by_peer[peer]=result.reason.name
                if result.winner_id == self.robot_id: self.conflict_metrics.conflicts_won += 1
                else: self.conflict_metrics.conflicts_lost += 1
            if self._winner_by_peer[peer] != self.robot_id: losing.extend(by_peer[peer])
        if losing:
            self._blocked_by_peer_ids=tuple(sorted({rid for c in losing for rid in c.robot_ids if rid != self.robot_id})); self.waiting_ticks += 1; self.conflict_metrics.blocked_ticks += 1
            side_step=any(c.conflict_type in {ConflictType.HEAD_ON,ConflictType.CORRIDOR} or c.cell == self.position for c in losing)
            if self.state in {RobotState.MOVING,RobotState.WAITING} and side_step:
                cell=self._choose_yield_cell()
                if cell is not None:
                    self.state=RobotState.YIELDING; self._pending_yield_destination=cell; self.conflict_metrics.yield_events += 1; self._refresh_own_state(self._last_timestamp); return True,cell
            if self.state is RobotState.MOVING: self.conflict_metrics.wait_events += 1
            self.state=RobotState.WAITING; self.velocity=(0,0); self._refresh_intent(self._last_timestamp); self._refresh_own_state(self._last_timestamp); return True,None
        if self.state in {RobotState.WAITING,RobotState.YIELDING}:
            self._blocked_by_peer_ids=(); self._resume_after_conflict(); return True,None
        return False,None

    def _choose_yield_cell(self) -> Cell | None:
        grid=self.local_world_model.static_map
        if grid is None:return None
        occupied={p.pose for p in self.local_world_model.peer_states.values()}; predicted={s.cell for i in self.local_world_model.peer_intents.values() if i for s in i.steps}; excluded=occupied|predicted|set(self.path[self._path_index+1:])
        return next((c for c in grid.neighbors(self.position) if c not in excluded),None)
    def _resume_after_conflict(self) -> None:
        self.state=RobotState.MOVING if self.path and self._path_index < len(self.path) and self.path[self._path_index] == self.position else RobotState.PLANNING
        self.velocity=(0,0); self.waiting_ticks=0; self.conflict_metrics.resume_events += 1; self._refresh_intent(self._last_timestamp); self._refresh_own_state(self._last_timestamp)
    @staticmethod
    def _conflict_signature(conflict: Conflict) -> tuple[object,...]: return (conflict.conflict_type,conflict.robot_ids,conflict.cell)
    def _refresh_intent(self,timestamp:int) -> None:
        if self.state in {RobotState.WAITING, RobotState.REPLANNING, RobotState.UNREACHABLE}:self.intent=self.intent_manager.generate_hold(self.robot_id,self.position,timestamp)
        elif self.path:self.intent=self.intent_manager.generate(self.robot_id,self.path,self._path_index,timestamp)
        else:self.intent=None
    def _refresh_own_state(self,timestamp:int)->None:self.local_world_model.update_own_state(self._build_state_message(timestamp))
    def _build_state_message(self,timestamp:int)->PeerStateMessage:
        return PeerStateMessage(robot_id=self.robot_id,timestamp=timestamp,pose=self.position,velocity=self.velocity,goal=self.goal,intent=self.intent,priority=self.priority,battery=self.battery,state=self.state,heartbeat=timestamp,waiting_ticks=self.waiting_ticks,safety_clear=self.safety_clear,corridor_requests=self.corridor_requests,corridor_tokens=self.corridor_tokens,blocked_by=self._blocked_by_peer_ids)