"""Deterministic demo-clock and read-only visualization adapter."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from threading import RLock
from typing import Any
from models.grid import Cell
from models.priority import TaskPriority
from models.robot_state import RobotState
from simulation.coordination_scenarios import build_head_on_scenario, build_three_robot_intersection_scenario, build_two_robot_intersection_scenario
from simulation.corridor_scenarios import build_opposing_corridor_scenario, build_recoverable_deadlock_scenario
from simulation.failure_scenarios import build_robot_failure_scenario
from simulation.offline_blockage_scenarios import build_offline_robot_blockage_scenario
from simulation.scenarios import build_basic_three_robot_scenario
from tasks.lifecycle import DeterministicTaskDispatcher, FleetTask, TaskStatus

class RunState(str, Enum):
    RUNNING="RUNNING"; PAUSED="PAUSED"; STOPPED="STOPPED"; RESETTING="RESETTING"; SCENARIO_COMPLETE="SCENARIO_COMPLETE"
SPEEDS=(0.25,0.5,1.0,2.0,4.0)
SCENARIOS={
 "basic":{"name":"Basic Fleet","demonstrates":"Three independent AMRs completing non-conflicting work.","watch":"All three agents plan and move on separate routes."},
 "intersection":{"name":"Intersection Conflict","demonstrates":"Decentralized prediction at a shared crossing.","watch":"Both robots detect the overlap; the local deterministic loser waits, then resumes."},
 "head_on":{"name":"Head-On Conflict","demonstrates":"Local edge-swap detection and a deterministic side yield.","watch":"The yielding AMR moves to a safe side cell while the winner passes."},
 "three_robot_intersection":{"name":"Three-Robot Intersection","demonstrates":"Pairwise local agreement under a three-agent convergence.","watch":"Multiple local decisions serialize access without a central traffic brain."},
 "corridor":{"name":"Narrow Corridor","demonstrates":"Distributed leased corridor ownership.","watch":"Both robots compute the same owner; one waits outside until ownership clears."},
 "deadlock":{"name":"Deadlock Recovery","demonstrates":"Persistent mutual-block detection and deterministic backoff.","watch":"Each robot logs the same recovery outcome and the blocked pair clears."},
 "robot_failure":{"name":"Robot Failure","demonstrates":"Heartbeat timeout, task release, and deterministic reassignment.","watch":"Press Fail R2 after it starts; R1 and R3 independently time out R2."},
 "offline_robot_blockage":{"name":"Offline Robot Blockage","demonstrates":"A failed robot remains physical occupancy and triggers local replanning.","watch":"Press Fail R2; R1 routes around its stopped cell without central path selection."},
}

@dataclass(slots=True)
class DemoController:
    simulation: Any=field(init=False); scenario_name:str=field(default="basic",init=False); zones:tuple[Any,...]=field(default=(),init=False)
    run_state:RunState=field(default=RunState.PAUSED,init=False); speed:float=field(default=1.0,init=False)
    _lock:RLock=field(default_factory=RLock,init=False); _events:list[dict[str,Any]]=field(default_factory=list,init=False); _event_keys:set[tuple[Any,...]]=field(default_factory=set,init=False)
    _previous_states:dict[str,str]=field(default_factory=dict,init=False); _previous_metrics:dict[str,tuple[int,int,int]]=field(default_factory=dict,init=False); _task_number:int=field(default=0,init=False)
    def __post_init__(self)->None:self.load_scenario("basic")
    @property
    def tick_interval(self)->float:return 0.25/self.speed
    def load_scenario(self,name:str)->None:
        if name not in SCENARIOS:raise ValueError(f"Unknown scenario: {name}")
        with self._lock:
            self.run_state=RunState.RESETTING;self.scenario_name=name;self._events=[];self._event_keys=set();self._previous_states={};self._previous_metrics={}
            if name=="basic": scenario,self.simulation=build_basic_three_robot_scenario();self.zones=scenario.zones
            elif name=="intersection":self.simulation=build_two_robot_intersection_scenario().simulation;self.zones=()
            elif name=="head_on":self.simulation=build_head_on_scenario().simulation;self.zones=()
            elif name=="three_robot_intersection":self.simulation=build_three_robot_intersection_scenario().simulation;self.zones=()
            elif name=="corridor":self.simulation=build_opposing_corridor_scenario();self.zones=()
            elif name=="deadlock":self.simulation=build_recoverable_deadlock_scenario();self.zones=()
            elif name=="robot_failure":self.simulation=build_robot_failure_scenario().simulation;self.zones=()
            else:self.simulation=build_offline_robot_blockage_scenario().simulation;self.zones=()
            self.run_state=RunState.PAUSED;self._emit("SCENARIO_LOADED","INFO",f"{SCENARIOS[name]['name']} ready; press Play or Step.")
            self._collect_events()
    def reset(self,name:str="basic")->None:self.load_scenario(name)
    def restart(self)->None:self.load_scenario(self.scenario_name)
    def next_scenario(self)->str:
        names=tuple(SCENARIOS);name=names[(names.index(self.scenario_name)+1)%len(names)];self.load_scenario(name);return name
    def play(self)->None:
        with self._lock:
            if self.run_state is RunState.SCENARIO_COMPLETE:self.restart()
            self.run_state=RunState.RUNNING
    def pause(self)->None:
        with self._lock:self.run_state=RunState.PAUSED
    def stop(self)->None:
        with self._lock:self.run_state=RunState.STOPPED
    def set_speed(self,speed:float)->None:
        if speed not in SPEEDS:raise ValueError("Unsupported simulation speed.")
        with self._lock:self.speed=speed
    def tick(self)->bool:
        """Compatibility helper: advance exactly one deterministic tick."""
        return self.step()
    def update(self)->bool:
        with self._lock:
            if self.run_state is not RunState.RUNNING:return False
            return self._advance()
    def step(self)->bool:
        with self._lock:
            if self.run_state is RunState.RUNNING:raise RuntimeError("Pause before stepping one tick.")
            changed=self._advance()
            if self.run_state is not RunState.SCENARIO_COMPLETE:self.run_state=RunState.PAUSED
            return changed
    def _advance(self)->bool:
        if not self._has_work():self.run_state=RunState.SCENARIO_COMPLETE;return False
        before=self.simulation.tick_count;self.simulation.step();self._collect_events()
        if not self._has_work():self.run_state=RunState.SCENARIO_COMPLETE
        return self.simulation.tick_count==before+1
    def _has_work(self)->bool:
        active=any(r.enabled and r.state not in {RobotState.IDLE,RobotState.TASK_COMPLETE,RobotState.UNREACHABLE} for r in self.simulation.robots)
        tasks=self._tasks();pending=bool(tasks) and any(t.status is not TaskStatus.COMPLETED for t in tasks)
        return active or pending
    def fail_robot(self,robot_id:str)->None:
        with self._lock:
            robot=next((r for r in self.simulation.robots if r.robot_id==robot_id),None)
            if robot is None:raise KeyError(robot_id)
            if not robot.enabled:raise ValueError(f"{robot_id} is already offline.")
            self.simulation.disable_robot(robot_id);self._emit("FAILURE_INJECTED","FAILURE",f"Operator disabled {robot_id}.",robot_id=robot_id)
    def add_obstacle(self,cell:Cell)->None:
        with self._lock:self.simulation.add_dynamic_obstacle(cell);self._emit("DYNAMIC_OBSTACLE_ADDED","SAFETY",f"Physical obstacle placed at ({cell.x},{cell.y}).",cell=self._cell(cell))
    def remove_obstacle(self,cell:Cell)->None:
        with self._lock:self.simulation.remove_dynamic_obstacle(cell);self._emit("DYNAMIC_OBSTACLE_REMOVED","INFO",f"Physical obstacle removed from ({cell.x},{cell.y}).",cell=self._cell(cell))
    def create_task(self,destination:Cell,priority:TaskPriority)->dict[str,Any]:
        with self._lock:
            if not self.simulation.grid.is_walkable(destination) or destination in self.simulation.physical_obstacles:raise ValueError("Destination is not currently walkable.")
            dispatcher=self.simulation.task_dispatcher
            if dispatcher is None:dispatcher=DeterministicTaskDispatcher();self.simulation.task_dispatcher=dispatcher
            candidates=[r for r in self.simulation.robots if r.enabled and r.state in {RobotState.IDLE,RobotState.TASK_COMPLETE}]
            if not candidates:raise RuntimeError("No idle robot is available for a new task.")
            self._task_number+=1;task=FleetTask(f"DASH-{self._task_number}",destination,priority=priority);dispatcher.add_task(task)
            winner=min(candidates,key=lambda r:(abs(r.position.x-destination.x)+abs(r.position.y-destination.y),r.robot_id));dispatcher.assign_initial(task.task_id,winner)
            self._emit("TASK_CREATED","TASKS",f"{task.task_id} created with {priority.name} priority.",task_id=task.task_id)
            self._emit("TASK_ASSIGNED","TASKS",f"{task.task_id} assigned to {winner.robot_id}.",task_id=task.task_id,robot_id=winner.robot_id)
            self.run_state=RunState.PAUSED
            return self._task(task)
    def clear_events(self)->None:
        with self._lock:self._events=[];self._event_keys=set()
    def snapshot(self)->dict[str,Any]:
        with self._lock:
            conflicts=[]
            for robot in self.simulation.robots:
                for marker in robot.active_conflict_markers():
                    item={"type":marker["conflict_type"],"robot_ids":list(marker["robot_ids"]),"cell":self._cell(marker["cell"]) if marker["cell"] else None}
                    if item not in conflicts:conflicts.append(item)
            return {"scenario":self.scenario_name,"scenario_info":SCENARIOS[self.scenario_name],"run_state":self.run_state.value,"speed":self.speed,"time":self.simulation.tick_count,"available_scenarios":[{"id":k,**v} for k,v in SCENARIOS.items()],"grid":{"width":self.simulation.grid.width,"height":self.simulation.grid.height,"obstacles":[self._cell(c) for c in sorted(self.simulation.grid.obstacles)],"zones":[{"name":z.name,"left":z.left,"top":z.top,"width":z.width,"height":z.height} for z in self.zones],"dynamic_obstacles":[self._cell(c) for c in sorted(self.simulation.physical_obstacles)]},"robots":[self._robot(r) for r in self.simulation.robots],"tasks":[self._task(t) for t in self._tasks()],"metrics":self._metrics(),"events":list(self._events[-250:]),"conflicts":conflicts}
    def _robot(self,r:Any)->dict[str,Any]:
        task=next((t for t in self._tasks() if t.assigned_robot_id==r.robot_id and t.status is not TaskStatus.COMPLETED),None)
        return {"robot_id":r.robot_id,"pose":self._cell(r.position),"state":r.state.name,"goal":self._cell(r.goal) if r.goal else None,"task_id":task.task_id if task else None,"priority":getattr(r.priority,"name",str(r.priority)),"battery":r.battery,"planned_path":[self._cell(c) for c in r.path],"intent":[self._cell(s.cell) for s in r.intent.steps] if r.intent else [],"online":r.enabled,"waiting_time":r.waiting_ticks,"replan_count":r.conflict_metrics.replans_succeeded,"decisions":list(r.coordination_decisions())}
    def _tasks(self):return tuple(self.simulation.task_dispatcher.tasks.values()) if self.simulation.task_dispatcher else ()
    def _task(self,t:FleetTask)->dict[str,Any]:
        progress=100 if t.status is TaskStatus.COMPLETED else (25 if t.status is TaskStatus.IN_PROGRESS else 0)
        return {"task_id":t.task_id,"destination":self._cell(t.goal),"status":t.status.value,"assigned_robot_id":t.assigned_robot_id,"priority":t.priority.name,"release_count":t.release_count,"progress":progress}
    def _metrics(self)->dict[str,int]:
        robots=self.simulation.robots;m=[r.conflict_metrics for r in robots];tasks=self._tasks()
        return {"active_robots":sum(r.enabled for r in robots),"tasks_completed":sum(t.status is TaskStatus.COMPLETED for t in tasks),"conflicts":sum(x.predicted_conflicts for x in m),"replans":sum(x.replans_succeeded for x in m),"deadlocks":sum(x.deadlocks_detected for x in m),"safety_stops":sum(getattr(x,"safety_stops",0) for x in m),"offline_events":sum(1 for r in robots for e in r.local_world_model.liveness_events if e.event_type.value=="ROBOT_OFFLINE"),"collisions":self.simulation.collision_count}
    @staticmethod
    def _cell(c:Cell)->dict[str,int]:return {"x":c.x,"y":c.y}
    def _emit(self,event_type:str,category:str,message:str,**details:Any)->None:
        event={"type":event_type,"category":category,"message":message,"timestamp":self.simulation.tick_count,**details};key=tuple(sorted((k,str(v)) for k,v in event.items()))
        if key not in self._event_keys:self._event_keys.add(key);self._events.append(event)
    def _collect_events(self)->None:
        for r in self.simulation.robots:
            state=r.state.name;previous=self._previous_states.get(r.robot_id)
            if state!=previous:
                mapping={"MOVING":("PATH_PLANNED","INFO","Path planned; movement ready."),"WAITING":("ROBOT_WAITING","COORDINATION","Waiting for locally resolved right-of-way."),"YIELDING":("ROBOT_YIELDING","COORDINATION","Robot selected a local yield move."),"REPLANNING":("REPLAN_TRIGGERED","SAFETY","Observed occupancy invalidated remaining path."),"TASK_COMPLETE":("TASK_COMPLETE","TASKS","Destination reached."),"OFFLINE":("ROBOT_DISABLED","FAILURE","Robot is physically stationary and offline.")}
                if state in mapping:self._emit(*mapping[state],robot_id=r.robot_id)
                if previous in {"WAITING","YIELDING"} and state=="MOVING":self._emit("ROBOT_RESUMED","COORDINATION","Local conflict cleared; robot resumed.",robot_id=r.robot_id)
                self._previous_states[r.robot_id]=state
            values=(r.conflict_metrics.predicted_conflicts,r.conflict_metrics.conflicts_cleared,r.conflict_metrics.replans_succeeded);old=self._previous_metrics.get(r.robot_id,(0,0,0))
            if values[0]>old[0]:self._emit("CONFLICT_DETECTED","COORDINATION","Predicted occupancy overlaps a peer intent.",robot_id=r.robot_id)
            if values[1]>old[1]:self._emit("CONFLICT_RESOLVED","COORDINATION","Local conflict no longer exists.",robot_id=r.robot_id)
            if values[2]>old[2]:self._emit("REPLAN_COMPLETE","SAFETY","A new route avoids observed physical occupancy.",robot_id=r.robot_id)
            self._previous_metrics[r.robot_id]=values
            for decision in r.coordination_decisions():self._emit("LOCAL_DECISION","COORDINATION",f"{r.robot_id}: {decision['local_decision']} — {decision['reason']}.",robot_id=r.robot_id,**decision)
            for e in r.local_world_model.liveness_events:self._emit(e.event_type.value,"FAILURE",f"{e.observer_robot_id} observed {e.peer_robot_id} heartbeat timeout.",observer_robot_id=e.observer_robot_id,offline_robot_id=e.peer_robot_id)
            for e in r.deadlock_events:self._emit(e.event_type.value,"COORDINATION",e.detail,participants=list(e.participants))
        if self.simulation.task_dispatcher:
            for e in self.simulation.task_dispatcher.events:self._emit(e.event_type.value,"TASKS",f"{e.task_id}: {e.event_type.value.replace('_',' ').lower()}.",task_id=e.task_id,robot_id=e.robot_id)