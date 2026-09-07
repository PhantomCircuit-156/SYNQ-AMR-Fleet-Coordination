"""Deterministic demo clock and read-only product telemetry adapter.

This controller creates missions and injects demo/environment inputs. It never
computes a path, conflict winner, yield, or corridor owner.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from threading import RLock
from typing import Any
from models.environment import EnvironmentObjectType
from models.grid import Cell
from models.mission import Payload, PayloadType, TaskType
from models.priority import TaskPriority
from models.robot_state import RobotState
from simulation.coordination_scenarios import build_head_on_scenario, build_three_robot_intersection_scenario, build_two_robot_intersection_scenario
from simulation.corridor_scenarios import build_opposing_corridor_scenario, build_recoverable_deadlock_scenario
from simulation.failure_scenarios import build_robot_failure_scenario
from simulation.full_demo import build_full_demo_scenario
from simulation.offline_blockage_scenarios import build_offline_robot_blockage_scenario
from simulation.scenarios import build_basic_three_robot_scenario
from tasks.lifecycle import DeterministicTaskDispatcher, FleetTask, TaskStatus

class RunState(str, Enum):
    RUNNING="RUNNING"; PAUSED="PAUSED"; STOPPED="STOPPED"; RESETTING="RESETTING"; SCENARIO_COMPLETE="SCENARIO_COMPLETE"
SPEEDS=(0.25,0.5,1.0,2.0,4.0)
SCENARIOS={
 "basic":{"name":"Basic Fleet","demonstrates":"Three payload-carrying AMRs completing non-conflicting warehouse missions.","watch":"Pickup dwell, visible loads, deterministic progress, and separate local plans."},
 "intersection":{"name":"Intersection Conflict","demonstrates":"Decentralized prediction at a shared crossing.","watch":"Both robots detect the overlap; the local deterministic loser waits, then resumes."},
 "head_on":{"name":"Head-On Conflict","demonstrates":"Local edge-swap detection and a deterministic side yield.","watch":"The yielding AMR moves to a safe side cell while the winner passes."},
 "three_robot_intersection":{"name":"Three-Robot Intersection","demonstrates":"Pairwise local agreement under a three-agent convergence.","watch":"Multiple local decisions serialize access without a central traffic brain."},
 "corridor":{"name":"Narrow Corridor","demonstrates":"Distributed leased corridor ownership.","watch":"Both robots compute the same owner; one waits outside until ownership clears."},
 "deadlock":{"name":"Deadlock Recovery","demonstrates":"Persistent mutual-block detection and deterministic backoff.","watch":"Each robot logs the same recovery outcome and the blocked pair clears."},
 "robot_failure":{"name":"Robot Failure","demonstrates":"Heartbeat timeout, task release, and deterministic reassignment.","watch":"Fail R2 after it starts; R1 and R3 independently time out R2."},
 "offline_robot_blockage":{"name":"Offline Robot Blockage","demonstrates":"A failed robot remains physical occupancy and triggers local replanning.","watch":"Fail R2; R1 routes around its stopped cell without central path selection."},
 "full_demo":{"name":"Full SIH Demo","demonstrates":"Payload work, local conflict resolution, obstruction, failure, and reassignment.","watch":"Follow the phase narration; every traffic decision still comes from each AMR."},
}

@dataclass(slots=True)
class DemoController:
    simulation:Any=field(init=False); scenario_name:str=field(default="basic",init=False); zones:tuple[Any,...]=field(default=(),init=False)
    run_state:RunState=field(default=RunState.PAUSED,init=False); speed:float=field(default=1.0,init=False)
    _lock:RLock=field(default_factory=RLock,init=False); _events:list[dict[str,Any]]=field(default_factory=list,init=False); _event_keys:set[tuple[Any,...]]=field(default_factory=set,init=False)
    _previous_states:dict[str,str]=field(default_factory=dict,init=False); _previous_metrics:dict[str,tuple[int,int,int]]=field(default_factory=dict,init=False)
    _previous_decisions:dict[str,tuple]=field(default_factory=dict,init=False); _previous_task_phases:dict[str,str]=field(default_factory=dict,init=False)
    _processed_liveness:dict[str,int]=field(default_factory=dict,init=False); _processed_deadlocks:dict[str,int]=field(default_factory=dict,init=False); _processed_task_events:int=field(default=0,init=False)
    _task_number:int=field(default=0,init=False); _scenario_revision:int=field(default=0,init=False); _full_demo_flags:set[str]=field(default_factory=set,init=False); _demo_phase:int=field(default=0,init=False)
    def __post_init__(self)->None:self.load_scenario("basic")
    @property
    def tick_interval(self)->float:return (1.0 if self.scenario_name=="full_demo" else 0.25)/self.speed
    def load_scenario(self,name:str)->None:
        if name not in SCENARIOS:raise ValueError(f"Unknown scenario: {name}")
        with self._lock:
            self.run_state=RunState.RESETTING;self.scenario_name=name;self._scenario_revision+=1;self._task_number=0
            self._events.clear();self._event_keys.clear();self._previous_states.clear();self._previous_metrics.clear();self._previous_decisions.clear();self._previous_task_phases.clear();self._processed_liveness.clear();self._processed_deadlocks.clear();self._processed_task_events=0;self._full_demo_flags.clear();self._demo_phase=1 if name=="full_demo" else 0
            if name=="basic": scenario,self.simulation=build_basic_three_robot_scenario();self.zones=scenario.zones
            elif name=="intersection":self.simulation=build_two_robot_intersection_scenario().simulation;self.zones=()
            elif name=="head_on":self.simulation=build_head_on_scenario().simulation;self.zones=()
            elif name=="three_robot_intersection":self.simulation=build_three_robot_intersection_scenario().simulation;self.zones=()
            elif name=="corridor":self.simulation=build_opposing_corridor_scenario();self.zones=()
            elif name=="deadlock":self.simulation=build_recoverable_deadlock_scenario();self.zones=()
            elif name=="robot_failure":self.simulation=build_robot_failure_scenario().simulation;self.zones=()
            elif name=="offline_robot_blockage":self.simulation=build_offline_robot_blockage_scenario().simulation;self.zones=()
            else: scenario=build_full_demo_scenario();self.simulation=scenario.simulation;self.zones=scenario.warehouse.zones
            self.run_state=RunState.PAUSED;self._emit("SCENARIO_LOADED","INFO",f"{SCENARIOS[name]['name']} ready; press Play or Step.");self._collect_events();self._update_demo_phase()
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
    def tick(self)->bool:return self.step()
    def update(self)->bool:
        with self._lock:return self._advance() if self.run_state is RunState.RUNNING else False
    def step(self)->bool:
        with self._lock:
            if self.run_state is RunState.RUNNING:raise RuntimeError("Pause before stepping one tick.")
            changed=self._advance()
            if self.run_state is not RunState.SCENARIO_COMPLETE:self.run_state=RunState.PAUSED
            return changed
    def advance_demo_phase(self)->int:
        with self._lock:
            if self.scenario_name!="full_demo":raise RuntimeError("Next Phase is available only in Full SIH Demo.")
            if self.run_state is RunState.RUNNING:raise RuntimeError("Pause before advancing to the next demo phase.")
            start=self._demo_phase
            for _ in range(120):
                if not self._advance() or self._demo_phase!=start:break
            return self._demo_phase
    def _advance(self)->bool:
        if not self._has_work():self.run_state=RunState.SCENARIO_COMPLETE;self._update_demo_phase();return False
        if self.scenario_name=="full_demo":self._apply_full_demo_inputs()
        before=self.simulation.tick_count;self.simulation.step();self._collect_events();self._update_demo_phase()
        if not self._has_work():self.run_state=RunState.SCENARIO_COMPLETE;self._update_demo_phase()
        return self.simulation.tick_count==before+1
    def _has_work(self)->bool:
        active=any(r.enabled and r.state not in {RobotState.IDLE,RobotState.TASK_COMPLETE,RobotState.UNREACHABLE} for r in self.simulation.robots)
        pending=any(t.status is not TaskStatus.COMPLETED for t in self._tasks())
        scripted=self.scenario_name=="full_demo" and "failure" not in self._full_demo_flags
        return active or pending or scripted
    def _apply_full_demo_inputs(self)->None:
        tick=self.simulation.tick_count
        if "expedite" not in self._full_demo_flags and tick>=18:
            r3=next(r for r in self.simulation.robots if r.robot_id=="R3")
            if r3.enabled and r3.state in {RobotState.IDLE,RobotState.TASK_COMPLETE}:
                payload=Payload("CARTON-X7",PayloadType.CARTON,20.0,fragile=True,priority=TaskPriority.HIGH)
                task=self._create_task_locked(Cell(36,22),TaskPriority.HIGH,TaskType.EXPEDITE,source=r3.position,payload=payload,allow_occupied_source=True,task_id="DEMO-HIGH")
                self._full_demo_flags.add("expedite");self._emit("HIGH_PRIORITY_MISSION","TASKS",f"{task.task_id} entered the workload queue.",task_id=task.task_id)
        if "expedite" in self._full_demo_flags and "obstacle" not in self._full_demo_flags and tick>=25:
            cell=Cell(24,22)
            if cell not in {r.position for r in self.simulation.robots}:
                self.simulation.add_dynamic_obstacle(cell,EnvironmentObjectType.PALLET);self._full_demo_flags.add("obstacle")
                self._emit("DYNAMIC_OBSTACLE_ADDED","SAFETY","A pallet blocks the active aisle; route choice remains robot-local.",cell=self._cell(cell),obstacle_type="PALLET")
        if "obstacle" in self._full_demo_flags and "failure" not in self._full_demo_flags and tick>=40:
            r2=next(r for r in self.simulation.robots if r.robot_id=="R2")
            if r2.enabled:
                self.simulation.disable_robot("R2");self._full_demo_flags.add("failure")
                self._emit("FAILURE_INJECTED","FAILURE","Deterministic demo fault disabled R2; peers must infer it from missed heartbeats.",robot_id="R2")
        dispatcher=self.simulation.task_dispatcher
        if dispatcher and any(e.event_type.value=="TASK_REASSIGNED" and e.task_id=="DEMO-102" for e in dispatcher.events):self._full_demo_flags.add("reassigned")
    def fail_robot(self,robot_id:str)->None:
        with self._lock:
            robot=next((r for r in self.simulation.robots if r.robot_id==robot_id),None)
            if robot is None:raise KeyError(robot_id)
            if not robot.enabled:raise ValueError(f"{robot_id} is already offline.")
            self.simulation.disable_robot(robot_id);self._emit("FAILURE_INJECTED","FAILURE",f"Operator disabled {robot_id}.",robot_id=robot_id)
    def add_obstacle(self,cell:Cell,obstacle_type:EnvironmentObjectType=EnvironmentObjectType.BOX)->None:
        with self._lock:
            self.simulation.add_dynamic_obstacle(cell,obstacle_type);self._emit("DYNAMIC_OBSTACLE_ADDED","SAFETY",f"{obstacle_type.value.replace('_',' ').title()} placed at ({cell.x},{cell.y}).",cell=self._cell(cell),obstacle_type=obstacle_type.value)
    def remove_obstacle(self,cell:Cell)->None:
        with self._lock:self.simulation.remove_dynamic_obstacle(cell);self._emit("DYNAMIC_OBSTACLE_REMOVED","INFO",f"Physical obstacle removed from ({cell.x},{cell.y}).",cell=self._cell(cell))
    def create_task(self,destination:Cell,priority:TaskPriority,task_type:TaskType=TaskType.TRANSFER,*,source:Cell|None=None,payload:Payload|None=None)->dict[str,Any]:
        with self._lock:
            task=self._create_task_locked(destination,priority,task_type,source=source,payload=payload);self.run_state=RunState.PAUSED;return self._task(task)
    def _create_task_locked(self,destination:Cell,priority:TaskPriority,task_type:TaskType,*,source:Cell|None,payload:Payload|None,allow_occupied_source:bool=False,task_id:str|None=None)->FleetTask:
        self._validate_waypoint(destination,"Destination",True)
        if source is not None:
            self._validate_waypoint(source,"Source",not allow_occupied_source)
            if source==destination:raise ValueError("Source and destination must be different cells.")
        dispatcher=self.simulation.task_dispatcher
        if dispatcher is None:dispatcher=DeterministicTaskDispatcher();self.simulation.task_dispatcher=dispatcher
        self._task_number+=1;task=FleetTask(task_id or f"DASH-{self._task_number}",destination,priority=priority,task_type=task_type,source=source,payload=payload);dispatcher.add_task(task)
        try:winner=dispatcher.assign_best(task.task_id,self.simulation.robots,self.simulation.tick_count)
        except Exception:dispatcher.tasks.pop(task.task_id,None);raise
        self._emit("TASK_CREATED","TASKS",f"{task.task_id} {task_type.value} mission created with {priority.name} priority.",task_id=task.task_id)
        self._emit("TASK_ASSIGNED","TASKS",f"{task.task_id} assigned to {winner.robot_id}: {task.assignment_reason[0]}.",task_id=task.task_id,robot_id=winner.robot_id)
        return task
    def _validate_waypoint(self,cell:Cell,label:str,reject_occupied:bool)->None:
        if not self.simulation.grid.in_bounds(cell):raise ValueError(f"{label} is outside the warehouse.")
        if not self.simulation.grid.is_walkable(cell):raise ValueError(f"{label} is a rack or wall cell.")
        if cell in self.simulation.physical_obstacles:raise ValueError(f"{label} is physically blocked.")
        if reject_occupied and cell in {r.position for r in self.simulation.robots}:raise ValueError(f"{label} is occupied by a robot.")
    def clear_events(self)->None:
        with self._lock:self._events.clear();self._event_keys.clear()
    def snapshot(self)->dict[str,Any]:
        with self._lock:
            conflicts={}
            for robot in self.simulation.robots:
                for m in robot.active_conflict_markers():
                    cell=self._cell(m["cell"]) if m["cell"] else None;key=(m["conflict_type"],tuple(m["robot_ids"]),str(cell))
                    conflicts[key]={"type":m["conflict_type"],"robot_ids":list(m["robot_ids"]),"cell":cell,"eta_ticks":m["eta_ticks"],"winner_robot_id":m["winner_robot_id"],"yielding_robot_id":m["yielding_robot_id"],"reason":m["reason"],"observer_robot_id":m["observer_robot_id"]}
            return {"scenario":self.scenario_name,"scenario_revision":self._scenario_revision,"scenario_info":SCENARIOS[self.scenario_name],"run_state":self.run_state.value,"speed":self.speed,"time":self.simulation.tick_count,"available_scenarios":[{"id":k,**v} for k,v in SCENARIOS.items()],"demo_phase":self._demo_phase_snapshot(),"grid":{"width":self.simulation.grid.width,"height":self.simulation.grid.height,"obstacles":[self._cell(c) for c in sorted(self.simulation.grid.obstacles)],"zones":[{"name":z.name,"left":z.left,"top":z.top,"width":z.width,"height":z.height} for z in self.zones],"dynamic_obstacles":[self._cell(c) for c in sorted(self.simulation.physical_obstacles)],"environment_objects":[{**self._cell(c),"object_type":o.value} for c,o in sorted(self.simulation.environment_objects.items())]},"robots":[self._robot(r) for r in self.simulation.robots],"tasks":[self._task(t) for t in self._tasks()],"metrics":self._metrics(),"events":list(self._events[-250:]),"conflicts":list(conflicts.values())}
    def _robot(self,r:Any)->dict[str,Any]:
        task=next((t for t in self._tasks() if t.assigned_robot_id==r.robot_id and t.status is not TaskStatus.COMPLETED),None);future=list(r.path)
        try:i=future.index(r.position);next_cell=future[i+1] if r.enabled and i+1<len(future) else None;remaining=len(future)-i-1
        except ValueError:next_cell=None;remaining=0
        payload=r.current_payload
        return {"robot_id":r.robot_id,"pose":self._cell(r.position),"state":r.state.name,"goal":self._cell(r.goal) if r.goal else None,"next_cell":self._cell(next_cell) if next_cell else None,"velocity":{"x":r.velocity[0],"y":r.velocity[1]},"remaining_path_length":remaining,"task_id":task.task_id if task else None,"mission":self._task(task) if task else None,"priority":getattr(r.priority,"name",str(r.priority)),"battery":r.battery,"low_battery":r.battery<20,"load_state":r.load_state.value,"payload":self._payload(payload),"max_payload_kg":r.max_payload_kg,"current_payload_kg":payload.weight_kg if payload else 0.0,"planned_path":[self._cell(c) for c in r.path],"intent":[{**self._cell(s.cell),"timestep":s.timestep} for s in r.intent.steps] if r.intent else [],"online":r.enabled,"waiting_time":r.waiting_ticks,"total_waiting_ticks":r.conflict_metrics.blocked_ticks,"replan_count":r.conflict_metrics.replans_succeeded,"decisions":list(r.coordination_decisions()),"corridor_ownership":list(r.corridor_ownership()),"offline_peers":sorted(r.local_world_model.offline_peers)}
    def _tasks(self)->tuple[FleetTask,...]:return tuple(self.simulation.task_dispatcher.tasks.values()) if self.simulation.task_dispatcher else ()
    def _task(self,t:FleetTask)->dict[str,Any]:
        robot=next((r for r in self.simulation.robots if r.robot_id==t.assigned_robot_id),None)
        return {"task_id":t.task_id,"task_type":t.task_type.value,"source":self._cell(t.source) if t.source else None,"destination":self._cell(t.goal),"status":t.status.value,"phase":t.phase.value,"assigned_robot_id":t.assigned_robot_id,"priority":t.priority.name,"payload":self._payload(t.payload),"release_count":t.release_count,"progress":t.progress_percent(robot),"eta_ticks":t.eta_ticks(robot),"assignment_reason":list(t.assignment_reason),"allocation_cost":t.allocation_cost}
    @staticmethod
    def _payload(p:Payload|None)->dict[str,Any]|None:return None if p is None else {"payload_id":p.payload_id,"payload_type":p.payload_type.value,"weight_kg":p.weight_kg,"fragile":p.fragile,"priority":p.priority.name}
    def _metrics(self)->dict[str,Any]:
        robots=self.simulation.robots;m=[r.conflict_metrics for r in robots];tasks=self._tasks();durations=[t.completed_at-t.assigned_at for t in tasks if t.completed_at is not None and t.assigned_at is not None];online=[r for r in robots if r.enabled];busy=[r for r in online if r.state not in {RobotState.IDLE,RobotState.TASK_COMPLETE,RobotState.UNREACHABLE}]
        return {"active_robots":len(online),"fleet_availability_percent":round(100*len(online)/len(robots)) if robots else 0,"fleet_utilization_percent":round(100*len(busy)/len(online)) if online else 0,"tasks_completed":sum(t.status is TaskStatus.COMPLETED for t in tasks),"average_task_time_ticks":round(sum(durations)/len(durations),1) if durations else None,"conflicts":sum(x.predicted_conflicts for x in m),"conflicts_resolved":sum(x.conflicts_cleared for x in m),"total_waiting_ticks":sum(x.blocked_ticks for x in m),"replans":sum(x.replans_succeeded for x in m),"deadlocks":sum(x.deadlocks_detected for x in m),"deadlocks_recovered":sum(x.deadlocks_cleared for x in m),"safety_stops":sum(getattr(x,"safety_stops",0) for x in m),"offline_events":sum(1 for r in robots for e in r.local_world_model.liveness_events if e.event_type.value=="ROBOT_OFFLINE"),"collisions":self.simulation.collision_count}
    @staticmethod
    def _cell(c:Cell)->dict[str,int]:return {"x":c.x,"y":c.y}
    def _emit(self,event_type:str,category:str,message:str,**details:Any)->None:
        event={"type":event_type,"category":category,"message":message,"timestamp":self.simulation.tick_count,**details};key=tuple(sorted((k,str(v)) for k,v in event.items()))
        if key not in self._event_keys:self._event_keys.add(key);self._events.append(event)
    def _collect_events(self)->None:
        for r in self.simulation.robots:
            state=r.state.name;previous=self._previous_states.get(r.robot_id)
            if state!=previous:
                mapping={"MOVING":("PATH_PLANNED","INFO","Path planned; movement ready."),"WAITING":("ROBOT_WAITING","COORDINATION","Waiting for locally resolved right-of-way."),"YIELDING":("ROBOT_YIELDING","COORDINATION","Robot selected a local yield move."),"REPLANNING":("REPLAN_TRIGGERED","SAFETY","Observed occupancy invalidated remaining path."),"TASK_COMPLETE":("WAYPOINT_REACHED","TASKS","Mission waypoint reached."),"OFFLINE":("ROBOT_DISABLED","FAILURE","Robot is physically stationary and offline.")}
                if state in mapping:self._emit(*mapping[state],robot_id=r.robot_id)
                if previous in {"WAITING","YIELDING"} and state=="MOVING":self._emit("ROBOT_RESUMED","COORDINATION","Local conflict cleared; robot resumed.",robot_id=r.robot_id)
                self._previous_states[r.robot_id]=state
            values=(r.conflict_metrics.predicted_conflicts,r.conflict_metrics.conflicts_cleared,r.conflict_metrics.replans_succeeded);old=self._previous_metrics.get(r.robot_id,(0,0,0))
            if values[0]>old[0]:self._emit("CONFLICT_DETECTED","COORDINATION","Predicted occupancy overlaps a peer intent.",robot_id=r.robot_id)
            if values[1]>old[1]:self._emit("CONFLICT_RESOLVED","COORDINATION","Local conflict no longer exists.",robot_id=r.robot_id)
            if values[2]>old[2]:self._emit("REPLAN_COMPLETE","SAFETY","A new route avoids observed physical occupancy.",robot_id=r.robot_id)
            self._previous_metrics[r.robot_id]=values
            decisions=tuple(tuple(sorted(d.items())) for d in r.coordination_decisions())
            if decisions!=self._previous_decisions.get(r.robot_id,()):
                for d in r.coordination_decisions():self._emit("LOCAL_DECISION","COORDINATION",f"{r.robot_id}: {d['local_decision']} — {d['reason']}.",robot_id=r.robot_id,**d)
                self._previous_decisions[r.robot_id]=decisions
            start=self._processed_liveness.get(r.robot_id,0);events=r.local_world_model.liveness_events
            for e in events[start:]:self._emit(e.event_type.value,"FAILURE",f"{e.observer_robot_id} observed {e.peer_robot_id} heartbeat timeout.",observer_robot_id=e.observer_robot_id,offline_robot_id=e.peer_robot_id)
            self._processed_liveness[r.robot_id]=len(events);start=self._processed_deadlocks.get(r.robot_id,0);deadlocks=r.deadlock_events
            for e in deadlocks[start:]:self._emit(e.event_type.value,"COORDINATION",e.detail,participants=list(e.participants))
            self._processed_deadlocks[r.robot_id]=len(deadlocks)
        for t in self._tasks():
            phase=t.phase.value;previous=self._previous_task_phases.get(t.task_id)
            if phase!=previous:
                mapping={"PICKUP_DWELL":("PAYLOAD_PICKUP_STARTED",f"{t.task_id} pickup dwell started."),"TO_DROPOFF":("PAYLOAD_LOADED",f"{t.task_id} payload loaded; AMR is travelling to dropoff."),"DROPOFF_DWELL":("PAYLOAD_DROPOFF_STARTED",f"{t.task_id} dropoff dwell started."),"COMPLETE":("TASK_COMPLETE",f"{t.task_id} completed and payload unloaded.")}
                if phase in mapping:self._emit(mapping[phase][0],"TASKS",mapping[phase][1],task_id=t.task_id,robot_id=t.assigned_robot_id)
                self._previous_task_phases[t.task_id]=phase
        dispatcher=self.simulation.task_dispatcher
        if dispatcher:
            for e in dispatcher.events[self._processed_task_events:]:self._emit(e.event_type.value,"TASKS",f"{e.task_id}: {e.event_type.value.replace('_',' ').lower()}.",task_id=e.task_id,robot_id=e.robot_id)
            self._processed_task_events=len(dispatcher.events)
    def _update_demo_phase(self)->None:
        if self.scenario_name!="full_demo":self._demo_phase=0;return
        tick=self.simulation.tick_count;metrics=self._metrics()
        if self.run_state is RunState.SCENARIO_COMPLETE or ("reassigned" in self._full_demo_flags and all(t.status is TaskStatus.COMPLETED for t in self._tasks())):self._demo_phase=8
        elif "failure" in self._full_demo_flags:self._demo_phase=7
        elif "obstacle" in self._full_demo_flags:self._demo_phase=6
        elif "expedite" in self._full_demo_flags:self._demo_phase=5
        elif metrics["conflicts_resolved"]:self._demo_phase=4
        elif metrics["conflicts"]:self._demo_phase=3
        elif tick>0:self._demo_phase=2
        else:self._demo_phase=1
    def _demo_phase_snapshot(self)->dict[str,Any]|None:
        if self.scenario_name!="full_demo":return None
        content={1:("MISSIONS INITIALIZED","Three independent AMRs receive deterministic warehouse work."),2:("INTENT SHARING","Each AMR broadcasts bounded future occupancy to its peers."),3:("DECENTRALIZED CONFLICT RESOLUTION","Both robots detect future overlap and independently derive the same winner."),4:("WINNER PASSES / LOSER RESUMES","The locally yielding AMR resumes after the conflict clears."),5:("HIGH-PRIORITY PAYLOAD","A capacity-eligible AMR receives an expedite mission; traffic safety rules are unchanged."),6:("DYNAMIC REPLANNING","A pallet blocks the remaining path; the affected AMR computes its own new route."),7:("FAILURE AND REASSIGNMENT","Peers infer R2's timeout locally; unfinished work is released and deterministically reassigned."),8:("DEMO COMPLETE","Remaining AMRs completed the implemented workflow with zero collisions.")};title,narration=content[self._demo_phase];return {"current":self._demo_phase,"total":8,"title":title,"narration":narration}
