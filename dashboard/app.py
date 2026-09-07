"""FastAPI transport for the deterministic SYNQ demo controller."""
from __future__ import annotations
from contextlib import asynccontextmanager
import asyncio
from pathlib import Path
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from dashboard.controller import DemoController, SCENARIOS, SPEEDS
from models.grid import Cell
from models.priority import TaskPriority

DashboardEngine=DemoController
class TaskRequest(BaseModel):
    destination_x:int=Field(ge=0);destination_y:int=Field(ge=0);priority:str="NORMAL"
class CellRequest(BaseModel):
    x:int=Field(ge=0);y:int=Field(ge=0)
class SpeedRequest(BaseModel):speed:float

def create_app(engine:DemoController|None=None,auto_tick:bool=True)->FastAPI:
    engine=engine or DemoController()
    @asynccontextmanager
    async def lifespan(app:FastAPI):
        task=None
        if auto_tick:
            async def loop():
                while True:
                    engine.update();await asyncio.sleep(engine.tick_interval)
            task=asyncio.create_task(loop())
        yield
        if task:
            task.cancel()
            try:await task
            except asyncio.CancelledError:pass
    app=FastAPI(title="SYNQ SIH26123 Control Dashboard",lifespan=lifespan);app.state.engine=engine
    static=Path(__file__).with_name("static");app.mount("/static",StaticFiles(directory=static),name="static")
    @app.get("/")
    def index():return FileResponse(static/"index.html")
    @app.get("/api/status")
    def status():return engine.snapshot()
    @app.get("/api/robots")
    def robots():return engine.snapshot()["robots"]
    @app.get("/api/tasks")
    def tasks():return engine.snapshot()["tasks"]
    @app.get("/api/metrics")
    def metrics():return engine.snapshot()["metrics"]
    @app.get("/api/events")
    def events():return engine.snapshot()["events"]
    @app.post("/api/control/play")
    def play():engine.play();return {"run_state":engine.run_state.value}
    @app.post("/api/control/pause")
    def pause():engine.pause();return {"run_state":engine.run_state.value}
    @app.post("/api/control/step")
    def step():
        try:advanced=engine.step()
        except RuntimeError as error:raise HTTPException(409,str(error))
        return {"advanced":advanced,"time":engine.simulation.tick_count,"run_state":engine.run_state.value}
    @app.post("/api/control/speed")
    def speed(request:SpeedRequest):
        try:engine.set_speed(request.speed)
        except ValueError as error:raise HTTPException(422,str(error))
        return {"speed":engine.speed}
    @app.post("/api/tasks")
    def create_task(request:TaskRequest):
        try:priority=TaskPriority[request.priority.upper()];return engine.create_task(Cell(request.destination_x,request.destination_y),priority)
        except KeyError:raise HTTPException(422,"Priority must be NORMAL or HIGH.")
        except (ValueError,RuntimeError) as error:raise HTTPException(409,str(error))
    @app.post("/api/scenarios/{scenario_name}")
    def scenario(scenario_name:str):
        if scenario_name not in SCENARIOS:raise HTTPException(404,"Unknown scenario")
        engine.load_scenario(scenario_name);return {"scenario":scenario_name,"run_state":engine.run_state.value}
    @app.post("/api/scenarios/current/restart")
    def restart():engine.restart();return {"scenario":engine.scenario_name,"run_state":engine.run_state.value}
    @app.post("/api/control/next-scenario")
    def next_scenario_control():return {"scenario":engine.next_scenario(),"run_state":engine.run_state.value}
    @app.post("/api/scenarios/next")
    def next_scenario():return {"scenario":engine.next_scenario(),"run_state":engine.run_state.value}
    @app.post("/api/reset")
    def reset():engine.load_scenario("basic");return {"scenario":"basic","run_state":engine.run_state.value}
    @app.post("/api/robots/{robot_id}/fail")
    def fail_robot(robot_id:str):
        try:engine.fail_robot(robot_id)
        except KeyError:raise HTTPException(404,"Unknown robot")
        except ValueError as error:raise HTTPException(409,str(error))
        return {"robot_id":robot_id,"online":False}
    @app.post("/api/environment/obstacles")
    def add_obstacle(request:CellRequest):
        try:engine.add_obstacle(Cell(request.x,request.y))
        except ValueError as error:raise HTTPException(409,str(error))
        return {"cell":{"x":request.x,"y":request.y},"present":True}
    @app.delete("/api/environment/obstacles")
    def remove_obstacle(request:CellRequest):
        try:engine.remove_obstacle(Cell(request.x,request.y))
        except ValueError as error:raise HTTPException(409,str(error))
        return {"cell":{"x":request.x,"y":request.y},"present":False}
    @app.delete("/api/events")
    def clear_events():engine.clear_events();return {"events":0}
    @app.websocket("/ws")
    async def websocket(websocket:WebSocket):
        await websocket.accept()
        try:
            while True:await websocket.send_json(engine.snapshot());await asyncio.sleep(0.2)
        except WebSocketDisconnect:pass
    return app
app=create_app()