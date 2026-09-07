from pathlib import Path
from fastapi.testclient import TestClient
from dashboard.app import DashboardEngine, create_app
from dashboard.controller import RunState

def client_and_engine():
    engine=DashboardEngine();return TestClient(create_app(engine,auto_tick=False)),engine

def positions(engine):return [(r.robot_id,r.position.x,r.position.y,r.state.name) for r in engine.simulation.robots]

def test_dashboard_status_robots_metrics_events_and_run_state():
    client,engine=client_and_engine()
    with client:
        status=client.get('/api/status').json()
        assert status['run_state']=='PAUSED' and status['speed']==1.0
        assert {'scenario_info','available_scenarios','conflicts'}<=status.keys()
        assert {r['robot_id'] for r in client.get('/api/robots').json()}=={'R1','R2','R3'}
        assert client.get('/api/metrics').json()['collisions']==0
        assert isinstance(client.get('/api/events').json(),list)

def test_pause_play_resume_and_single_step_are_deterministic():
    client,engine=client_and_engine();initial=positions(engine)
    with client:
        assert client.post('/api/control/pause').json()['run_state']=='PAUSED'
        engine.update();assert engine.simulation.tick_count==0 and positions(engine)==initial
        assert client.post('/api/control/step').json()['time']==1
        assert engine.simulation.tick_count==1
        client.post('/api/control/play');engine.update();assert engine.simulation.tick_count==2
        client.post('/api/control/pause');frozen=positions(engine);engine.update();assert positions(engine)==frozen and engine.simulation.tick_count==2

def test_speed_controls_accept_only_supported_values():
    client,engine=client_and_engine()
    with client:
        for speed in (.25,.5,1,2,4):assert client.post('/api/control/speed',json={'speed':speed}).json()['speed']==speed
        assert client.post('/api/control/speed',json={'speed':3}).status_code==422

def test_scenario_load_restart_next_and_reset_are_paused_and_repeatable():
    client,engine=client_and_engine()
    with client:
        first=client.post('/api/scenarios/head_on').json();initial=positions(engine)
        assert first=={'scenario':'head_on','run_state':'PAUSED'}
        engine.step();client.post('/api/scenarios/current/restart');assert positions(engine)==initial and engine.simulation.tick_count==0
        client.post('/api/scenarios/head_on');assert positions(engine)==initial
        next_name=client.post('/api/control/next-scenario').json()['scenario'];assert next_name=='three_robot_intersection'
        reset=client.post('/api/reset').json();assert reset=={'scenario':'basic','run_state':'PAUSED'}
        assert client.post('/api/scenarios/not-real').status_code==404

def test_task_creation_priority_and_destination_validation():
    client,engine=client_and_engine()
    for _ in range(50):engine.tick()
    with client:
        created=client.post('/api/tasks',json={'destination_x':30,'destination_y':2,'priority':'HIGH'})
        assert created.status_code==200 and created.json()['priority']=='HIGH' and created.json()['assigned_robot_id']
        assert client.post('/api/tasks',json={'destination_x':5,'destination_y':5,'priority':'NORMAL'}).status_code==409
        assert client.post('/api/tasks',json={'destination_x':99,'destination_y':99,'priority':'NORMAL'}).status_code==409
        assert client.post('/api/tasks',json={'destination_x':2,'destination_y':2,'priority':'URGENT'}).status_code==422

def test_robot_failure_and_environment_obstacle_injection_validation():
    client,engine=client_and_engine()
    with client:
        client.post('/api/scenarios/offline_robot_blockage')
        engine.step();engine.step()
        assert client.post('/api/robots/R2/fail').status_code==200
        assert client.post('/api/robots/R2/fail').status_code==409
        assert client.post('/api/robots/R9/fail').status_code==404
        assert {'x':2,'y':2} in client.get('/api/status').json()['grid']['dynamic_obstacles']
        client.post('/api/scenarios/basic')
        assert client.post('/api/environment/obstacles',json={'x':15,'y':12}).status_code==200
        assert client.request('DELETE','/api/environment/obstacles',json={'x':15,'y':12}).status_code==200
        assert client.post('/api/environment/obstacles',json={'x':5,'y':5}).status_code==409

def test_websocket_receives_complete_control_snapshot():
    client,engine=client_and_engine()
    with client:
        with client.websocket_connect('/ws') as websocket:
            snapshot=websocket.receive_json()
            assert len(snapshot['robots'])==3
            assert {'run_state','speed','scenario_info','conflicts','metrics','events'}<=snapshot.keys()

def test_failure_events_preserve_local_observer_decisions():
    client,engine=client_and_engine();engine.load_scenario('robot_failure')
    engine.step();engine.step();engine.step();engine.fail_robot('R2')
    for _ in range(15):engine.step()
    with client:
        data=client.get('/api/status').json();r2=next(r for r in data['robots'] if r['robot_id']=='R2')
        assert r2['online'] is False
        offline=[e for e in data['events'] if e['type']=='ROBOT_OFFLINE']
        assert {e['observer_robot_id'] for e in offline}=={'R1','R3'}
        assert {e['offline_robot_id'] for e in offline}=={'R2'}

def test_event_log_clear_and_dashboard_has_no_traffic_control_surface():
    client,engine=client_and_engine()
    with client:assert client.delete('/api/events').json()=={'events':0}
    source='\n'.join(path.read_text(encoding='utf-8') for path in Path('dashboard').glob('*.py'))
    for prohibited in ('resolve_conflict(', 'known_corridor_owner_for(', '_evaluate_deadlock_before_motion(', '_evaluate_conflicts_before_motion(', 'ConflictResolver(', 'CorridorProtocol('):assert prohibited not in source
def test_structured_payload_mission_capacity_and_real_progress_api():
    client,engine=client_and_engine()
    engine.simulation.run_until_complete(100)
    with client:
        response=client.post('/api/tasks',json={
            'task_type':'PICKUP_DELIVERY','source_x':10,'source_y':2,
            'destination_x':30,'destination_y':2,'priority':'HIGH',
            'payload_id':'PALLET-9','payload_type':'PALLET','weight_kg':45,'fragile':True,
        })
        assert response.status_code==200
        task=response.json();assert task['assigned_robot_id']=='R2' and task['payload']['weight_kg']==45
        assert task['source']=={'x':10,'y':2} and task['progress']==0
        for _ in range(8):engine.step()
        updated=next(t for t in client.get('/api/tasks').json() if t['task_id']==task['task_id'])
        assert 0<updated['progress']<100 and updated['eta_ticks'] is not None


def test_typed_obstacle_metadata_revision_and_nonduplicated_routes():
    client,engine=client_and_engine()
    with client:
        initial=client.get('/api/status').json()['scenario_revision']
        added=client.post('/api/environment/obstacles',json={'x':15,'y':12,'obstacle_type':'FORKLIFT'})
        assert added.status_code==200 and added.json()['object_type']=='FORKLIFT'
        status=client.get('/api/status').json();assert {'x':15,'y':12,'object_type':'FORKLIFT'} in status['grid']['environment_objects']
        client.post('/api/scenarios/current/restart');assert client.get('/api/status').json()['scenario_revision']==initial+1
        paths=[route.path for route in client.app.routes]
        assert paths.count('/api/control/next-scenario')==1 and '/api/scenarios/next' not in paths


def test_event_sources_are_consumed_once_instead_of_replayed_each_tick():
    engine=DashboardEngine();engine.load_scenario('robot_failure')
    for _ in range(3):engine.step()
    engine.fail_robot('R2')
    for _ in range(15):engine.step()
    first=[e for e in engine.snapshot()['events'] if e['type']=='ROBOT_OFFLINE']
    for _ in range(4):engine.step()
    second=[e for e in engine.snapshot()['events'] if e['type']=='ROBOT_OFFLINE']
    assert len(first)==len(second)==2