from fastapi.testclient import TestClient
from dashboard.app import DashboardEngine, create_app

def client_and_engine():
    engine=DashboardEngine(); return TestClient(create_app(engine,auto_tick=False)),engine

def test_dashboard_status_robots_metrics_and_events_are_exposed():
    client,engine=client_and_engine()
    with client:
        assert client.get('/api/status').status_code==200
        robots=client.get('/api/robots').json(); assert {r['robot_id'] for r in robots}=={'R1','R2','R3'}
        assert client.get('/api/metrics').json()['collisions']==0
        assert isinstance(client.get('/api/events').json(),list)

def test_dashboard_task_creation_and_scenario_validation():
    client,engine=client_and_engine()
    for _ in range(50): engine.tick()
    with client:
        created=client.post('/api/tasks',json={'destination_x':30,'destination_y':2})
        assert created.status_code==200 and created.json()['assigned_robot_id'] is not None
        assert client.get('/api/tasks').json()
        assert client.post('/api/scenarios/intersection').json()=={'scenario':'intersection'}
        assert client.post('/api/scenarios/not-real').status_code==404
        assert client.post('/api/reset').json()=={'scenario':'basic'}

def test_websocket_receives_current_state():
    client,engine=client_and_engine()
    with client:
        with client.websocket_connect('/ws') as websocket:
            snapshot=websocket.receive_json()
            assert snapshot['scenario']=='basic' and len(snapshot['robots'])==3

def test_failure_events_preserve_observer_and_offline_ids_and_robot_visibility():
    client,engine=client_and_engine(); engine.reset('robot_failure')
    for _ in range(15): engine.tick()
    with client:
        data=client.get('/api/status').json()
        r2=next(robot for robot in data['robots'] if robot['robot_id']=='R2')
        assert r2['online'] is False
        offline=[event for event in data['events'] if event['type']=='ROBOT_OFFLINE']
        assert {event['observer_robot_id'] for event in offline}=={'R1','R3'}
        assert {event['offline_robot_id'] for event in offline}=={'R2'}

def test_dashboard_has_no_traffic_control_surface():
    source=open('dashboard/app.py',encoding='utf-8').read()
    for prohibited in ('resolve_conflict(', 'known_corridor_owner_for(', '_evaluate_deadlock_before_motion(', '_evaluate_conflicts_before_motion('):
        assert prohibited not in source