"""Robot-local intent decisions. No shared winner or traffic controller."""
from dataclasses import dataclass
from benchmark_config import APPROACH_CELLS

@dataclass(frozen=True)
class PeerIntent:
    robot_id: str
    timestamp: float
    cell: tuple[int, int]
    position: tuple[float, float]
    goal: tuple[int, int]
    next_waypoints: tuple[tuple[int, int], ...]
    status: str
    priority: int
    waiting_time: float

    @classmethod
    def from_dict(cls, data):
        return cls(data["robot_id"], float(data["timestamp"]), tuple(data["cell"]),
                   tuple(data["position"]), tuple(data["goal"]),
                   tuple(tuple(cell) for cell in data["next_waypoints"]),
                   data["status"], int(data["priority"]), float(data["waiting_time"]))

def priority_key(priority, waiting_time, robot_id):
    # One point per full five seconds waiting; ID breaks exact ties.
    return (priority * 10 + int(waiting_time // 5), -int(robot_id[1:]))

def should_wait(mode, mine, peers, region, now, timeout=0.5):
    """Decide before entering the region; occupants always finish crossing."""
    my_path = set(mine.next_waypoints) & region
    if not mine.next_waypoints or mine.next_waypoints[0] not in region or mine.cell in region:
        return False, ()
    blockers = []
    for peer in peers:
        if now - peer.timestamp > timeout or peer.status == "TASK_COMPLETE":
            continue
        if mode == "baseline":
            if peer.cell not in region and peer.cell not in APPROACH_CELLS:
                continue
            # Conservative region reservation. If both approach, fixed ID
            # breaks symmetry; an occupant always retains its reservation.
            block = peer.cell in region or peer.robot_id < mine.robot_id
        else:
            peer_path = set(peer.next_waypoints) & region
            if peer.cell not in region and not peer_path:
                continue
            shared = my_path & (peer_path | ({peer.cell} if peer.cell in region else set()))
            block = bool(shared) and (peer.cell in region or
                   priority_key(peer.priority, peer.waiting_time, peer.robot_id) >
                   priority_key(mine.priority, mine.waiting_time, mine.robot_id))
        if block:
            blockers.append(peer.robot_id)
    return bool(blockers), tuple(sorted(blockers))
