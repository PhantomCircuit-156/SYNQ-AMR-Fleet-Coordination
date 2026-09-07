"""In-process broadcast transport with no coordination authority."""

from __future__ import annotations

from dataclasses import dataclass, field

from models.messages import PeerStateMessage


@dataclass(slots=True)
class InProcessPeerMessageBus:
    """Route immutable messages between registered robot IDs only.

    The bus stores queues; it never owns robot objects and never interprets a
    message to make planning, priority, right-of-way, or safety decisions.
    """

    _inboxes: dict[str, list[PeerStateMessage]] = field(default_factory=dict)

    def register(self, robot_id: str) -> None:
        if robot_id in self._inboxes:
            raise ValueError(f"Robot is already registered: {robot_id}")
        self._inboxes[robot_id] = []

    def publish(self, message: PeerStateMessage) -> None:
        if message.robot_id not in self._inboxes:
            raise ValueError(f"Unregistered publisher: {message.robot_id}")
        for recipient_id, inbox in self._inboxes.items():
            if recipient_id != message.robot_id:
                inbox.append(message)

    def drain(self, robot_id: str) -> tuple[PeerStateMessage, ...]:
        if robot_id not in self._inboxes:
            raise ValueError(f"Unregistered recipient: {robot_id}")
        messages = tuple(self._inboxes[robot_id])
        self._inboxes[robot_id].clear()
        return messages

    def pending_message_count(self, robot_id: str) -> int:
        if robot_id not in self._inboxes:
            raise ValueError(f"Unregistered recipient: {robot_id}")
        return len(self._inboxes[robot_id])