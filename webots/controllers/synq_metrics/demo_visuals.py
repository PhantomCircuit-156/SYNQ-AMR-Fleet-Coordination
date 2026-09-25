"""Display-only labels and optional Webots viewport captures; no traffic decisions."""
import os
from pathlib import Path


class DemoVisuals:
    COLORS = {'R1': 0xEF4444, 'R2': 0x22C55E, 'R3': 0x3B82F6}

    def __init__(self, supervisor, mode):
        self.supervisor = supervisor
        self.mode = mode
        self.states = {}
        self.capture_times = iter((0.032, 12.0, 30.0, 50.0))
        self.next_capture = next(self.capture_times, None)
        directory = os.environ.get('SYNQ_CAPTURE_DIR')
        self.directory = Path(directory) if directory else None
        if self.directory:
            self.directory.mkdir(parents=True, exist_ok=True)
        supervisor.setLabel(90, f'SYNQ | {mode.upper()} | SIMULATED LOCALIZATION',
                            0.02, 0.02, 0.07, 0xFFFFFF, 0, 'Arial')
        for index, (name, color) in enumerate(self.COLORS.items()):
            supervisor.setLabel(91 + index, f'{name}: INITIALIZING',
                                0.02, 0.09 + index * 0.055, 0.065, color, 0, 'Arial')

    def update(self, now, telemetry):
        for index, (name, color) in enumerate(self.COLORS.items()):
            packet = telemetry.get(name)
            if not packet:
                continue
            status = packet.get('status', 'INITIALIZING')
            display = 'YIELDING' if status == 'WAITING' and self.mode == 'synq' else status
            if self.states.get(name) != display:
                previous = self.states.get(name)
                self.states[name] = display
                self.supervisor.setLabel(91 + index, f'{name}: {display}',
                                         0.02, 0.09 + index * 0.055, 0.065, color, 0, 'Arial')
                event = 'RESUME / MOVING' if display == 'MOVING' and previous in ('WAITING', 'YIELDING') else display
                print(f'DEMO {name}: {event} t={now:.3f}', flush=True)
        if self.next_capture is not None and now >= self.next_capture:
            self.capture(now)
            self.next_capture = next(self.capture_times, None)

    def capture(self, now, final=False):
        if self.directory:
            suffix = 'complete' if final else f'{now:07.3f}s'
            target = self.directory / f'{self.mode}_{suffix}.png'
            self.supervisor.exportImage(str(target.resolve()), 100)
            print(f'DEMO viewport captured: {target.name}', flush=True)
