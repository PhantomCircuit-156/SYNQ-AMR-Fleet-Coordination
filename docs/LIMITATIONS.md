# Limitations

- One controlled overlapping-path benchmark scenario, with only three MiR100 robots. The 35.92% makespan reduction is measured for this scenario only.
- A known static map and configured starts/goals. The Webots benchmark uses simulated wheel-encoder localization; Supervisor ground-truth pose validates outcomes. Sensor-based localization and SLAM are not implemented.
- Webots radio simulates local peer communication. Real Wi-Fi, dead zones, packet loss, delay, and independent edge-computer deployment were not tested.
- Dynamic obstacle perception and physical rerouting are not demonstrated in the Webots benchmark. The separate grid simulator has more behaviors.
- The reported zero is zero detected inter-robot center-distance threshold breaches in the tested runs. It is not a proof of collision freedom, a contact-sensor result, or a production safety claim.
- No physical AMR validation, ROS 2 deployment, or large-fleet scalability benchmark.
- The benchmark Supervisor can issue an emergency stop on a threshold breach. It never chooses a traffic winner or robot route.
- The Webots GUI was not visually verified by Codex because desktop control failed to initialize. The Supervisor ground-truth logs and saved result JSON document physical motion in batch mode; a human should still review the GUI run.
- Webots uses external Cyberbotics PROTO references; initial asset resolution may require network access. These models retain Cyberbotics licensing.
