# Limitations

- One controlled overlapping-path benchmark scenario, with only three MiR100 robots. The 35.92% makespan reduction is measured for this scenario only.
- A known static map and configured starts/goals. The Webots benchmark uses simulated wheel-encoder localization; Supervisor ground-truth pose validates outcomes. Sensor-based localization and SLAM are not implemented.
- Webots radio simulates local peer communication. Real Wi-Fi, dead zones, packet loss, delay, and independent edge-computer deployment were not tested.
- Dynamic obstacle perception and physical rerouting are not demonstrated in the Webots benchmark. The separate grid simulator has more behaviors.
- The reported zero is zero detected inter-robot center-distance threshold breaches in the tested runs. It is not a proof of collision freedom, a contact-sensor result, or a production safety claim.
- No physical AMR validation, ROS 2 deployment, or large-fleet scalability benchmark.
- The benchmark Supervisor can issue an emergency stop on a threshold breach. It never chooses a traffic winner or robot route.
- Actual Webots viewport exports from real-time runs were inspected for start framing and physical movement. Native desktop control still fails to initialize, so GUI toolbar interaction was not independently inspected. The portable GUI launcher explicitly selects pause or real-time rendering.
- Webots uses external Cyberbotics PROTO references; initial asset resolution may require network access. These models retain Cyberbotics licensing.
