# Webots project

Open worlds/synq_three_robot.wbt for the three-AMR benchmark. It uses controllers/synq_controller for R1, R2, and R3 and controllers/synq_metrics for ground-truth measurement and emergency stop. Run the same world in both modes from the repository root:

~~~powershell
& '.\webots\controllers\synq_metrics\run_webots_experiment.ps1' -Mode both
~~~

The runner chooses the repository root relative to its own path. Webots R2025a supplies its Python controller API. The repository's Python 3.11 virtual environment should be installed with pip install -e ".[dev]". Set WEBOTS_HOME if Webots is not under %LOCALAPPDATA%\Programs\Webots.

The older worlds/synq_mir100_r1.wbt and controllers/synq_mir100 form a separate one-robot physical execution probe. It does not demonstrate peer coordination; use the three-robot world for the measured comparison.

See ../README.md and ../docs/EXPERIMENT.md for setup, results, and limitations.
