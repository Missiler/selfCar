<!-- Copilot / AI agent instructions for the `porsche` ROS2 package -->
# Quick orientation

This is an ament_cmake ROS2 package that integrates ROS2 nodes with the Gazebo (Ignition/ros_gz) simulator, Nav2, and robot_localization filters. The main runtime composition is defined in `launch/` and configuration lives in `config/`.

Key runtime pieces:
- Simulation: `ros_gz_sim` (`gz` / `gz sim`) via `ros_gz_sim.actions.GzServer` and `gz_spawn_model.launch.py`.
- Bridge: `ros_gz_bridge` configured by `config/bridge_config.yaml`.
- State publisher: `robot_state_publisher` driven from the model in `src/description/` (SDF/URDF/xacro).
- Localization/filtering: `robot_localization` nodes (`ekf_node` / `ukf_node`) configured from `config/*.yaml`.
- Navigation: `nav2_bringup` is included through `launch/display.launch.py` using `nav2` params and `RewrittenYaml`.

Files to inspect first
- `CMakeLists.txt`, `package.xml` — ament_cmake packaging and install targets.
- `launch/display.launch.py` and `launch/headless.launch.py` — how nodes, sim and bridge are composed and parameterized.
- `config/ekf.yaml`, `config/ukf.yaml`, `config/bridge_config.yaml`, `config/nav2_params*.yaml` — runtime tuning and frame names.
- `src/description/porsche_description.sdf` and `.urdf` — physical model used in sim and `robot_state_publisher`.
- `world/` and `world/maps/` — maps and world files used by Gazebo and Nav2.

Build / run (concrete commands)
- Build: `colcon build --packages-select porsche` from the workspace root.
- Source: `source install/setup.bash` (important before `ros2` or `colcon` commands in a new shell).
- Run display launch (GUI, SLAM/Navigation):
  `ros2 launch porsche display.launch.py use_sim_time:=True`
- Run headless (no mapper or GUI, uses UKF in this package):
  `ros2 launch porsche headless.launch.py use_sim_time:=False`
- The launch files call `gz sim` directly (`ExecuteProcess(cmd=['gz','sim','-g'])`). To restart sim manually use `gz sim -r` or run `gz sim` in a separate terminal.

Project-specific conventions & gotchas
- The package uses `use_sim_time` heavily. Launch files pass it via `DeclareLaunchArgument` and `RewrittenYaml` (see `display.launch.py`). Verify `use_sim_time` is set correctly when debugging time-sync issues.
- Headless vs display differences:
  - `display.launch.py` starts `ekf_node` configured by `config/ekf.yaml` and enables Nav2 bringup.
  - `headless.launch.py` starts `ukf_node` but currently points at `config/ekf.yaml` for parameters. This mismatch is a known gotcha — verify whether the `ekf.yaml` contents are intended for `ukf_node` before changing parameters.
- Frames: the filters expect `map`, `odom`, `base_link` frames configured in the YAML (see `config/ukf.yaml` and `config/ekf.yaml`). Confirm sensor topics match these frames (e.g., `pose0: /uwb/odom` in `ukf.yaml`).
- When adding a Nav2 parameter override, use `nav2_common.launch.RewrittenYaml` as shown in `display.launch.py` to inject `use_sim_time` and preserve typed values.

Testing & linting
- Tests reference `ament_lint_auto` / `ament_lint_common` in `package.xml`. Use `colcon test` and `ament_lint_auto` if tests are added.

Debugging checklist (fast commands)
- Verify nodes: `ros2 node list`
- Echo topics: `ros2 topic echo /tf` or `ros2 topic echo /uwb/odom`
- Inspect params: `ros2 param get /ukf_node use_sim_time` (or `ros2 param list`)
- Check Gazebo logs / processes: run `ps aux | grep gz` or check the terminal running `gz sim`.

External integrations to watch
- `ros_gz_sim` / `ros_gz_bridge` — simulator and bridge; changes to `bridge_config.yaml` map Gazebo topics to ROS topics.
- `robot_localization` — EKF/UKF configuration is YAML-driven; watch `*_config` entries for which state elements are fused.
- `nav2_bringup` and SLAM Toolbox — Nav2 is included via `IncludeLaunchDescription` in `display.launch.py`.

When editing code or launch files
- Keep changes minimal and local to `launch/` or `config/` unless adding nodes or C++/Python sources under `src/`.
- Use `launch_ros.actions.Node` when adding new runtime nodes and pass parameters as file paths or dicts, following the existing patterns.

If anything here is unclear or you want me to expand sections (e.g., add examples for a new node, correct the ukf/ekf mismatch, or include test commands), tell me which area to expand.
