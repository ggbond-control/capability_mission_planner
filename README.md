# Capability Mission Planner

Capability-aware multi-robot, multi-task mission planning with internal modules
for:

- task booking, priority, label, category, and duration metadata;
- A* obstacle-aware travel costs;
- CBS conflict-free timed schedules.

The planner accepts any number of robots, points, and atomic tasks. A robot may
receive zero, one, or many tasks. Tasks at the same point are merged into one
stop when they are assigned to the same robot.

## Build and test

The planner is self-contained. Adapted source is organized by function under
`include/capability_mission_planner/` and `src/`; its upstream licenses are
retained under `third_party/licenses/`. No RMF workspace or separate planning
project is required at configure, build, or runtime.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Run the 4-robot, 20-point, 40-atomic-task example with:

```bash
./build/mission_demo > mission_plan.json
```

Validate planning on a PNG occupancy map and render the result with:

```bash
./build/image_map_demo examples/data/image.png output/capability_mission_plan.png
```

This optional example uses system OpenCV for PNG input/output and raster
preprocessing. The planning library itself does not depend on OpenCV.

## Offline Nav2 map bundles

The optional offline adapter accepts either of these local directory layouts:

```text
single_map/
  map_000.yaml
  map_000.png

multi_map/
  map_000.yaml ... map_N.yaml
  map_000.png  ... map_N.png
  map_relations.csv
  transition_points.csv
```

Each YAML file uses the standard Nav2 `image`, `resolution`, `origin`,
`negate`, `occupied_thresh`, `free_thresh`, and `mode` fields. Multi-map
relations are resolved into a common ROOT coordinate frame. Transition rows
define stairs, elevators, or other portals; traversal can require matching
robot capabilities.

Scenario maps and mission configurations are maintained separately in
[`capability_mission_scenarios`](../capability_mission_scenarios). Run the
supplied single-map and seven-map validations with:

```bash
./build/offline_map_demo \
  ../capability_mission_scenarios/maps/zju2 output/zju2_demo
./build/offline_map_demo \
  ../capability_mission_scenarios/maps/myj1 output/myj1_demo
```

For an actual configurable run, edit a scenario configuration in
[`capability_mission_scenarios`](../capability_mission_scenarios) and execute:

```bash
./build/capability_mission_planner_cli \
  ../capability_mission_scenarios/configs/myj1.yaml
```

The complete Chinese input, parameter, coordinate, and output reference is in
[`docs/offline_planner_usage_zh.md`](docs/offline_planner_usage_zh.md).

The ROS2 service adapter and task-planning bridge interface are documented in
[`docs/ros2_service_interface_zh.md`](docs/ros2_service_interface_zh.md).

No ROS2 process is started and no result is published. Each output directory
contains:

- `plan.json`: assignments, loads, stops, grid/local/ROOT coordinates, and
  conflict-free timed schedules;
- `summary.txt`: compact human-readable route summary;
- `routes_<map_id>.png`: one original occupancy map per layer with robot paths,
  starts, tasks, and transition points overlaid.

Cross-map routing uses A* inside each occupancy grid and Dijkstra over the
sparse transition topology. Task allocation minimizes weighted maximum robot
load plus total load. CBS then schedules progress and waits along the selected
routes, resolving same-cell, opposing-edge, and shared-transition conflicts.

The offline planner uses a Nav2-inspired clearance cost field and optional
robot-specific `clearance_radius_m`, `safety_margin_m`, and
`nominal_speed_mps`. It emits task-level stops with position tolerances for
Nav2; Nav2 remains responsible for final footprint collision checking,
real-time replanning, and control.

For large maps, set `planner.traversal.downsample_costmap: true` and choose a
`coarse_search_factor` (typically 2 or 4) to plan directly on a conservative
downsampled grid. This accelerates task-level planning; Nav2 must still replan
each executable segment on its live costmap.

## Planning hierarchy

1. Reject robot-task edges that do not satisfy every required capability.
2. Insert constrained/high-priority tasks first, minimizing maximum route load
   plus weighted total load with exact A* travel costs.
3. Merge same-location tasks assigned to the same robot.
4. Convert each route into ordered milestones and mandatory service waits.
5. Use CBS to remove vertex and opposing-edge conflicts while preserving task
   order.
