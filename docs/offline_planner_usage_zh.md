# capability_mission_planner 离线使用说明

## 1. 目录职责

项目只使用一个 `build/` 目录保存编译产物。规划结果默认写入 `output/`，两者不要
混用：

```text
build/                         CMake、目标文件和可执行程序
config/                        配置 schema 示例
output/offline_multi_map/      一次规划的本地输出
```

`build-offline/` 和 `build-sanitize/` 只是开发阶段隔离普通构建和 sanitizer 构建
时使用的临时目录，正常使用不需要创建。

## 2. 编译和运行

在项目根目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/capability_mission_planner_cli \
  ../capability_mission_scenarios/configs/myj1.yaml
```

配置中的 `output_directory` 决定输出位置。也可以在命令行临时覆盖：

```bash
./build/capability_mission_planner_cli \
  ../capability_mission_scenarios/configs/myj1.yaml \
  output/my_run
```

配置文件中的相对路径以配置文件所在目录为基准，命令行覆盖的输出路径以当前工作
目录为基准。

## 3. 地图输入

单地图目录包含一组标准 Nav2 YAML 和图像：

```text
map_000.yaml
map_000.png
```

多地图目录包含多组地图，并增加地图坐标关系和跨图通道：

```text
map_000.yaml ... map_006.yaml
map_000.png  ... map_006.png
map_relations.csv
transition_points.csv
```

地图配置示例：

```yaml
map:
  directory: ../../maps/myj1
  allow_unknown: false
  inflation_radius_m: 0.0
  inscribed_radius_m: 0.0
  cost_scaling_factor: 10.0
  persistent_cache: true
  cache_directory: ../../maps/myj1/.capability_mission_cache
```

- `allow_unknown`：是否允许路径经过未知栅格。
- `inflation_radius_m`：按机器人安全半径膨胀障碍物，单位为米。
- `inscribed_radius_m` 和 `cost_scaling_factor`：生成 Nav2 风格简化软代价场，供
  任务级路线排序使用，不替代 Nav2 的实时 footprint 检查。
- `persistent_cache`：是否将可通行栅格、净空场和软代价场保存为磁盘缓存；默认开启。
  设置为 `false` 时每次启动都重新解析并预处理地图。
- `cache_directory`：可选缓存目录；未设置时写入每张地图目录下的
  `.capability_mission_cache/`。缓存使用地图 YAML/PNG 的文件大小和修改时间及地图
  参数校验；任一项变化时自动重建。

## 4. 机器人和任务输入

每台机器人配置起点、能力和是否返航：

```yaml
robots:
  - id: a
    start: {map_id: map_000, grid: [409, 392]}
    capabilities: [fire, camera, stairs]
    return_home: true
    clearance_radius_m: 0.35
    safety_margin_m: 0.10
    nominal_speed_mps: 0.5
```

每个原子任务配置位置、能力要求、类型和作业时间：

```yaml
tasks:
  - id: T1-photo
    location: {map_id: map_000, grid: [278, 481]}
    requirements: [camera]
    category: gimbal_photo
    service_seconds: 2
    high_priority: true
    position_tolerance_m: 0.5
```

机器人数量和任务数量不要求相等。一台机器人可以获得零个、一个或多个任务。同一
位置的多个任务如果分给同一台机器人，会合并为一个停靠点。

`footprint_radius_m` 是机器人 footprint 外接圆半径。CBS 使用两机器人外接圆半径
与安全余量之和作为最小中心距离。未填写时为 `0`，保持旧配置的点代理行为；实际
机器狗配置必须显式填写。

只有能力集合覆盖任务全部要求的机器人才能接收该任务。例如要求
`[fire, camera]` 的任务不能交给只有 `fire` 能力的机器人。跨楼梯所需的
`stairs` 能力也会独立检查。

## 5. 位置坐标

每个位置必须给出 `map_id`，并且只使用以下三种坐标表示之一。

规划栅格坐标：

```yaml
location: {map_id: map_003, grid: [358, 336]}
```

该地图自身的 Nav2 局部坐标，单位为米：

```yaml
location: {map_id: map_003, local_xy: [1.25, -0.40]}
```

所有地图共用的 ROOT 平面坐标，单位为米；`map_id` 用于确定该位置属于哪张占据
栅格：

```yaml
location: {map_id: map_003, root_xy: [4.10, 0.99]}
```

PNG 左上角坐标不能直接填入 `grid`。规划栅格原点位于左下角，转换关系为：

```text
grid_x = image_column
grid_y = image_height - 1 - image_row
```

配置加载时会检查位置是否越界或落在障碍物上。

`position_tolerance_m` 允许任务点在给定半径内投影到最近的可达栅格，输出给 Nav2
时可作为目标位置容差。

## 6. 算法参数

```yaml
planner:
  coordinate_conflicts: true
  objective:
    maximum_load_weight: 1.0
    total_load_weight: 0.1
  traversal:
    time_step_seconds: 0.1
    nominal_speed_mps: 0.5
    obstacle_cost_weight: 1.0
    allow_diagonal: true
    downsample_costmap: false
    coarse_search_factor: 1
    resource_buffer_seconds: 0.5
    default_transition_seconds: 5.0
    map_switch_seconds: 2.0
    transition_seconds:
      stairs: 8.0
      elevator: 15.0
    transition_requirements:
      stairs: [stairs]
      elevator: []
```

- `coordinate_conflicts`：启用 CBS 时间协调；关闭时仍分配和规划路径，但不消解
  机器人之间的时空冲突。
- `maximum_load_weight`：最大单机负载权重，越大越倾向负载均衡。
- `total_load_weight`：所有机器人总负载权重，越大越倾向减少总路程和总时间。
- `time_step_seconds`：时间离散粒度。
- `nominal_speed_mps`：机器人在栅格中的标称速度。
- `obstacle_cost_weight`：靠近障碍物的软代价权重。
- `allow_diagonal`：是否允许八连通对角移动；对角穿过障碍角会被禁止。
- `downsample_costmap`：是否直接在降采样地图上进行任务级路径规划。开启后，
  粗路径用于任务分配、时间估计和路线输出；Nav2 执行时仍需基于实时 costmap
  重新规划。
- `coarse_search_factor`：N×N 栅格降采样因子。`1` 表示原始分辨率；建议从 `2`
  开始。粗栅格采用保守规则：净空取最小值、代价取最大值、区域内有障碍则不可通行。
- `resource_buffer_seconds`：共享资源占用前后的安全缓冲时间。

窄通道或交叉口可配置为共享资源：

```yaml
planner:
  shared_resources:
    - id: corridor-west
      capacity: 1 # 当前版本支持互斥资源；capacity 必须为 1
      buffer_seconds: 1.0
      cells: [[map_000, 10, 20], [map_000, 11, 20]]
```

CLI 输出中的 `timing_*_seconds` 分别对应配置加载、任务规划、文件导出和总耗时。
在大地图上启用降采样后，`timing_planning_seconds` 通常是最主要的可比较指标；
最终路线是粗栅格任务级路线，Nav2 执行时必须重新规划。
- `default_transition_seconds`：未单独配置的通道通过时间。
- `map_switch_seconds`：切换地图的附加时间。
- `transition_seconds`：按通道类型配置楼梯、电梯等通过时间。
- `transition_requirements`：按通道类型配置机器人必须具备的能力。

显示参数：

```yaml
export:
  path_thickness: 3
  draw_grid: false
  navigation_checkpoint_types: [start, task, turn, holding, finish]
```

`navigation_checkpoint_types` 可选，用于筛选导出的
`navigation_checkpoints`，不影响路径规划、任务分配、返航或冲突协调。未填写时输出
全部类型；填写空数组 `[]` 时不输出任何导航检查点。可选值为：

```text
start, task, turn, resource_entry, resource_exit,
transition_entry, transition_exit, holding, finish
```

`holding` 仅在多机器人协调确实插入等待时出现，`arrival_tick` 和
`departure_tick` 分别是等待开始和结束时间步。等待同时保留在 `traffic_events` 中；
筛选导航检查点不会删除该审计信息。

## 7. 输出

成功后输出目录包含：

```text
plan.json
summary.txt
routes_map_000.png
routes_map_001.png
...
```

- `plan.json`：机器人的任务分配、停靠顺序、负载、栅格/局部/ROOT 坐标，以及
  面向导航执行的 `navigation_checkpoints` 和冲突等待等 `traffic_events`。等待检查点
  的类型为 `holding`。
  CBS 内部仍按时间片求解，但导出只保留语义化导航关键点。
- `summary.txt`：便于人工快速阅读的路线摘要。
- `routes_<map_id>.png`：在每张原始栅格图上叠加机器人起点、任务编号、通道和
  规划路径。

当前接口只把结果写到本地，不启动 ROS2，也不直接向机器人发送任务。
