# ROS2 任务规划服务接口

规划节点只通过 ROS2 服务与任务规划桥交互。任务 YAML 文件、地图包和输出目录必须能被规划节点访问。

## 构建与启动

```bash
cd ~/colcon_ws/agibot_d1max_ws
colcon build --packages-select capability_mission_planner --symlink-install
source install/setup.bash
ros2 launch capability_mission_planner capability_mission_planner.launch.py
```

## 服务

```text
服务名: /capability_mission_planner/plan
服务类型: rcl_interfaces/srv/SetParameters
```

请求必须恰好有一个字符串参数：

```text
name: mission_yaml_path
type: PARAMETER_STRING
string_value: 任务 YAML 文件的绝对路径
```

调用示例：

```bash
ros2 service call /capability_mission_planner/plan \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: mission_yaml_path, value: {type: 4, string_value: '/data/missions/mission-001.yaml'}}]}"
```

节点读取该 YAML 配置并执行规划。配置中的相对路径相对于 YAML 文件所在目录解析；地图包本身使用标准 Nav2 地图 YAML、PNG 和多地图 CSV 文件。配置中的 `output_directory` 为必填字段。

## 响应

成功时：

```text
successful = true
reason = /data/mission_results/mission-001/plan.json
```

规划器会在 `output_directory` 中生成 `plan.json`、`summary.txt` 和路线 PNG，`reason` 只返回已生成 `plan.json` 的绝对路径，任务规划桥直接读取该文件即可。

失败时 `successful = false`，`reason` 为可读的失败原因，例如参数错误、路径不是绝对 `.yaml` 文件、文件不存在、YAML/地图配置错误或无可行规划结果。

## 配置要点

任务输入保持 YAML，输出计划保持 JSON。`export.navigation_checkpoint_types` 可选，用于筛选 `navigation_checkpoints`，不影响任务分配、路径、返航或冲突协调。合法值是 `start`、`task`、`turn`、`resource_entry`、`resource_exit`、`transition_entry`、`transition_exit`、`holding`、`finish`。`holding` 仅在协调器实际安排等待时出现，等待区间由 `arrival_tick` 和 `departure_tick` 给出；同一等待也会记录在 `traffic_events` 中。
