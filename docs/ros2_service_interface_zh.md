# ROS2 任务规划服务接口

`capability_mission_planner_node` 只通过 ROS2 服务与任务规划桥交互，不包含 MQTT、服务器协议或地图目录表依赖。

## 构建与启动

```bash
cd ~/colcon_ws/agibot_d1max_ws
colcon build --packages-select capability_mission_planner --symlink-install
source install/setup.bash
ros2 launch capability_mission_planner capability_mission_planner.launch.py
```

无需单独执行 `cmake --build`。启动节点也不需要指定地图参数。

启动参数：

- `max_request_bytes`：单次 `mission_json` 最大字节数，默认 `1048576`。
- `max_result_bytes`：单次计划结果最大字节数，默认 `4194304`。

例如：

```bash
ros2 launch capability_mission_planner capability_mission_planner.launch.py \
  max_request_bytes:=2097152 max_result_bytes:=8388608
```

## 服务

```text
服务名: /capability_mission_planner/plan
服务类型: rcl_interfaces/srv/SetParameters
```

请求必须恰好有一个参数：

```text
name: mission_json
type: PARAMETER_STRING
string_value: <UTF-8 JSON>
```

请求 JSON：

```json
{
  "request_id": "mission-20260908-0001",
  "map": {
    "directory": "/data/mission_maps/bdz1",
    "allow_unknown": false,
    "inflation_radius_m": 0.0,
    "inscribed_radius_m": 0.20,
    "cost_scaling_factor": 10.0,
    "persistent_cache": true
  },
  "robots": [
    {
      "id": "robot-01",
      "start": {"map_id": "bdz1", "grid": [100, 1036]},
      "capabilities": ["fire", "camera", "gas"],
      "return_home": true,
      "clearance_radius_m": 0.01,
      "safety_margin_m": 0.0,
      "nominal_speed_mps": 0.8,
      "footprint_radius_m": 0.0
    }
  ],
  "tasks": [
    {
      "id": "task-001",
      "location": {"map_id": "bdz1", "grid": [300, 1036]},
      "requirements": ["fire"],
      "category": "fire_suppression",
      "service_seconds": 6,
      "high_priority": true,
      "position_tolerance_m": 0.0
    }
  ],
  "planner": {
    "coordinate_conflicts": true,
    "objective": {
      "maximum_load_weight": 1.5,
      "total_load_weight": 0.1
    },
    "traversal": {
      "time_step_seconds": 0.1,
      "nominal_speed_mps": 0.8,
      "obstacle_cost_weight": 1.0,
      "allow_diagonal": true,
      "downsample_costmap": true,
      "coarse_search_factor": 4
    }
  }
}
```

`request_id` 和 `map.directory` 是必填字段。`map.directory` 必须是规划节点所在设备上已经存在的绝对路径。该目录直接包含地图 YAML、PNG，以及多地图任务所需的 `map_relations.csv`、`transition_points.csv` 等文件；不依赖 `capability_mission_scenarios` 仓库。

`map.persistent_cache: true` 时，规划器会在地图目录中读取或生成 `.capability_mission_cache`。可以通过 `map.cache_directory` 指定一个单独的缓存目录。

对于多地图任务，`map.directory` 指向整个地图包，机器人和任务位置的 `map_id` 必须是该包中实际存在的地图 ID。

## 响应

请求只有一个参数，因此 `response.results` 也只有一个 `SetParametersResult`。

成功时：

```text
successful = true
reason = {"request_id":"...","status":"success","plan":{...}}
```

`plan` 与 CLI 输出的 `plan.json` 使用同一结构，包含地图信息、每台机器人路线、停靠任务、导航检查点、交通等待事件及负载统计。

失败时：

```text
successful = false
reason = {"status":"error","code":"INVALID_REQUEST","message":"..."}
```

错误码：

- `INVALID_REQUEST`：参数名、参数类型、任务 JSON 或地图目录不合法。
- `REQUEST_TOO_LARGE`：请求为空或超过 `max_request_bytes`。
- `RESULT_TOO_LARGE`：完整计划超过 `max_result_bytes`。
- `PLANNING_FAILED`：任务无法规划、能力不匹配、地图文件无效、位置不可通行或规划过程异常。

第一版将完整计划放入 `reason`，任务规划桥需要按 JSON 解析该字段，而不能将其视为普通错误文本。
