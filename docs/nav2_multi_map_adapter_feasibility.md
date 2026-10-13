# Nav2 多栅格地图适配可行性与实施方案

## 1. 结论

为 `capability_mission_planner` 增加 Nav2 单地图及多地图适配是可行的。

- 单地图 Nav2 栅格适配：可行性高，主要工作是 YAML/PNG 解析和坐标转换。
- 多楼层任务规划：可行性中高，但不能只增加格式转换包装，需要增加地图 ID、
  传送拓扑和跨图路径代价模型。
- Nav2 执行对接：可行，但多机器人场景需要每台机器人独立的 Nav2 和
  `multi_map_nav` 命名空间。
- 示例数据与现有建图、导航实现之间的坐标关系一致，适配基础可靠。

真正的工作量主要不在文件解析，而在把当前“单张二值栅格上的二维规划”提升为
“多地图拓扑上的带资源时空规划”。仅做输入输出包装可以完成 Nav2 坐标转换，
但不足以保证多楼层任务分配和冲突规划正确。

## 2. 输入资料

分析涉及以下目录：

- 单地图示例：[`zju2`](../../capability_mission_scenarios/maps/zju2)
- 多地图示例：[`myj1`](../../capability_mission_scenarios/maps/myj1)
- 多地图构建端：`/home/jazzy/nav_t_ws/src/gridmapper`
- 多地图使用端：`/home/jazzy/nav_t_ws/src/multi_map_nav_ros2`

构建端重点参考：

```text
/home/jazzy/nav_t_ws/src/gridmapper/launch/global.launch.py
```

使用端重点参考：

```text
/home/jazzy/nav_t_ws/src/multi_map_nav_ros2
```

## 3. 单地图示例分析

[`zju2/map_000.yaml`](../../capability_mission_scenarios/maps/zju2/map_000.yaml)
描述了一张标准 Nav2 栅格地图：

```text
图像尺寸：764 × 462
分辨率：0.05 m/格
原点：(-19.15, -4.25, 0)
```

覆盖的导航坐标范围约为：

```text
X：[-19.15, 19.05] m
Y：[-4.25, 18.85] m
```

图像像素统计为：

```text
纯白自由区域：约 17.3%
纯黑占据区域：约 80.7%
灰度未知区域：约 2.0%
```

这种单图可以直接转换为规划器的 `GridMap`，但需要正确处理图像 Y 轴方向、
未知栅格和机器人外形膨胀。

## 4. Nav2 坐标与图像坐标转换

PNG 图像的原点在左上角，Nav2 地图原点在左下角，因此 Y 轴需要翻转。

### 4.1 Nav2 坐标到规划栅格

```text
grid_x = floor((nav_x - origin_x) / resolution)
grid_y = floor((nav_y - origin_y) / resolution)
```

规划栅格到 PNG 像素索引：

```text
image_col = grid_x
image_row = image_height - 1 - grid_y
```

### 4.2 规划栅格到 Nav2 坐标

应返回栅格中心，而不是栅格边界：

```text
nav_x = origin_x + (grid_x + 0.5) × resolution
nav_y = origin_y + (grid_y + 0.5) × resolution
```

使用栅格中心可以避免浮点误差导致往返转换时落入相邻栅格。

### 4.3 原点旋转

Nav2 YAML 的 `origin` 第三个值可以表示地图旋转角。当前示例全部为 `0`，现有
`multi_map_nav_ros2` 也主要按纯平移处理。

适配器的数据结构仍应保留完整 SE(2) 变换能力。如果未来出现非零 YAML yaw，
需要在平移之外应用二维旋转。当前 `map_relations.csv` 只包含 `dx,dy`，若地图
坐标系之间存在旋转，还需要扩展关系文件格式或明确禁止旋转关系。

## 5. 多地图示例分析

[`myj1`](../../capability_mission_scenarios/maps/myj1) 包含：

- 7 张地图：`map_000` 至 `map_006`；
- 每张地图各自的 PNG、分辨率、原点和尺寸；
- [`map_relations.csv`](../../capability_mission_scenarios/maps/myj1/map_relations.csv)；
- [`transition_points.csv`](../../capability_mission_scenarios/maps/myj1/transition_points.csv)。

地图拓扑是一条双向楼梯链：

```text
map_000 ↔ map_001 ↔ map_002 ↔ map_003
        ↔ map_004 ↔ map_005 ↔ map_006
```

6 个传送点均为 `stairs`，并且全部标记为双向。

## 6. 地图关系

`map_relations.csv` 格式为：

```text
from_map,to_map,dx,dy
```

从 `ROOT` 开始累加关系后，得到每张地图局部坐标系相对 ROOT 的偏移：

```text
map_000: (0.00000, 0.00000)
map_001: (2.55496, 1.19778)
map_002: (2.97657, 3.00940)
map_003: (2.46931, 1.39610)
map_004: (2.84887, 3.00544)
map_005: (3.19016, 1.26534)
map_006: (3.24055, 3.09797)
```

在当前纯平移模型下，地图局部坐标与 ROOT 坐标的关系为：

```text
root_x = local_x + map_offset_x
root_y = local_y + map_offset_y

local_x = root_x - map_offset_x
local_y = root_y - map_offset_y
```

`map_relations.csv` 描述地图局部坐标系相对 ROOT 的关系，不是传送点列表。

## 7. 传送点

`transition_points.csv` 格式为：

```text
transition_id,from_map,to_map,
world_x,world_y,world_z,world_yaw_rad,
bidirectional,type
```

它描述：

- 从哪张地图切换到哪张地图；
- 发生切换时机器人在 ROOT 坐标系中的位姿；
- 高度 `world_z`；
- 朝向；
- 是否双向；
- 楼梯、电梯、门等传送类型。

地图关系与传送点用途不同，不能互相替代。

### 7.1 数据闭环检查

6 个传送点均在相邻两张地图中落入纯白自由栅格。

例如 `tp_1`：

```text
ROOT 坐标：(2.55496, 1.19778)

在 map_000 中：
  local = (2.55496, 1.19778)

在 map_001 中：
  local = (0, 0)
```

后续传送点也呈现相同规律：传送点在旧地图中是楼梯出口位置，在首次创建的新
地图中通常成为局部 `(0,0)` 附近的位置。这说明构建端和使用端对于坐标关系的
定义是一致的。

## 8. 构建端行为分析

`gridmapper/launch/global.launch.py` 启动 `gridmapper_node` 的 GLOBAL 模式。

构建端调用 `/switch_map` 时的主要步骤为：

1. 保存当前地图的 PNG、YAML 和内部 GridMap 状态；
2. 读取机器人切图时的 ROOT 位姿；
3. 将该位置记录为传送点；
4. 如果目标地图首次创建，将当前位置作为新地图坐标系的 ROOT 偏移；
5. 将相邻地图的偏移写入 `map_relations.csv`；
6. 创建或加载目标地图，并将其设为 active map。

构建端维护：

```text
T_ROOT_map
T_ROOT_active_map
```

关系文件记录的是地图坐标系之间的平移，传送点文件记录的是切图时的 ROOT
世界位姿。

## 9. 使用端行为分析

`multi_map_nav_ros2` 会：

1. 扫描所有地图 YAML；
2. 加载并校验 `map_relations.csv`；
3. 从 ROOT 开始累加每张地图的偏移；
4. 加载 `transition_points.csv` 并构建地图邻接图；
5. 为每张地图生成临时 Nav2 YAML；
6. 通过 `map_server/load_map` 动态切换地图；
7. 等待 Nav2 StaticLayer 元数据同步后继续导航。

临时 Nav2 YAML 的原点为：

```text
nav2_origin = original_yaml_origin + T_ROOT_map.translation
```

因此 Nav2 始终使用统一的 `map`/ROOT 平面坐标进行导航，而 `map_id` 用来选择
当前加载的占据图。

跨图导航会被拆成：

```text
当前地图导航到传送点
  ↓
调用 map_server/load_map
  ↓
等待 StaticLayer 同步
  ↓
在新地图中继续导航
  ↓
到达最终任务点
```

使用端当前在地图拓扑上使用 BFS，即选择传送次数最少的地图序列，而不是移动
时间、地图内距离与传送时间之和最小的路线。

## 10. 当前规划器的模型差距

当前规划器的位置类型只有：

```cpp
struct Location {
  int x;
  int y;
};
```

它不能区分：

```text
(map_002, x=100, y=50)
(map_004, x=100, y=50)
```

两个位置虽然栅格坐标相同，但属于不同楼层，不能被视为同一位置。

当前 `GridPathPlanner` 也只支持一张矩形二值地图，无法表达：

- 传送边；
- 楼梯、电梯、门等传送类型；
- 机器人是否具备爬楼能力；
- 传送耗时；
- 楼梯、电梯容量；
- 不同地图上的相同坐标不产生空间冲突；
- 多机器人同时使用同一传送设施的资源冲突。

因此，仅增加 PNG/YAML 读取器不能完成正确的多楼层任务规划。

## 11. 建议的模块结构

离线适配实现已组织为独立模块，当前目录结构为：

```text
src/offline/
  offline_mission_planner.cpp
  offline_planner_config.cpp
  maps/                       # 地图读取、坐标转换、预处理、缓存与跨地图路径
  coordination/               # 冲突协调、路线预约与执行阶段资源预约
  output/                     # JSON、文本、PNG 与 GIF 导出

src/ros/
  mission_planner_node.cpp     # ROS2 接口适配
```

公开接口声明放在：

```text
include/capability_mission_planner/offline/
```

实现归入 `src`，公开的数据类型和调用接口仍放在 `include`。仅把所有声明和实现
都放在 `src` 会导致外部调用者无法规范地使用适配器。

建议将 ROS 无关的数据解析、坐标转换和规划逻辑与 ROS2 执行节点分开编译，避免
让核心规划库强制依赖 ROS2。

## 12. 建议的数据模型

建议增加平行的多地图类型，避免破坏现有单地图 API：

```cpp
struct MapGridLocation {
  std::string map_id;
  int x;
  int y;
};

struct MapPose {
  std::string map_id;
  double x;
  double y;
  double z;
  double yaw;
};

struct MapTransition {
  std::string id;
  std::string from_map;
  std::string to_map;
  MapPose world_pose;
  std::string type;
  bool bidirectional;
  double traversal_time;
  std::size_t capacity;
};

struct MultiMapBundle {
  std::map<std::string, OccupancyMap> maps;
  std::vector<MapTransition> transitions;
  std::map<std::string, Transform2D> root_transforms;
};
```

机器人起点和任务位置需要使用 `MapPose` 或 `MapGridLocation`，不能继续只使用
不带地图 ID 的 `Location`。

## 13. 地图加载器职责

`MapBundleLoader` 应负责：

- 扫描 `map_*.yaml`；
- 使用结构化 YAML 解析器读取字段；
- 解析 PNG/PGM 图像；
- 解析 `mode`、`negate`、`occupied_thresh`、`free_thresh`；
- 校验图像路径、尺寸、分辨率和原点；
- 加载 `map_relations.csv`；
- 从 ROOT 计算每张地图的变换；
- 检查关系图是否连通、是否存在冲突环；
- 加载 `transition_points.csv`；
- 检查传送点两侧是否在地图范围内且可通行；
- 为双向传送点创建反向边；
- 输出包含详细错误位置的校验报告。

## 14. 占据图语义

示例 YAML 使用：

```yaml
mode: scale
occupied_thresh: 1
free_thresh: 0
```

对应语义为：

- 纯黑：占据；
- 纯白：自由；
- 中间灰度：未知。

适配器的默认安全策略应为：

```text
黑色 → 障碍
白色 → 可通行
灰色 → 不可通行
```

后续可提供配置，允许机器人进入未知区域并增加路径代价。但当前 `GridMap` 是
二值模型，如果需要保留灰度可通行性，就需要增加带权地图类型。

另外还应根据机器人 footprint 或半径对障碍物进行膨胀。否则上层规划器可能
认为狭窄通道可通行，但 Nav2 inflation layer 会在实际执行时拒绝该路线。

不同机器人尺寸不同时，膨胀结果也可能不同，路径可达性需要按机器人型号或
尺寸配置。

## 15. 跨图路径算法

跨图路径不能只对地图 ID 使用 BFS。建议使用分层 A* 或带权 Dijkstra。

一条跨图路径由以下部分组成：

```text
机器人当前位置
  ↓ 当前地图内 A*
传送点入口
  ↓ 传送边代价
目标地图传送点
  ↓ 目标地图内 A*
最终任务点
```

总代价建议统一为时间：

```text
cost =
  各地图内实际移动时间
  + 各传送边耗时
  + 地图切换固定开销
  + 任务服务时间
```

如果一对地图之间存在多个楼梯或电梯，带权搜索可以选择整体代价最低的传送点，
而不是简单选择地图跳数最少的路线。

传送边还应支持能力约束。例如：

```text
stairs   → 需要 stair_climbing
elevator → 需要 elevator_access
door     → 可能需要 door_access
```

这样任务能力约束和路径能力约束可以同时参与机器人候选过滤。

## 16. 路径规划接口抽象

当前 `MissionPlanner` 直接依赖具体的 `GridPathPlanner`。为了复用现有任务分配、
最便宜插入和局部搜索算法，建议抽象路径代价接口，例如：

```cpp
class RouteCostProvider {
public:
  virtual PathResult plan(
    const Robot& robot,
    const MapGridLocation& start,
    const MapGridLocation& goal) const = 0;
};
```

然后提供：

```text
SingleGridPathPlanner
MultiMapPathPlanner
```

这样不需要复制一套任务分配算法，也能保留现有单图功能。

## 17. 时间与代价单位

当前规划器隐含使用：

```text
移动一个栅格 = 1 个路径代价单位 = 1 个时间步
```

但示例地图分辨率为 `0.05 m`，任务服务时长使用秒。直接相加会混合栅格步数和
秒，物理含义不一致。

多地图适配时应统一为时间：

```text
移动时间 = 栅格路径长度 × resolution / robot_speed

总负载 =
  移动时间
  + 传送时间
  + 地图切换时间
  + 任务服务时间
```

CBS 的时间分辨率也应显式配置，例如 `0.5 s`，不能继续隐含地认为一个
`0.05 m` 栅格等于一秒。

## 18. CBS 多地图扩展

多地图 CBS 状态至少应包含：

```text
map_id
grid_x
grid_y
time
task_progress
```

冲突规则需要扩展为：

- 只有 `map_id` 相同且栅格相同才产生普通空间冲突；
- 同地图上仍检查顶点冲突和对向边冲突；
- 传送设施作为共享资源处理；
- 楼梯容量为 1 时，同一时间只能有一台机器人进入；
- 电梯需要建模为带持续时间、容量和可能等待时间的资源；
- 地图切换不能被视为零时间瞬移。

对于资源冲突，可以增加：

```text
ResourceConstraint {
  resource_id;
  start_time;
  end_time;
  capacity;
}
```

这样 CBS 不仅消解栅格冲突，也能消解楼梯、电梯等共享资源冲突。

## 19. Nav2 输出适配

发送给 `multi_map_nav_ros2` 的任务点应保持地图局部坐标：

```text
PoseStamped.header.frame_id = "map_003"
PoseStamped.pose.position   = map_003 局部米制坐标
```

执行接口可以使用：

```text
multi_map_navigate_to_pose
```

或：

```text
multi_map_navigate_through_poses
```

`multi_map_nav_ros2` 负责：

- 局部坐标转换为 ROOT 坐标；
- 选择地图传送序列；
- 导航到传送点；
- 调用 `map_server/load_map`；
- 等待 StaticLayer 同步；
- 继续执行目标导航。

规划器输出的路线应保留任务边界、服务时长和任务 ID。现有
`waypoint_dwell_time` 是统一参数，但每个任务的服务时间可能不同，因此执行器
最好逐任务提交，或者在每个任务完成后自行等待，不能完全依赖统一 dwell time。

## 20. 多机器人部署约束

当前 `multi_map_nav_ros2` 使用若干绝对 ROS 名称，例如：

```text
/multi_map_goal_pose
/odometry_multi_maps
/validate_route_waypoints
/manual_switch_current_map
```

如果在一台计算机上同时启动多个机器人实例，会发生接口冲突。

实施时应满足：

- 每台机器人使用独立 namespace；
- 将绝对名称改为相对名称，或者提供完整 remap；
- 每台机器人拥有独立的 `map_server` 和 Nav2 栈；
- 每台机器人拥有独立的 `multi_map_nav` 实例；
- 包装执行器根据 `robot.id` 选择对应 action server；
- 规划器集中计算全局任务分配，执行器分别下发每台机器人的路线。

## 21. 建议的实施阶段

### 阶段 1：地图数据层

- 解析 YAML、PNG、关系 CSV 和传送 CSV；
- 实现严格的格式和拓扑校验；
- 实现局部坐标、ROOT 坐标、规划栅格和 PNG 像素之间的转换；
- 完成坐标往返误差测试。

### 阶段 2：单地图适配

- 使用示例 1 构造 `GridMap`；
- 验证 Nav2 米制坐标到规划栅格的转换；
- 验证 A* 路径不穿越黑色和未知区域；
- 验证输出坐标可以交给 Nav2。

### 阶段 3：多地图拓扑

- 使用示例 2 加载 7 张地图；
- 构建 ROOT 变换和传送图；
- 验证 6 个传送点在两侧地图中都可通行；
- 验证正向和反向地图路径。

### 阶段 4：跨图距离接口

- 将 `MissionPlanner` 对具体 `GridPathPlanner` 的依赖抽象为路径代价接口；
- 保留现有单图实现；
- 增加多图分层 A* 或 Dijkstra 实现；
- 对跨图路径代价进行缓存。

### 阶段 5：多图任务规划

- 为机器人起点和任务位置加入 `map_id`；
- 将任务能力和传送能力同时纳入候选过滤；
- 在最大负载和总负载中计入地图内移动与传送时间；
- 输出带地图 ID 的停靠点和分段路径。

### 阶段 6：多图 CBS

- 在 CBS 状态中加入 `map_id`；
- 保留同图栅格冲突检测；
- 增加楼梯、电梯等传送资源的占用区间；
- 验证多机器人相反方向使用楼梯的冲突消解。

### 阶段 7：ROS2 执行包装

- 将路线转换为每台机器人的 multi-map Nav2 action；
- 处理 action 接受、反馈、成功、取消和失败；
- 处理服务时间；
- 处理地图加载失败和机器人偏离路径后的重规划；
- 发布任务和机器人执行状态。

### 阶段 8：系统验证

至少覆盖：

- 单地图单机器人；
- 单地图多机器人；
- 跨一层导航；
- 连续跨多层导航；
- 目标地图不可达；
- 单向传送；
- 机器人缺少爬楼能力；
- 传送点位于障碍或地图外；
- 多机器人争用单容量楼梯；
- 不同地图相同 `(x,y)` 不产生错误冲突；
- 坐标转换往返误差不超过半个栅格；
- Nav2 map server 切图和 StaticLayer 同步失败；
- 执行中动态障碍导致的失败与重规划。

## 22. 主要风险与约束

### 22.1 当前关系文件不支持旋转

`map_relations.csv` 只有 `dx,dy`。示例地图 yaw 为 0，因此当前数据没有问题；
未来如果地图坐标系存在旋转，需要扩展为 SE(2) 关系。

### 22.2 二值地图会丢失可通行性代价

`gridmapper` 输出来自 2.5D 地形分析，灰度可能携带未知或可通行性信息。直接转换
为二值 `GridMap` 会丢失坡度、粗糙度和风险代价。

### 22.3 上层路径与 Nav2 代价模型可能不一致

上层规划器如果不使用与 Nav2 相同的 footprint、inflation 和 unknown 策略，可能
输出理论可达但 Nav2 拒绝执行的路线。

### 22.4 传送时间和容量当前不在数据文件中

`transition_points.csv` 只有类型，没有明确的持续时间和容量。需要通过配置文件
按类型提供默认值，或者扩展 CSV 格式。

### 22.5 当前拓扑路径只优化跳数

`multi_map_nav_ros2` 当前 BFS 只最小化地图切换次数。规划器如果已经计算了更优的
具体传送序列，执行端需要能够遵循该序列，否则规划结果和实际执行路径可能不一致。

### 22.6 多机器人需要独立导航上下文

单个 `map_server` 同一时刻只能加载一张 active map。如果多台机器人位于不同
楼层，不能共享同一个 map server 和导航上下文。

## 23. 推荐实施边界

建议最终形成三层：

```text
多地图数据层
  YAML / PNG / CSV / 坐标转换 / 拓扑校验

多地图规划层
  能力约束 / 跨图路径 / 负载优化 / 多图 CBS

ROS2 执行层
  Nav2 action / map_server 切图 / 反馈 / 失败处理
```

`gridmapper` 作为地图生产者，适配器只消费其输出文件，不应成为
`capability_mission_planner` 的编译依赖。

`multi_map_nav_ros2` 作为执行参考和运行时接口，可以通过 ROS2 action/service
对接，也不需要作为源码或链接依赖加入核心规划库。
