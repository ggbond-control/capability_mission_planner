# 多机能力任务规划器

`capability_mission_planner` 是 C++17 离线规划库及 ROS2 服务：根据地图、机器人起点、能力和任务生成分配、路线与时间协调结果。服务端维护全局账本，Leader Agent 调用本包，各 Agent 转换 TaskHub YAML；本包不直接控制机器狗，也不维护在线成员或执行状态。

本文件说明当前接口、参数和使用方法；版本对比、性能数据与失败记录统一见 [TEST_LOG.md](TEST_LOG.md)。

阅读顺序：首次使用先看“构建与运行”和“输入 YAML”；现场排障看“地图与阈值”和“常见问题”；调整性能前先看“分配与时间协调”的预算边界。Server生成的内部YAML才是本包实际输入，不能直接将平台JSON交给CLI。

## 系统中的输入输出边界

| 上游／下游 | 交付内容 | 本包负责的部分 |
| --- | --- | --- |
| 平台或ROS调用者 → Server | 地图包、任务点、内层操作与能力要求 | 不直接消费平台协议 |
| Server → Leader Agent | 本轮机器人／未完成任务组成的规划YAML | 消费该YAML，不查询在线心跳 |
| Agent → 本包 | 本机可读的mission_yaml_path | 地图载入、任务分配、路径和时间协调 |
| 本包 → Agent → Server | plan.json路径、计划内容及PNG | 文件输出，不提交全局计划 |
| 各Agent → TaskHub | 转换后的本机执行YAML | 本包不直接调用start_route |

因此“没有能力可分配”和“起点不可通行”属于不同阶段；前者不能靠改地图阈值解决，后者也不能靠给机器人补传感器能力解决。Leader可以零任务，算法不会为Leader强制预留任务。

## 构建与运行

依赖OpenCV、yaml-cpp；ROS2环境为Jazzy。GIF导出可选，需在配置CMake前安装giflib开发库（Ubuntu包libgif-dev）。库与离线适配器的依赖范围不同，基础规划库不依赖OpenCV。包声明的许可证为Apache-2.0，见package.xml。

RK3588 和 PC 的性能测试均应显式使用 Release：

```bash
cd ~/Workspace/task_ws  # PC 改为对应 colcon 工作空间
source /opt/ros/jazzy/setup.bash
colcon build --packages-select capability_mission_planner --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch capability_mission_planner capability_mission_planner.launch.py
```

本包没有安装 CLI；离线调用工作空间构建目录中的程序：

```bash
./build/capability_mission_planner/capability_mission_planner_cli \
  /absolute/path/mission.yaml /absolute/path/output
```

也支持包目录内独立构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/capability_mission_planner_cli \
  ../capability_mission_scenarios/configs/myj1/myj1.yaml
```

场景地图与批量配置由相邻的 [capability_mission_scenarios](../capability_mission_scenarios) 维护，更多可选字段见 [配置示例](config/config_schema_example.yaml)。构建不会替换已运行进程，部署后需按原服务管理方式重启规划器。

## ROS2 服务

| 项目 | 约定 |
| --- | --- |
| 服务 | `/capability_mission_planner/plan` |
| 类型 | `rcl_interfaces/srv/SetParameters` |
| 请求 | 恰好一个名为 `mission_yaml_path` 的字符串参数 |
| 路径 | 规划板可访问、已存在的绝对 `.yaml` 文件 |
| 成功 | `results[0].successful=true`，`reason` 是生成的 `plan.json` 绝对路径 |
| 失败 | `successful=false`，`reason` 是参数、地图或规划失败原因 |

```bash
ros2 service call /capability_mission_planner/plan \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: mission_yaml_path, value: {type: 4, string_value: '/data/missions/mission.yaml'}}]}"
```

输入 YAML 的相对路径以该文件所在目录为基准。未设置 `output_directory` 时输出到输入文件所在目录；CLI 覆盖的相对输出路径以当前工作目录为基准。同名结果会覆盖，因此每个请求应使用独立目录。

## 输入 YAML

下面是规划器的内部输入格式，与平台 JSON 不同：平台只在子任务内填写 `requirements`，Server 汇总后才写成本格式的任务能力约束。

```yaml
map:
  directory: /data/maps/company2
robots:
  - id: robot_a
    start: {map_id: company2, local_xy: [1.25, 1.25]}
    capabilities: [device, gimbal.vis]
    nominal_speed_mps: 1.0
tasks:
  - id: point-1
    location: {map_id: company2, local_xy: [2.25, 1.25]}
    requirements: [device]
    category: photo
    service_seconds: 2
output_directory: /data/results/request-001
```

位置需指定 `map_id`，并且只选一种表达：整数 `grid: [col, row]`、地图局部米制 `local_xy: [x, y]` 或公共 ROOT 平面米制 `root_xy: [x, y]`。

机器人可增加：

| 字段 | 用途 |
| --- | --- |
| `return_home` | 返航位置对象，格式同 start；不接受布尔值，缺省结束于末任务点 |
| `clearance_radius_m` | 静态障碍净空半径 |
| `footprint_radius_m` | 多机冲突检查外接圆半径 |
| `safety_margin_m` | 安全余量 |
| `nominal_speed_mps` | 标称速度，缺省使用 traversal 默认值 |

任务的id、location、requirements、category、service_seconds在内部YAML中均必填，service_seconds按整数秒读取；Server接受JSON中的非负小数后，会在生成该YAML时向上取整。这个估计时长与Agent对子任务wait.duration的执行规范化分属两个边界。

任务可增加 `high_priority`、`position_tolerance_m`。同一机器人在同坐标的多个任务可合并停靠，原任务 ID 保留。任务点容差允许在范围内投影到有效栅格；机器人起点仍严格校验，不自动吸附。旧 `image_map_demo` 的起点修正行为不代表生产离线接口。

能力必须完整覆盖任务要求；楼梯等跨图通道也可独立要求能力。机器人可以获得零个任务，Leader 身份不保证获得任务。

## 地图与阈值

单图目录包含标准 Nav2 YAML 与其图像；多图目录还包含 `map_relations.csv`、`transition_points.csv`。地图关系使用 `from_map,to_map,dx,dy` 建立 ROOT 坐标关系，过渡点定义楼梯、电梯等跨图通道。可参考场景包中的完整配置。

```yaml
map:
  directory: /data/maps/VY0UgTP
  allow_unknown: false
  free_thresh_override: 0.05
  # occupied_thresh_override: 1.0  # 省略则逐图继承
  inflation_radius_m: 0.0
  inscribed_radius_m: 0.0
  cost_scaling_factor: 10.0
  persistent_cache: true
  cache_directory: /data/planner_cache
```

覆盖值仅影响本规划器，不修改地图或导航配置。每项须为有限的 [0, 1] 数值，有效值满足 `free <= occupied`；省略时逐图继承。Server 用 -1 表示继承，规划器 YAML 不接受这个哨兵值。

对于 `scale/trinary`，按 `negate` 得到占用概率 p：

| 范围 | 判定 |
| --- | --- |
| p <= free | 自由 |
| free < p < occupied | 未知，仅 allow_unknown=true 可走 |
| 其余 | 阻塞 |

free 与 occupied 相等时，等值属于自由区域。默认禁止未知区域时，仅调高 occupied 不会放宽自由区域。`raw` 模式沿用像素 0～100、`pixel/100 <= free` 的规则，occupied 和 allow_unknown 不改变其判定。

例如 negate=0、灰度253时 p≈0.007843，free=0.01 或0.05可接纳该像素。原图灰度不是导航 costmap 代价；通行阈值、膨胀、机器人净空和实时导航检查仍可能产生不同结论。

以8位灰度、negate=0为例，p=(255-gray)/255。下面仅说明自由阈值对原始像素的影响，不包含后续净空和膨胀：

| 原图灰度 | 占用概率约值 | free=0.01 | free=0.05 |
| --- | --- | --- | --- |
| 255 | 0 | 自由 | 自由 |
| 253 | 0.00784 | 自由 | 自由 |
| 248 | 0.02745 | 非自由 | 自由 |
| 240 | 0.05882 | 非自由 | 非自由 |

没有对所有地图通用的“安全放宽值”。确认地图mode、negate和实际失败栅格后再设置free，保持free不超过有效occupied；同时复查窄通道、障碍边缘及起点。不要把导航的0～255代价值代入这个概率公式，也不要为了绕过一个起点错误直接允许全图未知区域。

磁盘缓存默认开启，缺省目录为各地图的 `.capability_mission_cache/`。地图文件大小、修改时间及预处理参数变化会使缓存失效；阈值覆盖计入缓存签名。缓存目录应可写。障碍膨胀及软代价用于任务级规划，不能代替导航实时 footprint 检查。

## 分配与时间协调

流程为能力筛选、约束／高优先任务插入、任务搬移、有限点数均衡、路线生成、优先级时间协调，失败时回退 CBS。图内使用 A*，跨图通道使用稀疏图搜索。

```yaml
planner:
  coordinate_conflicts: true
  objective:
    maximum_load_weight: 1.0
    total_load_weight: 0.1
  allocation:
    max_passes: 8
    max_candidate_evaluations: 50000
    budget_seconds: 1.0
    balance_task_counts: true
    balance_load_slack: 0.10
  traversal:
    time_step_seconds: 0.1
    nominal_speed_mps: 0.5
    obstacle_cost_weight: 1.0
    allow_diagonal: true
    downsample_costmap: false
    coarse_search_factor: 1
    resource_buffer_seconds: 0.5
    coordination_local_repair: true
    coordination_repair_budget_seconds: 0.2
    coordination_repair_max_expanded_nodes: 20000
    default_transition_seconds: 5.0
    map_switch_seconds: 2.0
    transition_seconds: {stairs: 8.0, elevator: 15.0}
    transition_requirements: {stairs: [stairs], elevator: []}
```

任务超过20个仍执行搬移优化。两个分配优化阶段共享1秒／50000候选预算，每阶段最多接受8次搬移；预算不包含地图读取、距离预计算、初始插入、最终路线和时间协调，属于协作式检查。

点数均衡以耗时优化后的结果为基准，最大负载和总负载各允许增加10%；超界回退。它不保证点数相同，不保证每项指标优于最初插入，也不约束后续协调等待或真实执行耗时。`plan.json.allocation` 保存候选数、搬移、预算和回退结果。

分配评分使用本轮最慢机器人作参考，对每段行走代价计算：
`ceil(参考行走ticks × 参考速度 / 本机速度)`。这样区分1.0与1.5m/s，无需缩小tick；服务等待和固定跨图耗时不按速度缩放。同速编队保留原评分。**导出的 travel/load 与协调时间仍使用原粗粒度模型，不能据此宣称实际耗时已精确反映速度。**

例如参考速度1.0m/s、某路段参考代价100ticks时，1.0／1.5／2.0m/s在分配评分中分别约为100／67／50ticks；再分别加上不缩放的服务和固定过渡代价。每段单独向上取整，短路段仍可能有相同评分，这个修复是粗略估计的改善，不是连续时间最优解。

时间协调使用安全时间区间和预约，检查 ROOT 坐标下 footprint、扫掠冲突和容量为1的共享资源，终点继续占位。初始碰撞会拒绝。一次局部修复最多重排3台机器人，共享0.2秒／20000节点预算，仅接受安全、缩短总完工时间且完工时间之和不增加的结果。

footprint、clearance 缺省为0时属于点模型，不代表机器狗实体安全保证。`LiveReservationTable` 是库能力，当前 Agent 并未接入实时预约执行。真实动态避障、转弯和控制仍由导航承担。降采样功能存在但默认关闭，本系统不依赖启用降采样获取性能。

## 输出与查看

| 文件／字段 | 内容 |
| --- | --- |
| `plan.json` | 分配、停靠、坐标、路线、检查点、交通事件与统计 |
| `summary.txt` | 人类可读路线摘要 |
| `routes_<map_id>.png` | 各地图原图叠加起点、任务、路线、通道 |
| `routes_overall.gif` | 可选的时间协调回放 |
| `coordination.method` | prioritized、cbs 或 none |
| `coordination.local_repair` | 局部修复结果 |
| `timed_schedule` | 开启动画时导出的栅格时间表 |

`export.navigation_checkpoint_types` 可选 start、task、turn、resource_entry、resource_exit、transition_entry、transition_exit、holding、finish。空数组表示不导出执行检查点，不影响原分配和路径。

动画需 giflib、`planner.coordinate_conflicts=true` 与 `export.animation=true`；还可设置 animation_fps、animation_speedup、animation_max_frames、animation_width。GIF 采样可能漏掉短等待，图示不等于实机轨迹保证。单次规划预计耗时与实际任务执行历时应分开呈现。

检查输出时，按以下顺序核对：任务ID在所有stops中恰好出现一次；每项requirements被所属机器人能力覆盖；同点合并后仍保留全部原ID；执行所需的task／holding检查点没有被export筛选掉。routes中的零任务机器人不能直接理解为遗漏。

holding的arrival_tick／departure_tick表示协调等待区间，Agent会将其转换为整数秒等待；离线时间表没有实时机器人反馈闭环，不能据此假设机器狗会精确在同一绝对时刻经过交叉点。导航仍负责真实环境避障。

## 常见问题与复现方法

| 现象 | 先核对 | 处理边界 |
| --- | --- | --- |
| no capable robot for task | 输入YAML的requirements、机器人capabilities、通道能力 | 以输入为准，不从camera_type推导能力 |
| robot start resolves to a blocked or out-of-map cell | 起点map_id、局部坐标、原图栅格、阈值与净空 | 生产接口不自动移动机器人起点 |
| task location resolves to a blocked or out-of-map cell | 任务地图与坐标、position_tolerance_m | 容差不是放行真实障碍 |
| 分配数量不一致 | 起点、速度、任务耗时、能力及allocation统计 | 点数均衡是受耗时约束的次要目标 |
| 1.0／1.5m/s导出时间仍接近 | 分配评分与输出travel/load的区别 | 当前修复没有重写协调离散模型 |
| 冷运行明显慢于第二次 | Release、地图缓存命中、输出图片／动画 | 冷／热运行分开统计，不只报告最快一次 |
| CLI成功但ROS调用失败 | 绝对.yaml路径、参数数目／类型、服务进程版本与输出权限 | PC文件路径不代表板端可读 |
| CBS达到搜索上限 | 初始冲突、瓶颈和共享资源、搜索边界 | 达到上限不证明数学上无解 |

复现时从Server的output_planner_input.yaml或Leader的input_planner_input.yaml取实际输入，另存到独立目录，确认map.directory在复现机器上有效，并设置独立output_directory与cache_directory；不要覆盖原请求审计。CLI退出码、stdout／stderr、plan.json、summary.txt和PNG一起保存。

性能比较应固定源码版本、Release、输入、地图、机器负载、PNG／GIF开关和缓存条件。算法优化预算只是其中一个阶段，不是ROS Action端到端时限；网络、选举、路线就绪确认也不属于CLI计算耗时。RK3588结论必须来自板端测量，不能用PC耗时换算。

## 测试入口与证据阅读

| 测试范围 | 主要入口 |
| --- | --- |
| 能力、分配、超过20任务的搬移和速度评分 | test/allocation_tests.cpp、test/mission_tests.cpp |
| 地图阈值、坐标、缓存、跨图 | test/map_threshold_tests.cpp、test/map_cache_tests.cpp、test/offline_adapter_tests.cpp |
| 时间协调、冲突和预约 | test/coordination_tests.cpp、test/live_reservation_tests.cpp |
| 输出动画与场景 | test/plan_animation_tests.cpp、test/offline_scenario_test.cpp |

用前述CTest命令运行当前构建中的测试；可用ctest --test-dir build -N只列清单。场景相关测试可能依赖相邻场景包，查看测试发现结果，不能把未被注册的场景当作通过。完整Server／Agent契约回归入口见 [服务端验收](../mission_dispatch_server/README.md#构建与本地验收)。

场景测试默认未启用。需要覆盖相邻场景包时，在规划器包目录使用显式配置：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DBUILD_SCENARIO_TESTS=ON \
  -DSCENARIO_ROOT="$PWD/../capability_mission_scenarios"
cmake --build build --parallel
ctest --test-dir build -N
ctest --test-dir build --output-on-failure
```

测试数量还受GIF依赖、场景文件存在与构建选项影响，不能要求任意新环境都自动发现历史记录中的22项。

[test/data](test/data)中的两份JSON是2026-10-13历史测量数据，保存版本、环境及指标，供TEST_LOG引用；它们不是运行参数、预期结果金标准，也不是本次整理后新跑出的结果。

## 目录与维护

公共头文件按 core、offline、search 分组；旧根头文件保留转发兼容。实现按 src/core、src/offline/{maps,coordination,output}、src/ros 分组。新代码优先使用模块头路径，例如 `capability_mission_planner/core/mission_planner.hpp`。

历史专项文档已归并至本说明与 [测试记录](TEST_LOG.md)，原始指标归于test/data，不再保留docs目录。运行产物与缓存不应混入说明文档，基准数据必须注明版本、硬件、构建类型和统计范围。
