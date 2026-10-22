# 测试与变更记录

按日期倒序维护。历史数字只代表对应版本、环境和场景；2026-10-21、22文档整理没有重跑算法或实机测试。整套Server／Agent验收及现场回放见 [服务端测试记录](../mission_dispatch_server/TEST_LOG.md)。

## 2026-10-22：证据归类与说明补充

两份历史指标JSON从docs迁至test/data，内容逐字节保留；空docs目录删除。只改变归档位置，不改变测试注册、算法或配置。

README补充系统边界、灰度阈值换算、分配／协调时间的区别、常见错误及复现／性能比较方法。旧专题文档中的已实现内容仍以README为准，失败案例与性能边界保留在本文件，不把文档整理写成新算法验收。

| 保留证据 | 可以说明 | 不能说明 |
| --- | --- | --- |
| test/data/coordination_validation_results.json | 2026-10-13基础场景阶段耗时、内存和协调结果 | 当前版本在任意地图均同样加速 |
| test/data/coordination_extended_validation_results.json | 扩展规模、成功／失败及安全检查记录 | CBS超限场景不存在可行解 |
| Server runtime/speed_tick_diagnosis_20261021 | 速度评分修复、ARM对比及4份YAML契约 | 导出时间已成为精确行走时间 |
| Server runtime/planner_balance_20261015 | 28点均衡前后代价与预算取舍 | 每台机器狗任务数量应强制相等 |

引用JSON时先读取其版本和场景字段，再对照下面的日期记录。Server清理了重复性能运行目录，但保留比较汇总、代表性输入／输出和原始地图；历史脚本中的旧构建路径或输出目录可能需要重新准备，不能直接视为现成可运行的测试套件。


## 2026-10-21：文档归并

README统一当前构建、服务、输入、地图语义、分配、协调与输出；本文件集中历史结果和局限。旧docs文档迁移如下，原Git历史仍可查阅：

| 原文档 | 当前入口 |
| --- | --- |
| offline_planner_usage_zh、ros2_service_interface_zh | README的输入、构建与ROS2服务 |
| image_map_planning_algorithms、multi_robot_conflict_resolution_zh | README的地图及分配与时间协调 |
| nav2_multi_map_adapter_feasibility、nav2_task_level_integration_zh | README的职责边界与地图／输入说明 |
| coordination_optimization_validation_zh、coordination_extended_validation_zh | 本文件2026-10-13记录及原始JSON |

可行性旧文中已实现的功能改为当前说明，不再保留重复的未来方案。没有修改源码、配置或算法。

## 2026-10-21：速度评分与RK3588性能

基线10bf600；在X30-A独立临时目录编译候选，未替换开发板原源码或运行二进制，未启动运动。场景为VY0UgTP七地图、4机28任务、0.1m分辨率／0.1秒tick，保留PNG。

| 指标 | 基线 | 候选 |
| --- | --- | --- |
| 冷运行 | 9.972秒 | 5.588秒 |
| 热运行中位数（各2次） | 9.549秒 | 5.277秒 |
| 距离场数 | 64 | 32 |
| 峰值RSS | 约148MiB | 约132MiB |

分配数按Cyvet-A／Cyvet-B／X30-A／X30-B为7／8／7／6。参考最慢速度并逐路段折算，仅用于分配评分；服务／固定过渡不缩放，协调与导出粗时间模型未重写。

22项CTest、397项联合pytest通过；板端分配／协调测试及候选输出经过Server与Agent转换为4份YAML，并模拟完成。数据仅代表该场景，不是所有地图的性能保证。

证据：`../mission_dispatch_server/runtime/speed_tick_diagnosis_20261021/`，保留result.json、rk3588_validation_summary.json、rk3588_candidate_plan.json、agent_contract/result.json及复现脚本。

## 2026-10-20：地图阈值与现场回放

引入仅供本规划器的free／occupied阈值覆盖及缓存签名；不修改地图或导航源码。原输入可复现起点拒绝，覆盖后三份真实输入（28／9／4点）均成功。28点PC约4.12秒，四机各7点；不是ARM耗时。

最终22项CTest、397项联合pytest通过。历史测试free=0.01，当前Server配置为0.05；二者不要混淆。地图与导航语义核对、TaskHub小数等待及能力不足收尾证据见服务端TEST_LOG，起点严格检查仍保留。

## 2026-10-15：新版适配、均衡与Release

从a4c6161到5fc1eaa的初次兼容性核对：PC19项CTest、169项pytest通过，6份真实输入的旧JSON约定保持一致；本地真实C++服务配合模拟TaskHub，不是初次评估即完成ARM测试。

搬移优化移除超过20任务的跳过条件，增加有预算的点数均衡。28点案例由4／9／6／9变为5／8／7／8；总负载498.4→549.9秒（+10.33%），最大152.3→151.4秒。10%约束参考耗时优化中间结果523秒／139.1秒，因此候选增幅约5.14%／8.84%，并非相对原始插入违反上限。

X30-B临时构建对比27次CLI、12次ROS调用：基线CLI9.172秒／ROS9.095秒，候选9.395／9.235秒（+2.43%／+1.54%），局部优化约0.002～0.029秒。PC21项CTest、ARM6项相关测试、Python169项通过，原部署保持不变。

Release最终验证保留PNG，撤回默认Release构建改动、服务端新增降采样开关、额外计时日志和CLI安装方案，改用显式Release命令。四板共28次CLI、12次ROS，四机28点约9.2秒、两机18点约4.6秒；169项pytest、17项CTest及每板4项相关测试通过。旧performance记录为中途方案，已撤回，不能作为现行部署说明。

证据在服务端runtime的planner_review_5fc1eaa、planner_balance_20261015、performance_final_20261015；保留比较JSON、JUnit、输入与输出。

## 2026-10-13：时间协调优化

基线c35a9f4，场景版本5f09847，PC i9-12900H／Release。优先级预约改用安全时间区间，并增加有限局部修复。

示例协调阶段16.083→0.102秒，修复0.190秒；调度完工时间1867.6→1363.8秒（-27%），等待1505.8→652.6秒（-56.7%），RSS595→78MiB。这些是离线调度估计，不是实机执行时间。

20项CTest、400组固定种子BFS几何比较、19种顺序组合各3次通过。100任务合并为20停靠点、6机器人，6820ticks／15对轨迹；最小扫掠距离0.715542m>0.6m，最小障碍净空0.477501m>=0.3m。GIF476帧，CLI7.83秒，其中导出7.07秒。

扩展bdz04／05／06分别50／50／52位置、50／60／80任务；05协调1.2723→0.0814秒，04／06收益有限。footprint=0的场景只验证点模型。

sx06～09为6／12／12／20机，220／360／360／480任务；07调度完工1680.2→1485.4秒（-11.6%），等待2108.8→1926.2秒（-8.7%）。旧06存在扫掠间距0.565685m<0.6m，修复后耗时增加约1.6秒。

失败也保留：sx02～05均达到CBS512上限，不能视为场景无解。测试footprint0.25m、安全余量0.05m，要求中心距0.6m／净空0.3m；进程限制40秒及2GiB地址空间，后者不是RSS上限。

原始指标：[基础验证JSON](test/data/coordination_validation_results.json)、[扩展验证JSON](test/data/coordination_extended_validation_results.json)。基础JSON保存阶段测量、顺序对比和安全核对；扩展JSON保存规模覆盖及结果。完整字段以原始JSON为准，README中的简表不替代这些记录。
