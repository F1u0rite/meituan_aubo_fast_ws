# 美团 AUBO S3｜真实任务目标与坐标轨迹实施方案

> 2026-10-14，交接手册 v1.1 第 13 章独立版。前序实机连接 SOP、详细测试证据见完整交接手册。

> **下面是下一位同学应当实现的工程方案，而不是已经完成的软件功能。** 所有当前未实测的 Z 高度、插入方向和位移仅用于说明算法，严禁直接把文档示例送到实体机器人。**当前成功的 Base +Z 20 mm MoveL 是控制器原生 RPC 接口能力证明，不等于真实电池抓放可执行。**

## 13.1 项目的最终可验收目标

### 目标一：给一个准确的三维目标，机器人可以到达

输入：

```yaml
request_example:
  frame_id: base_link                 # 真实、已标定坐标系
  target_position_m: [X, Y, Z]       # 数值来自标定/测量，而不是图纸估计
  target_orientation_xyzw: [qx, qy, qz, qw]
  motion_policy: "FREE_SPACE_or_CARTESIAN"
  execute: false                     # 默认只做规划；不授权实体运动
```

要求：完成 `base_link` 坐标中的目标位姿验证、IK、整条路径碰撞检查、轨迹时间参数化和低层执行前校验。**XYZ 三个值不足以唯一确定六轴机械臂末端运动，必须同时指定方向/姿态约束；默认保持当前姿态也需要检查可达性。**

### 目标二：给“电池 A → T0”，自动生成可执行的安全任务计划

输入为源电池编号、目标区域和场景的**标定三维数据**。系统应生成：

```text
READ_SCENE → VALIDATE_CALIBRATION → PLAN_PREGRASP
    → APPROACH → INSERT → VERIFY_ENGAGEMENT
    → LOAD_TEST / LIFT → PLAN_FREE_SPACE_TRANSFER
    → MOVE_TO_PREPLACE → LOWER → VERIFY_SUPPORTED
    → WITHDRAW → RETREAT → VERIFY_PLACED → DONE
```

入环、提升、释放必须有失败判定与恢复；不要把“走完关节轨迹”误认为“已经成功抓住并放好电池”。

### 目标三：按比赛指令识别颜色、选择顺序并连续执行

应实现 A/B/C/D 与颜色的动态关联、任务口令/目标序列解析、对 T0/P1/P2/P3 的正确分配、相机外参与目标位置更新、状态机、失败重试策略及比赛视频记录。实际颜色/摆放可以变化，不能把 A 固定等同某种颜色。

## 13.2 已知任务板上的具体 XY 点——官方图纸的**绘图估计参考点**

坐标定义：

```text
frame: task_board_illustration
origin: 底图左下角（纸面原点）
+x: 向右       +y: 向上        +z: 垂直纸面向外
unit: 米
```

**这些是 `5F-1 config/*.yaml` 当前加载的绘图估计数值，并非主办方给定的精确中心坐标，也不是测量得到的 `base_link` 位置。**

**基础赛 `task_board_basic.yaml`：**

| 参考对象 | X (m) | Y (m) | Z (m) | 平面尺寸/备注 |
|---|---:|---:|---:|---|
| `battery_A_planar_nominal` | 0.178 | 0.446 | 0 | 电池本体参考中心，70×80 mm |
| `battery_B_planar_nominal` | 0.257 | 0.446 | 0 | 电池本体参考中心，70×80 mm |
| `battery_C_planar_nominal` | 0.178 | 0.359 | 0 | 电池本体参考中心，70×80 mm |
| `battery_D_planar_nominal` | 0.257 | 0.359 | 0 | 电池本体参考中心，70×80 mm |
| `target_T0_planar_nominal` | 0.508 | 0.400 | 0 | T0 区域参考中心，200×100 mm |
| `battery_zone_planar_nominal` | 0.218 | 0.415 | 0 | 300×250 mm |
| `base_reference_planar_nominal` | 0.393 | 0.100 | 0 | 图上底座参考圆中心；**不是 base_link 的实测位置** |

**基础赛图纸中心的几何偏移（供任务距离估算，不是实体运动命令）**：

| 起点 | 终点 T0 减起点（板坐标，mm） | 平面中心距约 (mm) |
|---|---|---:|
| A | (+330, −46) | 333.2 |
| B | (+251, −46) | 255.2 |
| C | (+330, +41) | 332.5 |
| D | (+251, +41) | 254.3 |

**进阶赛 `task_board_precision.yaml`（与 Basic 为两张不同示意布局）**：

| 参考对象 | X (m) | Y (m) | Z (m) |
|---|---:|---:|---:|
| 电池 A / B | 0.140 / 0.219 | 0.446 | 0 |
| 电池 C / D | 0.140 / 0.219 | 0.359 | 0 |
| `target_P1_planar_nominal` | 0.435 | 0.464 | 0 |
| `target_P2_planar_nominal` | 0.610 | 0.464 | 0 |
| `target_P3_planar_nominal` | 0.610 | 0.289 | 0 |

进阶目标外框尺寸 150×150 mm、内框 80×80 mm。**中心点只是当前图示坐标模型的估计，不能不经校准就拿来做精密放置。**

## 13.3 尚需测量/标定的“真正三维坐标”

原 `5F-1` 中所有电池/目标的 `z=0` **是板面，不是提环高度**。真实任务至少需要以下标定量：

| 量 | 坐标/变量 | 如何测量/标定 | 完成状态 |
|---|---|---|---|
| 机器人 → 实体板面 | `T_base_board`，6D | 板上至少 3 个非共线基准点、板面法向、机器人基座方位；使用测量治具/相机标定求解 | **未完成** |
| 每块电池相对板面位置 | `T_board_battery_i` | 电池中心和本体坐标轴、姿态；相机/测量尺/治具 | **未完成** |
| 每块提环中心和轴线 | `T_battery_ring_i` | 测环中心高度、开孔长宽、环平面、允许舌规进入的方向 | **未完成** |
| 实际工具挂点/承托点 | `T_ee_tool_tcp`、`T_tool_tcp_hook_entry` | 法兰安装方向、V1 CAD 与实物对照、TCP 标定 | **未完成** |
| 插入时的有效抓取刚体关系 | `T_tcp_battery_at_grasp` | 工具与提环结合后的实测相对变换、受力与晃动/滑移试验 | **未完成** |
| 放置槽目标姿态 | `T_board_slot_j` | 目标区板面+槽口高度、支撑方式、允许放置的角度 | **未完成** |
| 障碍与负载 | `scene_collision`, `payload` | 工作台、电池、相邻电池、机械臂、工具体积，电池质量重心 | **未完成** |

**关键数学关系：**

```text
T_base_ring   = T_base_board · T_board_battery · T_battery_ring
T_base_tcp    = T_base_ring · T_ring_hook_entry_goal · inverse(T_tcp_hook_entry)
T_base_object = T_base_tcp · T_tcp_battery_at_grasp        (挂住后)
T_base_tcp_place_goal = T_base_battery_place_goal · inverse(T_tcp_battery_at_grasp)
```

`T_A_B` 表示把 B 坐标系中的点变换到 A 坐标系；`inverse` 必须是完整刚体变换的逆（旋转和平移一起），**不可只做 XYZ 加减**。最后两式成立还需要近似刚性、无滑移的抓取关系；被动舌规如果会转动/滑移，必须引入实际约束或状态估计。

> V1 的 CAD 名义定义：`tool_tcp → hook_entry` 约沿工具局部 +X **38.562 mm**；这只是 CAD 设计值。实际 `ee_link → tool_tcp` 仍缺少安装姿态与物理 TCP 标定。**严禁在实机抓取中直接将 CAD 默认坐标当作已验收标定。**

## 13.4 一条带有具体数值的 A → T0 轨迹草案（**示范坐标，绝对禁止原样执行**）

为了让接手者清楚“应当给出怎样的一系列坐标点”，以下先把所有点放在**互不连接真实机器人 TF 的** `task_board_example` 坐标系中，只用于几何、消息和算法单元测试。

**演示假设（并非测量事实）**：

- 取 Basic 图纸 A 中心 `(0.178, 0.446)`、T0 中心 `(0.508, 0.400)` 米；**这些 XY 是图纸估计**。
- 假定“演示入环虚拟参考点”的板面高度为 `z=0.050 m`；**真实提环中心高度未知**。
- 假设演示插入轴是板面 `+X`，从电池左边往右进入；**真实环平面和舌规安装角度未知**。
- 假定入环参考点前后各有 30 mm 接近空间、入环后前进 15 mm；仅用于说明坐标序列，**不代表实际尺寸和避障距离**。
- 假设演示抬升高度为 `80 mm`，演示中转空中高度 `170 mm`；**未经机械臂可达性/障碍/负载检查**。
- 表中追踪的点是**抽象的 `hook_entry` 虚拟参考点**，不是 `ee_link` 或机械臂控制器当前 TCP；需要通过标定转换才能用于规划。

| 序号 | 状态名 | `hook_entry` 演示点 XYZ (m) | 推荐段类型 | 运动目的 |
|---|---|---|---|---|
| W0 | `PRE_INSERT` | **(0.148, 0.446, 0.050)** | 自由空间 `MoveJ` 到附近 + 末段对齐 | 预接近，距假定环中心 −X 30 mm |
| W1 | `AT_RING_ENTRANCE` | **(0.178, 0.446, 0.050)** | 受约束 `MoveL` | 到假定环入口中心 |
| W2 | `ENGAGED` | **(0.193, 0.446, 0.050)** | 受约束 `MoveL` | 沿 +X 再深入 15 mm |
| W3 | `LIFTED` | **(0.193, 0.446, 0.130)** | 受约束 `MoveL` | 从 W2 上提 80 mm（需先验证接触和负载） |
| W4 | `CRUISE_START` | **(0.193, 0.446, 0.170)** | 自由空间过渡，保持电池安全包络 | 提至示意中转高度 |
| W5 | `CRUISE_END` | **(0.523, 0.400, 0.170)** | 规划避障 `MoveJ`（TCP 路径不保证直线） | 平移至示意 T0 上方 |
| W6 | `PRE_PLACE` | **(0.523, 0.400, 0.130)** | 目标对齐 `MoveL` | 准备下降 |
| W7 | `SUPPORTED` | **(0.523, 0.400, 0.050)** | 受约束 `MoveL` | 放到假定支撑高度（需验证电池由台面承载） |
| W8 | `WITHDRAWN` | **(0.508, 0.400, 0.050)** | 受约束 `MoveL` | 沿 −X 退出 15 mm |
| W9 | `RETREAT` | **(0.478, 0.400, 0.050)** | 慢速退出后自由空间退避 | 拉开与目标的距离 |

**这张表的 W5/W7 只是示例 `hook_entry` 坐标，不代表“电池本体中心最终在 T0 中心”**。真实放置点要以测得的 `T_tcp_battery_at_grasp` 将**电池本体中心**对齐 T0/槽位中心；如果提环中心不在电池中心或工具抓取中会滑动，实际工具点将与表中不同。

**同样重要：这里所有姿态都未赋真实四元数**。只有 XYZ，尚不能直接作为六轴 IK 输入；必须测出真实提环轴向，生成 `R_board_hook_goal`，确保工具局部插入轴与提环通道一致，还要检查舌规承托面的朝向及退出时是否会勾住提环。

### 13.4.1 从板坐标到机器人坐标的明确步骤

对于任意上述示意点 `p_board=[x,y,z,1]^T`：

```text
p_base = T_base_board · p_board
```

一旦标定拿到 `T_base_board` 才能产生 `base_link` 中的数值。例如 W0 在板坐标是 `(0.148,0.446,0.050)`；**我们现在无法给出其真实 `base_link` XYZ**，因为 NUC 上只有与 `base_link` 隔离的绘图参考 TF。**绝对不要把 `base_reference`（图纸上的圆）直接当 `base_link`，也不要自行假定两坐标系旋转为零。**

### 13.4.2 运动段的正确策略

```text
S0: 实时测得的 q_current, T_base_ee_current
 S1: 规划【避障 MoveJ】到 W0 对应的真实 6D pre_insert
 S2: 规划【局部 MoveL】W0→W1，先校验环入口和工具扫掠
 S3: 规划【受约束 MoveL】W1→W2，确认已入环、无挤压
 S4: 规划【竖向 MoveL】W2→W3，观察承载/机械臂外力
 S5: 规划【附带电池碰撞体的自由空间 MoveJ】W3→W4→W5
 S6: 规划【姿态对齐 + MoveL】W5→W6→W7，确认电池受支撑
 S7: 规划【精确 MoveL】W7→W8→W9，保证工具确实退出
 S8: 检查电池位置/颜色/稳定性 → 下一块
```

`W3→W4→W5` 要使用**真实带负载碰撞几何的避障规划**，不应将 330 mm 的横向运输硬塞进当前 5D 的固定姿态单直线 IK 中。尤其进入/退出提环时需要允许接触的是哪个零件、接触力可能是多少，要由机械+控制共同确认。

## 13.5 现有代码能复用哪些、新代码具体应该怎么写

现有 ROS 2 包仍是 `~/meituan_aubo_fast_ws/src/mtc_control`（15 个 C++ 程序）；建议**在其旁新增模块**，不破坏已经 PASS 的 Mock 基线。

| 建议新增模块/文件 | 输入 | 输出 | 必须实现的要点 |
|---|---|---|---|
| `board_calibration_node.cpp` | 真实板面 3D 基准点、机器人基坐标 | 经验证的 `T_base_board` 和标定版本号 | 至少 3 非共线点、基准点残差、法向/朝向和单位检查；**未标定就不发布“真实” TF** |
| `ring_pose_node.cpp` | 实测或视觉电池/环位姿 | `T_board_ring_A..D`、质量评分 | 检查姿态、开孔/尺寸；区分人工示教值和实时视觉值 |
| `task_pose_builder.cpp` | ring、slot、工具 `T_ee_tcp`、抓取关系 | `pre_insert / inserted / lift / pre_place / withdraw` **PoseStamped** | 用完整 `SE(3)` 变换；保留插入轴、承托方向约束 |
| `planning_scene_manager.cpp` | URDF、工具 mesh、台面、电池 mesh、槽位、障碍物 | 已更新规划场景 | 机器人自碰撞、工具-桌面、工具-电池、其他物体；抓取后 attach 电池碰撞体，放下后 detach |
| `task_motion_planner.cpp` | `PoseStamped`、关节状态、规划场景 | 各阶段轨迹与完整 plan manifest | 接近和退出笛卡尔；长距离采用 MoveIt2 OMPL/Pilz 等**经工程验证后**的规划器；拒绝规划不完整和奇异/超限轨迹 |
| `real_trajectory_executor.cpp` | 单独审核过的完整轨迹 + 现场确认 | 真实控制器执行状态/反馈 | 自研独立实机通道，不修改旧 `GenericSystem` Mock executor；避免 RPC 与 ros2_control 两个控制客户端竞争；实时起点、速度/加速度、硬件身份、状态和超时门控 |
| `grasp_task_fsm.cpp` | 指令、视觉/人工目标、规划与反馈 | 抓取→放置任务状态与错误码 | 与实体传感、力/负载、视觉验证衔接；可停止和恢复 |
| `vision_battery_node.cpp` | 相机图像/深度、标定外参 | 电池颜色、目标位置、置信度 | 不要假定 A/B/C/D 永远对应相同颜色；执行前重新定位 |

MoveIt 2 / OMPL/Pilz 等规划组件**还没有在本项目安装、配置与联调**。应先检查系统软件，再决定是使用 MoveIt 2 的规划场景和规划器，还是在现有 KDL 上加入等价的碰撞与轨迹时间参数化功能。**不能为了赶进度直接把 IK 解当安全轨迹。**

### 13.5.1 任务数据结构（建议 `config/task_targets.yaml`，**仅模板，缺值严禁执行**）

```yaml
# 建议的应用级数据结构，不是现有 ROS 参数文件；尚未实现加载器。
mode: preview_only
board:
  frame_id: task_board_measured
  calibrated: false
  T_base_board: null             # 必须测量后填 4x4 或 xyz+quaternion
batteries:
  A:
    nominal_board_xy_m: [0.178, 0.446]  # 仅绘图估计
    measured_board_pose: null           # 必须实测
    ring_pose_in_battery: null          # 必须实测，带方向
  B:
    nominal_board_xy_m: [0.257, 0.446]
    measured_board_pose: null
    ring_pose_in_battery: null
  C:
    nominal_board_xy_m: [0.178, 0.359]
    measured_board_pose: null
    ring_pose_in_battery: null
  D:
    nominal_board_xy_m: [0.257, 0.359]
    measured_board_pose: null
    ring_pose_in_battery: null
targets:
  T0:
    nominal_board_xy_m: [0.508, 0.400]
    measured_board_pose: null
  P1:
    nominal_board_xy_m: [0.435, 0.464] # 注意这里来自 Precision 图
    measured_board_pose: null
  P2:
    nominal_board_xy_m: [0.610, 0.464]
    measured_board_pose: null
  P3:
    nominal_board_xy_m: [0.610, 0.289]
    measured_board_pose: null
tool:
  actual_T_ee_tool_tcp: null
  actual_T_tcp_hook_entry: null
  verified_load_and_clearance: false
execution:
  allow_real_motion: false
  collision_scene_validated: false
  operator_approved: false
```

运行时应在 `allow_real_motion == false`、任一关键标定值为空、TF 超时/跳变或碰撞校验缺失时**拒绝执行**，而不是用零值填补。单条抓放计划保存 `plan_id`、时间戳、机器人和工具标定版本、目标位姿与碰撞场景哈希，以避免将过期 Mock 轨迹重新执行在真实机器人上。

## 13.6 实现规划算法：从目标 Pose 到 JointTrajectory

每个段至少包含以下检查：

1. **目标生成**：根据 `T_base_board`、电池/环位姿、工具标定 `T_ee_tcp` 计算 `T_base_tcp`，转换到运动学使用的 `ee_link` 位姿；姿态必须正确。
2. **当前状态获取**：使用控制器最新关节角和时间戳，确认没有重复控制器/状态发布者；现在的 RPC `/joint_states` 桥接约 5 Hz，只宜用于状态/规划输入，**不适合作为伺服闭环控制频率**。
3. **段类型选择**：自由空间使用避障 MoveJ / 经规划器生成的关节轨迹；近距离入环/退出使用约束姿态的笛卡尔局部路径，不能只验证末端目标。
4. **逐点 IK 和奇异性**：逐点使用上点作种子或分支候选，检验完整 6D FK 残差、关节限位、连续性、Jacobian 最小奇异值/条件数；遇到局部病态则换构型/规划路线，不能靠扩大速度阈值放过。
5. **碰撞校验**：检查整条路径（包括关节插值的中间状态）、工具 swept volume、台面、其余电池及被抓电池；只有明确允许的工具-提环接触才能得到对应的碰撞例外。
6. **时间参数化**：为轨迹计算合理速度、加速度及必要的 jerk 限制；段间不允许突然跳速。按实际 S3/控制器限制设定，不照搬 Mock 中的 12 s 参数。
7. **执行前双重检查**：收到操作授权后重新读取 `q_current`，检查距轨迹起点的误差，重新确认 `Running/Normal`、控制权独占、场景未变化、工具和工件处于正确状态。
8. **控制器执行与监测**：使用经验证的真机接口；获取接受/拒绝、运行中、停止/故障、反馈误差和结束状态。RPC MoveL 可以用来验证简单直线段，但不能冒充已实现任意多阶段自定义轨迹流控。
9. **任务结果核验**：入环可靠、无滑移、提起成功、目标区域到位、完全脱钩，才能宣告单块电池任务完成。

**特别提醒：** 机器人关节插值 `q(t)` 即使连续，末端轨迹也未必是直线；`moveJoint` 不能被称为 MoveL。反过来，原生 `moveLine` 控制的 TCP 也未必就是 V1 舌规真正的 `hook_entry`，除非控制器 TCP 与我们标定工具点一致。

## 13.7 真实任务的验收顺序（务实版本）

| 顺序 | 必须交付/实际可测的产物 | 通过条件 | 当前状态 |
|---:|---|---|---|
| 0 | NUC↔控制柜连接，MoveJ 和 Base +Z 20 mm 原生 MoveL | 实体反馈/现场验证，原始日志归档 | **已有现场成功记录；20 mm 完整日志待补** |
| 1 | `calibration/board.yaml` + `calibration/tool.yaml` | 实测外参、方向/单位与误差复核、检查校准有效期 | 未完成 |
| 2 | `calibration/ring_slots.yaml` + 碰撞几何 | 电池环/槽/台面高度、姿态与间隙均测量并复核 | 未完成 |
| 3 | 实测目标预览 / RViz 机器人与底图对齐 | 全部目标为 `base_link` 中的 6D Pose，现场参照一致 | 未完成 |
| 4 | 单块电池**无接触**路径的碰撞验证 | 自碰撞/工具/桌面/其余电池均通过路径校验 | 未完成 |
| 5 | 在人员监护下逐段验证空载非接触运动 | 真机进出位置与期望方向一致，停止机制可用 | 未完成 |
| 6 | 舌规入环、提升和可靠脱钩 | 机械承托有效、碰撞/受力/挂脱验证通过 | 未完成 |
| 7 | A→T0 一块完整实机抓放 | 电池真正移动且稳定放置，记录视频及末端/关节日志 | 未完成 |
| 8 | A/B/C/D 到 T0 / P1/P2/P3 | 连续多块、不碰撞、满足精度与顺序要求 | 未完成 |
| 9 | 颜色识别 + 任务状态机 + 全流程比赛验收 | 根据指令自主选目标、异常可恢复、连续完整演示 | 未完成 |

### 优先级建议（若团队时间有限）

**先完成“一块真实电池 A→T0”，再扩展视觉和多块任务**。当前最大阻塞是**工具与提环/电池的三维关系、`T_base_board`、真实碰撞场景以及可靠实机轨迹执行**，而不是 ROS 通信和 IK 基础功能。

## 13.8 下一位同学可以立刻执行的 SOP（安全地开始实物开发）

```bash
# 只读联调；不要在机器人可能仍处于运动时启动多个控制器。
source /opt/ros/jazzy/setup.bash
source ~/aubo_driver_validation_ws/install_no_symlink/setup.bash
source ~/meituan_aubo_fast_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 官方 RPC 客户端：保持在独立终端运行
ros2 launch aubo_ros2_driver aubo_client.launch.py \
  robot_ip:=192.168.3.100 log_level:=info

# 只读关节反馈和 URDF：保持另一终端运行
ros2 launch ~/Downloads/mtc_real_readonly_v1/mtc_real_readonly.launch.py

# 另一终端先做只读状态与 FK 自检
ros2 topic echo --once /joint_states sensor_msgs/msg/JointState
ros2 run mtc_control kinematics_node

# 绘图示意场景（有意没有 base_link 连接）：仅供 RViz/算法单元测试
ros2 run mtc_control task_board_node --ros-args \
  --params-file ~/meituan_aubo_fast_ws/src/mtc_control/config/task_board_basic.yaml
```

下一步**不要**把 `task_board_illustration` 接到实体 `base_link`，也**不要**启动旧 `tcp_sequence_executor` 试图控制真机。先创建校准文件，要求所有未知位姿保持 `null` 并禁止运动；场景建成后再接入各段安全规划与真机动作接口。
