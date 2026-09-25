# omni_decision_sample — 哨兵导航调度器

> 最后更新: 2026-07-14 | 目标机器人: 全自动哨兵 (Sentry) | 赛季: RM2026

---

## 1. 定位

`omni_decision_sample` 是一个**纯导航调度器**。它只回答一个问题：**哨兵下一步往哪走。**

它不参与战斗（自瞄独立运作），不控制姿态（电控负责），不感知敌人。它只从裁判系统读取血量、弹药和前哨站状态，然后决定去巡逻还是回补给区。目标机器人是**全自动哨兵**（满血 400），弹药兑换/复活确认/姿态切换等全部交给电控，本模块只回答"下一步往哪走"。

### 与原版 `sentry_decision` 的关系

```
原版 sentry_decision (1,900 行)   → 完整 FSM，包揽导航+战斗+姿态，保留作参考
精简版 omni_decision_sample     → 纯导航 FSM，只调度目的地，当前主线
```

### 设计取舍

| 决策模块做 | 决策模块不做 |
|-----------|-------------|
| 血量低(hp<150)→去补给区 | 看到敌人→追击（自瞄独立） |
| 弹药低(≤50)→去补给区 | 被攻击→反击（电控+自瞄） |
| 补满(hp满且弹药≥100)→出去巡逻 | 主动兑换弹药 / 确认复活（电控） |
| 有血有弹→巡逻 | 切换进攻/防御姿态（电控） |
| 我方前哨存活→前压路线 / 我方前哨亡→半场防守 | 判断该不该开火（自瞄） |
| 开局→去打点位让自瞄打敌方前哨站（一次） | — |
| 阵亡复活后持续导航回补给区 | 接收雷达/自瞄数据 |
| 过起伏路段→舵轮纯直线开环冲过（绕过 Nav2） | — |

---

## 2. 输入源

| 数据源 | 话题 | 用途 |
|--------|------|------|
| 裁判系统-比赛状态 | `referee/game_status` | 判断比赛是否运行、剩余时间 |
| 裁判系统-机器人状态 | `referee/robot_status` | 当前血量、弹药（血量上限串口不上报，用配置 `max_hp`） |
| 裁判系统-RFID | `referee/rfidStatus` | 补给区到达的**辅助**确认（主判据是位置到达） |
| 裁判系统-全体血量 | `referee/all_robot_hp` | 前哨站是否存活、己方基地血量 |
| 里程计 | `odometry` | 己方位置（经 tf2 转到 map 系，用于位置到达检测） |

共 **5 个订阅**。

## 3. 输出

| 输出 | 目标 | 说明 |
|------|------|------|
| Nav2 goal | `navigate_to_pose` action | 导航目标点（主通道） |
| `/goal_pose` | PoseStamped topic | Nav2 不可用时的 fallback |
| `cmd_vel_chassis` | Twist topic | **仅过起伏段**：绕过 Nav2 直发底盘开环速度（见 8.6） |

常规导航走 Nav2 action（降级到 topic）；过起伏段临时直发底盘速度（cancel Nav2 后底盘话题自然静默，不打架）。

---

## 4. 顶层状态机

> 📊 图形化流转（ASCII + Mermaid 双份）+ 实战时序见 [docs/状态流转图.md](./docs/状态流转图.md)。

### 4.1 优先级链（每 tick 重评估，命中第一个满足的即返回）

```
优先级从高到低：

① IDLE           ← 裁判数据失效（超过 3 秒）或比赛未运行（也是唯一能打断穿越的状态）
② BUMP_TRAVERSE  ← 与目标分处某起伏段两侧且靠近入口 → 开环冲过；穿越中(未DONE/FAILED)不可被②以下任何状态打断
③ RESUPPLY       ← hp < 150 或 ammo ≤ 50 → 进入；退出需 hp 回满 400 且 ammo ≥ 100
                   （会抢断 OPENING_STRIKE，抢断即消费掉开局打点）
④ OPENING_STRIKE ← 比赛开局、配了打点位、且本局还没打过 → 去打点位停留打前哨站
⑤ PATROL         ← 以上都不满足时的默认状态
```

**迟滞设计**：进入用 `hp<150 / ammo≤50`，退出用 `hp满 / ammo≥100`，进出阈值分离防止边界抖动。弹药靠补给区每分钟被动 +100 恢复，一个免费周期即可从 50 补过 100。

**复活兜底**：哨兵阵亡时 `hp=0`，会留在 RESUPPLY；复活后（hp 从 0 恢复但仍 <150）继续保持 RESUPPLY 并持续导航回补给区，直到恢复满。这是"永不放弃"设计的直接结果。

**穿越抢占屏蔽**：BUMP_TRAVERSE 进入 DASHING 后，除 IDLE（比赛结束/裁判断连→立即零速）外不接受任何抢占——受击、血弹不足都不打断。中途停车 = 卡在波浪谷里。这道屏蔽有**两层**（select_state 强制返回 BUMP + can_leave_current_state 拒绝离开），纵深防御。

### 4.2 状态切换防振荡 (can_leave_current_state)

- IDLE 可以随时离开；进入 IDLE 总是允许（也用于穿越急停）
- **BUMP_TRAVERSE 穿越中（未 DONE/FAILED）只有 IDLE 能打断**；DONE/FAILED 后才放行交还业务状态
- RESUPPLY 可以立即抢断 PATROL（血/弹不足优先）
- 其他状态间切换需要 `min_ticks_in_state`（默认 4 ticks = 400ms）

### 4.3 状态行为

| 状态 | 导航目标 | 说明 |
|------|---------|------|
| IDLE | 无（取消所有导航） | 裁判断连或比赛未运行时原地等待 |
| OPENING_STRIKE | `opening_strike` 打点位 | 开局去打点位停留 `opening_strike_duration_s`(默认90s) 让自瞄摧毁敌方前哨站；被 RESUPPLY 抢断或时长到即消费，**每局一次** |
| BUMP_TRAVERSE | `bump_segments` 起伏段 | 舵轮纯直线恒速开环冲过波浪地形，绕过 Nav2 直发 `cmd_vel_chassis`；位置感知触发，见第 8.6 节 |
| PATROL | `patrol` / `patrol_aggressive` 路线循环 | **我方**前哨站存活走 `patrol_aggressive`（前压），被打掉走 `patrol`（我方半场防守）；只看我方前哨站，与敌方无关。路线切换时重置路点追踪 |
| RESUPPLY | `supply`（+ 可选 `backup_supply_points` 轮换） | 位置到达为主/RFID 为辅，确认到达后停留恢复；未到达则持续导航，卡住/超时就轮换候选点再回主点，**永不放弃**。满血且弹药≥100 后离开 |

---

## 5. 代码架构

```
omni_decision_sample/
├── CMakeLists.txt
├── package.xml
├── include/omni_decision_sample/
│   ├── types.hpp          # 枚举、结构体、阈值(含 BumpPhase/State/Thresholds)
│   ├── context.hpp        # Context: 比赛状态聚合器
│   ├── profile.hpp        # Profile: YAML 加载(含 BumpSegment)
│   ├── arrival_tracker.hpp # ArrivalTracker: 无ROS的到达判定(防feedback抖动)
│   ├── fsm.hpp            # DecisionFsm: 状态机声明
│   └── decision_node.hpp  # DecisionNode: ROS2 节点声明
├── src/
│   ├── profile.cpp        # YAML 解析 + 起伏段校验
│   ├── fsm.cpp            # FSM 实现(含 behave_bump_traverse)
│   └── decision_node.cpp  # 节点: 订阅、action client、fallback、cmd_vel
├── config/profiles/
│   ├── rmuc_red.yaml / rmuc_blue.yaml
├── launch/
│   └── omni_decision_sample_launch.py
└── test/
    ├── fsm_test.cpp             # 状态机 32 个
    ├── arrival_tracker_test.cpp # 到达判定 11 个
    └── bump_test.cpp            # 过起伏路段 11 个
```

### 5.1 依赖层次（单向，上层不依赖下层）

```
types.hpp  ← 纯数据，零依赖
profile.hpp → types.hpp
context.hpp → types.hpp + rm_interfaces
fsm.hpp → context.hpp + profile.hpp
decision_node.hpp → context.hpp + fsm.hpp + ROS2
```

- `Context`：零 ROS 依赖，可脱离 ROS 单独测试
- `DecisionFsm`：零 ROS 依赖，通过 `std::function` 回调与节点解耦
- `DecisionNode`：唯一的 ROS 层，负责订阅、发布、Action 客户端

### 5.2 types.hpp — 核心类型

| 类型 | 说明 |
|------|------|
| `Waypoint {x, y, dwell_s}` | 导航目标点 |
| `Route = vector<Waypoint>` | 路点序列 |
| `BumpSegment {entry, exit, yaw}` | 起伏段（entry/exit 同 y，沿 x 轴直线） |
| `State` (enum) | IDLE / OPENING_STRIKE / BUMP_TRAVERSE / PATROL / RESUPPLY |
| `BumpPhase` (enum) | GOTO_ENTRY / ALIGN / DASHING / DONE / FAILED |
| `BumpDir` (enum) | FORWARD / BACKWARD |
| `NavStatus` (enum) | IDLE / MOVING / ARRIVED / FAILED |
| `Thresholds` | 18 个可配置阈值，全部有默认值 |

### 5.3 context.hpp — 世界模型

无 ROS 依赖，纯 C++ 数据聚合。关键方法：

**裁判相关**：`game_running()`, `remain_time()`, `referee_fresh()`, `hp()`, `max_hp()`, `hp_low()`, `hp_full()`, `ammo()`, `ammo_low()`, `ammo_ok()`, `needs_resupply()`, `resupply_done()`

**场地相关**：`on_supply_pad()`, `outpost_alive()`, `ally_base_hp()`

**导航相关**：`nav_status()`, `nav_failed()`, `goal_reached()`, `set_nav_status()`

**位置相关**：`sentry_x()`, `sentry_y()`, `sentry_pos_valid()`, `set_sentry_position()`

### 5.4 fsm.cpp — 核心 FSM

- **tick()**：select_state → can_leave_current_state → on_exit/on_enter → run_behaviour
- **select_state()**：5 级优先级链（IDLE / BUMP_TRAVERSE / RESUPPLY / OPENING_STRIKE / PATROL），每 tick 重评估；先定业务状态+目标 x，再判断是否有起伏段挡在中间
- **behave_opening_strike()**：开局导航到打点位，停留 `opening_strike_duration_s` 让自瞄打前哨站；时长到或被 RESUPPLY 抢断即置 `opening_done_`，本局不再触发
- **behave_bump_traverse()**：5 阶段过起伏段（GOTO_ENTRY→ALIGN→DASHING→DONE/FAILED），舵轮恒速开环冲，见 8.6。用 `for(;;)` 包 switch 使阶段转换当 tick 生效
- **behave_patrol()**：按**我方**前哨站状态二选一路线（存活前压 / 被打掉半场防守），路线切换时重置路点追踪，卡住跳点
- **behave_resupply()**：位置到达（`goal_arrived_`）为主、RFID 滑动窗口为辅，任一确认即原地待命回血；未到达则持续导航，nav_failed 或单点超时轮换候选点（主点↔备用点循环），**永不放弃**，天然覆盖复活回归

---

## 6. 鲁棒性设计

### 6.1 防振荡

- **HP/弹药迟滞**：进入 hp<150 或 ammo≤50，退出需 hp满且 ammo≥100，进出阈值分离防边界抖动
- **min_ticks_in_state**：PATROL 切换需停留 4 tick（400ms），防止瞬态抖动（RESUPPLY 抢断不受限，血/弹优先）
- **路线切换重置**：巡逻路线切换时重置 `path_idx_`/`goal_sent_`，避免拿新路线索引判断旧目标的到达/超时

### 6.2 超时安全网

| 超时 | 默认值 | 作用 |
|------|--------|------|
| `stuck_timeout_s` | 10s | 单个巡逻点超时→跳过下一个点 |
| `resupply_timeout_s` | 30s | 单个补给点超时→轮换到下一候选点（循环，不放弃） |
| `bump_timeout_s` | 20s | 冲起伏段超时→反向退回入口，标记该段本局禁用 |

> 注：旧版的"总超时后原地放弃"逻辑已移除。RESUPPLY 只要未恢复满就持续导航，确保阵亡复活后一定能回补给区。

### 6.3 数据安全

- **裁判 stale 超时**：3s 无数据→IDLE，取消所有导航
- **所有数据访问**：`std::optional` 保护，null 时返回安全默认值
- **线程安全**：依赖 `SingleThreadedExecutor`，所有回调和 timer 在同一线程串行

### 6.4 导航容错

- **Nav2 action 可用**：走 Action 协议（feedback/result 回调，goal_id 比对防 stale）
- **Nav2 action 不可用**：自动降级到 PoseStamped topic + 位置距离到达检测
- **坐标系一致性**：odometry 在 odom 系、目标在 map 系，节点用 tf2 把位置转到 map 系再算距离（TF 不可用时安全降级用原值 + throttle 警告），避免 map→odom 漂移污染 fallback 到达判定
- **导航失败**：`nav_failed()` → 跳下一个路点或轮换补给候选点
- **cancel_nav**：始终 `async_cancel_all_goals`，fallback 模式下正确清理标志位

### 6.5 对上游坏数据的容错（决策不依赖可能出错的外部信号）

决策的核心闭环（去补给→回血→满了出去）完全由己方血量/弹药数值 + 自算的位置到达驱动，不依赖任何一个可能被上游喂错的辅助信号：

| 潜在坏输入 | 决策的应对 |
|-----------|-----------|
| 串口不上报血量上限（maximum_hp=0） | 用配置的 `max_hp`(400) 判满血，不读串口该字段 |
| RFID 补给区位被上游错映射 | **位置到达为主**（Nav2/odom，自算），RFID 仅作辅助确认，任一为真即到达 |
| 裁判数据断流（>3s） | 进 IDLE，不拿过期数据决策 |
| odom 与目标坐标系不一致 | tf2 转到 map 系再算距离 |
| 阵亡（hp=0） | 保持 RESUPPLY，复活后继续导航回补给区（永不放弃） |

> **到达补给区判定**：主判据 = `goal_arrived_`（Nav2 feedback 到达 / fallback 位置距离）；辅判据 = RFID 5-tick 滑动窗口（≥3 命中）。两者任一确认即原地待命回血，到达后不再重发目标（无 30s 抖动）。这样即使上游 RFID 字段映射错误，决策仍能靠位置到达正常工作。

---

## 7. 可配置参数

### YAML thresholds（默认值，可在 profile YAML 中覆盖）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_hp` | 400 | 满血目标（全自动哨兵=400；串口不上报 maximum_hp，故在此配置） |
| `hp_low` | 150 | 血量低于此值 → 进 RESUPPLY |
| `ammo_low` | 50 | 弹药 ≤ 此值 → 进 RESUPPLY |
| `ammo_ok` | 100 | 弹药 ≥ 此值（且血满）→ 退出 RESUPPLY |
| `game_total_time` | 420 | 比赛总时长（秒） |
| `min_ticks_in_state` | 4 | 状态最小停留 tick |
| `stuck_timeout_s` | 10.0 | 路点卡住超时（秒） |
| `resupply_timeout_s` | 30.0 | 单个补给点超时→轮换（秒） |
| `referee_stale_timeout_s` | 3.0 | 裁判数据过期时间（秒） |
| `opening_strike_duration_s` | 90.0 | 开局打点位停留时长（秒），让自瞄摧毁敌方前哨站 |
| `bump_dash_speed` | 0.8 | 过起伏段冲刺恒速（m/s），见 8.6 |
| `bump_reverse_speed` | 0.4 | 穿越失败反向退回速度（m/s） |
| `bump_tol` | 0.25 | 起伏段出口 x 到达容差（m） |
| `bump_entry_radius` | 0.5 | 距入口多近接管 + y 走廊门限（m） |
| `bump_y_tol` | 0.10 | 起伏段入口/出口 y 偏差上限（m，启动校验） |
| `bump_align_time_s` | 0.5 | cancel Nav2 后静默期（s，舵轮转正+链路排空） |
| `bump_timeout_s` | 20.0 | 穿越超时 → 反向退回（s） |
| `bump_stop_ticks` | 3 | 穿越到达后发几帧零速再交还 |

### YAML 路点/段（非 thresholds）

| 键 | 说明 |
|----|------|
| `bump_segments` | 起伏段列表，每项 `entry`/`exit`(同 y)/`yaw`；空=关闭过起伏功能，见 8.6 |

### ROS2 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `profile_path` | 必填 | YAML 战术文件路径 |
| `tick_frequency` | 10.0 Hz | FSM tick 频率 |
| `goal_topic` | `/goal_pose` | fallback goal 话题 |
| `nav_action_name` | `navigate_to_pose` | Nav2 action 名称 |
| `goal_frame` | `map` | goal 坐标帧 |
| `bump_cmd_vel_topic` | `cmd_vel_chassis` | 过起伏段直发的底盘速度话题 |
| `goal_reached_distance_tolerance` | 0.25 m | 到达判定距离 |

---

## 8. Patrol 战术路线切换规则

PATROL 状态每 tick 按**我方前哨站状态**二选一：

```
① outpost_alive（我方前哨站存活）且 patrol_aggressive 非空 → patrol_aggressive（前压）
② 否则（前哨站被打掉，或未配激进路线）              → patrol（我方半场防守）
```

依据：我方前哨站存活时己方基地无敌，可放心前压；我方前哨站被击毁后基地暴露，退回半场防守。判断依据仅为我方前哨站生死，与敌方前哨站是否被摧毁无关。

路线切换时会重置路点追踪（`path_idx_`/`goal_sent_`/`goal_arrived_`），从新路线起点重新发目标，避免追错航点。换战术只需修改 YAML，无需重新编译。

> **末局决策未实现**：规则上"双方前哨站均被摧毁 + 基地血量胶着"时靠全队总伤害定胜负，此时该攻该守取决于**敌我基地血量对比**。但当前 `GameRobotHP.msg` 只解析了己方字段（`enemy_base_hp`/`enemy_outpost_hp`/`damage_difference` 未接入），读不到敌方数据，故末局专用逻辑暂不实现，等上游串口驱动补全敌方字段后再做。

## 8.5 开局打前哨站（OPENING_STRIKE）

比赛开局，若 profile 配置了 `opening_strike` 打点位，哨兵**第一件事**是导航到该点并停留 `opening_strike_duration_s`（默认 90s），让自瞄摧毁敌方前哨站，然后转入正常巡逻/补给，**本局不再触发**。

- **触发**：比赛进入 RUNNING、配了 `opening_strike`、且 `opening_done_` 为假
- **结束**：停留时长到 → 置 `opening_done_`；或被 RESUPPLY 抢断（血/弹不足）→ 同样消费掉
- **优先级**：低于 RESUPPLY（血弹不足优先回补给），高于 PATROL
- **关闭功能**：profile 里删掉 `opening_strike` 项即可，哨兵直接进 PATROL

> ⚠️ **前提（需与视觉组确认）**：前哨站中央装甲开局旋转（比赛 3 分钟后才停），自瞄能否有效命中旋转装甲，决定本功能实际效果。决策只负责把哨兵开到打点位并停住，命中与否由自瞄决定。
>
> **判断"摧毁"的方式**：当前靠**固定时长**（读不到敌方前哨站血量——`enemy_outpost_hp` 串口未解析）。若将来上游补齐该数据，可改为"血量=0 才结束"更精确。

## 8.6 过起伏路段（BUMP_TRAVERSE）

起伏路段是**波浪形颠簸地形**（连续凸起凹陷交替，非单坡）。Nav2 在上面会因点云抖动、定位颠簸、costmap 误判坡面为障碍而失效。所以这段**不走 Nav2**，改用两段式开环穿越。

### 为什么能开环

- 入口对准由 Nav2 完成（`entry` 点带 yaw），起伏段本身是直线 → 二维降一维，只控 `x` 方向速度。
- 舵轮（swerve）底盘过坎**四轮必须同向前进** → 全程纯 `linear.x`，`y=0`、`yaw=0`、不自旋。这既是物理约束，也让开环成为唯一正确解。

### 五个子阶段（BumpPhase）

| 阶段 | 做什么 |
|------|--------|
| `GOTO_ENTRY` | 发 Nav2 goal 到入口点（对正冲刺 yaw），到达入口附近即进下一步 |
| `ALIGN` | cancel Nav2，静默 `bump_align_time_s`（默认0.5s）：等舵轮转正 + 速度链路排空 |
| `DASHING` | **恒速** `bump_dash_speed` 直冲，纯 `linear.x`；越过出口 x 即硬停 |
| `DONE` | 发几帧零速停稳，交还进入前记录的业务状态（`bump_return_state_`） |
| `FAILED` | 反向低速退回入口，退回成功则标记该段本局禁用 |

### 关键设计（波浪地形特化）

- **恒速不减速**：波浪地形靠冲量连续翻越，末端减速 = 卡在波谷。所以全程 `bump_dash_speed` 恒速，只在越过出口 x 后硬停（与单坡"末端线性减速"方案的本质区别）。
- **到达判据只看 x**：单轴单边比较，`bump_tol` 放宽到 0.25（吸收颠簸定位抖动）；y/yaw 忽略。
- **盲走兜底**：颠簸中 TF/定位短暂丢失是常态，`sentry_pos_valid()` 为假时保持上一帧速度继续冲，**绝不中途停车**（停 = 卡波谷）。
- **抢占屏蔽**：DASHING 中除 IDLE 外不接受任何抢占（见 4.1/4.2）。

### 触发（位置感知，自动）

`find_bump_to_cross()` 每 tick 扫描 `bump_segments`：当哨兵与当前业务目标（巡逻/补给点）**分处某段 x 跨度的两侧**、且哨兵在该段 y 走廊内（`bump_entry_radius`），即接管。冲刺方向由"哨兵在哪侧"自动定：
- **前压**：激进路线的点在高地侧 → 自然正向过（FORWARD）
- **撤退**：残血时补给点在低地侧 → 自然反向过（BACKWARD，速度减半更稳）

前压与过起伏段的绑定通过"激进路线的点配在起伏段对侧"体现，过起伏段能力本身只需纯位置感知。

### 速度链路（为什么不打架）

穿越起伏段直发 `cmd_vel_chassis`（底盘系终点），绕过 `fake_vel_transform`。实测该节点是纯回调驱动——只在收到 Nav2 速度时才转发一帧，自身 timer 只广播 TF、从不周期发零速。所以 cancel Nav2 后底盘话题自然静默，决策节点是穿越期间唯一速度源，无需 mux。

### 参数（写入两份 profile 的 `thresholds` + `bump_segments`）

| 参数 | 默认 | 说明 |
|------|------|------|
| `bump_dash_speed` | 0.8 | m/s 冲刺恒速（实测先低速再加） |
| `bump_reverse_speed` | 0.4 | m/s 失败反向退回速度 |
| `bump_tol` | 0.25 | m 出口 x 到达容差 |
| `bump_entry_radius` | 0.5 | m 距入口多近接管，也是 y 走廊门限 |
| `bump_y_tol` | 0.10 | m 入口/出口 y 偏差上限（启动校验，段必须沿 x 轴直线） |
| `bump_align_time_s` | 0.5 | s cancel 后静默期（舵轮转正 + 链路排空） |
| `bump_timeout_s` | 20.0 | s 穿越超时 → 反向退回 |
| `bump_stop_ticks` | 3 | DONE 时发几帧零速再交还 |

> ⚠️ `bump_segments` 坐标必须实车逐点标定（红蓝镜像分别标）。`bump_segments: []`（空）即关闭功能。启动时校验 `|entry.y-exit.y| ≤ bump_y_tol`，不满足直接抛异常。

---

## 9. 编译与运行

```bash
cd ~/omni_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select omni_decision_sample

# 单元测试
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --packages-select omni_decision_sample
./build/omni_decision_sample/fsm_test              # 状态机 (32)
./build/omni_decision_sample/arrival_tracker_test  # Nav2到达判定 (11)
./build/omni_decision_sample/bump_test             # 过起伏路段 (11)

# 红方(默认)
ros2 launch omni_decision_sample omni_decision_sample_launch.py profile:=rmuc_red.yaml

# 蓝方(裸文件名自动解析到 config/profiles/)
ros2 launch omni_decision_sample omni_decision_sample_launch.py profile:=rmuc_blue.yaml

# 也支持完整/绝对路径
ros2 launch omni_decision_sample omni_decision_sample_launch.py \
  profile:=~/omni_navigation/src/omni_decision_sample/config/profiles/rmuc_red.yaml
```

---

## 10. 当前状态

| 项目 | 状态 |
|------|------|
| 状态机 | 5 态（IDLE / OPENING_STRIKE / BUMP_TRAVERSE / PATROL / RESUPPLY） |
| 单元测试 | 54/54 通过（fsm 32 + arrival_tracker 11 + bump 11） |
| 编译警告 | 0 |
| 死代码 | 0 |
| 已知逻辑缺陷 | 0 |
| 待验证 | Profile 坐标需实车标定；起伏段 `bump_segments` 坐标+`bump_dash_speed` 需平地假坎→真实起伏段逐步验证 |
| 未实现（有意） | 末局攻守决策（依赖敌方基地血量，当前链路读不到） |

---

## 11. 与外部模块的边界

```
┌─────────────────────────────────────────────────┐
│                 omni_decision_sample           │
│                                                 │
│  裁判(血量/弹药) ──→ Context ──→ FSM ──→ Nav2    │
│                                                 │
│  只管一个问题: "下一步往哪走"                      │
└─────────────────────────────────────────────────┘
         │                              │
         │ 不需要                        │ 需要
         ▼                              ▼
┌─────────────────┐          ┌─────────────────┐          ┌─────────────────┐
│  自瞄 (视觉组)   │          │  Nav2 导航      │
│  独立运作        │          │  路径规划+执行   │
│  看到人自动锁    │          │  → /cmd_vel_... │
└─────────────────┘          └────────┬────────┘
         │                            │ Twist
         │ 不需要（电控组）            ▼
         │                   ┌─────────────────┐
         │                   │ rm_serial_driver│ 打包 lx/ly/az
         │                   │ 订阅cmd_vel_chassis│ → control帧(0xA0)
         │                   └────────┬────────┘
         ▼                            ▼ USB CDC
┌─────────────────┐          ┌─────────────────┐
│  姿态/小陀螺     │          │  电控底盘        │
│  电控自行管      │◀─────────│  舵轮运动        │
│  (受击才转)      │  同一板    │  mode电控内部管  │
└─────────────────┘          └─────────────────┘
```

> 过起伏时决策也发 /cmd_vel_chassis(纯 linear.x), 经同一条串口链路到电控。详见 11.1。

### 11.1 电控串口对接（已核实链路）

决策发速度 → 串口驱动打包 → 电控底盘，全链路话题/字段已对齐:

```
omni_decision_sample                    rm_serial_driver              电控(sentry_chassis)
  publish (Twist)                         订阅 /cmd_vel_chassis          USB CDC 收 control 帧(0xA0)
  /cmd_vel_chassis  ──────────────────▶   lx=linear.x                ──▶ AUTO模式: vy=lx(前后)
    linear.x = 前向速度                    ly=linear.y                     vx=ly(左右)
    linear.y = 侧向(过起伏恒为0)           az=angular.z                    → 四舵轮
    angular.z= 0(过起伏)                   mode=current_mode_(见下)
```

**职责边界（与电控约定，已确认）**:

| 事项 | 归属 | 说明 |
|------|------|------|
| 底盘速度 lx/ly/az | **导航侧发** | 决策/Nav2 → /cmd_vel_chassis → 串口驱动 |
| mode (小陀螺/estop) | **电控内部管** | 导航侧不发, 恒为 normal(0) |
| 小陀螺旋转 | **电控** | 受击时电控自己触发(flag_groy), 不经串口 mode |
| 急停 | **电控看门狗** | 决策停发 cmd_vel → 电控超时停车(非主动 estop) |
| 姿态切换(MOVE/ATTACK/DEFENSE) | **电控** | 电控按哨兵血量+运动状态自行切 |

**过起伏(BUMP_TRAVERSE)对接**: 决策直发 /cmd_vel_chassis(纯 linear.x), 与正常导航
(fake_vel_transform 输出同话题)在时序上不冲突——过起伏前已 cancel Nav2, controller
停止输出, fake_vel_transform 纯回调驱动随之静默, 决策成为唯一速度源。

**⚠️ 依赖电控侧(非本模块)**:
1. **电控"受击小陀螺"若实装**, 必须与移动/过起伏互斥——过起伏冲刺中触发旋转会导致舵轮翻车。
2. **cmd_vel_chassis 双发布者**(fake_vel_transform + 决策)逻辑上不打架(过起伏已 cancel Nav2), 需实车/仿真验证。

**信号未接真但暂时无害**:
- `stage_remain_time` 电控写死 400: game_running() 主要靠 game_progress(真实), stage_remain_time 只做范围校验(400 合法), 当前不影响。若将来实现末局决策(依赖剩余时间), 需电控改回真值。
- `ros_state` 串口驱动写死 2(running): 导航→电控状态心跳恒 running, 电控未用它做故障保护, 当前无影响。
