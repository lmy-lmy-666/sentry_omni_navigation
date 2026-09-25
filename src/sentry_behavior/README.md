# sentry_behavior

哨兵机器人战术决策包。**自研分层状态机（HSM）**,脱离 BehaviorTree 库,零外部状态机依赖。

`sentry_behavior_node` 单节点订阅裁判系统话题写入快照,以固定频率（`tick_frequency`,默认 20Hz）驱动 2 层状态机,把目标点发布到 `/goal_pose`。节点内嵌一个 TCP NDJSON 状态可视化协议,供独立的 Rust/C 客户端连接刷新。

## 架构

```
裁判系统 (referee/*) ──► sentry_behavior_node
                            │  订阅回调写入 RefereeSnapshot:
                            │    referee/game_status   (GameStatus)  -> progress / remain
                            │    referee/robot_status  (RobotStatus) -> hp / ammo
                            │    referee/all_robot_hp   (GameRobotHP) -> outpost_hp
                            │
                            ▼
                  SingleThreadedExecutor + 20Hz Timer
                            │
                            ▼
                  BehaviorMachine (2 层 HSM)
                    ├─ 生命周期层: WAIT_START <-> IN_MATCH (reactive 父 guard)
                    └─ 战术层: 按 strategy 选的 guard 表 (首个命中者胜)
                            │
                            ▼
                  GoalPublisher -> /goal_pose (PoseStamped)
                  (去重 + 0 订阅重发 + 进入 WAIT_START 复位)
```

### 2 层分层状态机

- **生命周期层（唯一真有状态）**:`WAIT_START ↔ IN_MATCH`。每 tick 先判 reactive 父 guard(等价旧树 `ReactiveFallback[Inverter(IsGameStatus), 主决策]`):比赛中 `game_progress` 离开 4 立即转 `WAIT_START`、复位战术层并清 `GoalPublisher` 去重缓存,主决策被抢占。
- **战术层（无状态）**:每个策略 = 一张按优先级排序的 `guard 表`,逐项求值,首个 `active` 命中者保持其目标;末项为无条件兜底。

> 历史 `RMUC.xml` / `b.xml` 中"补满返回守点"的高阈值（400/50、381/50）滞回分支**逻辑上不可达**(进入该分支的前提与其判据矛盾),实际行为是**无状态纯函数**。本实现按真实生效语义用**无状态互补 guard**复刻,丢弃死阈值。

### GoalPublisher 语义

单一 `/goal_pose` publisher,构造期一次性创建(消除旧多 publisher 的 DDS 首包丢失):

- 去重:与上次"已送达"坐标精确 `(x,y)` 相等则不重发;
- 即使 0 订阅者也 publish,但**仅当有订阅者时才记为已送达**,否则下个 tick 继续重发(处理本节点早于 Nav2 `bt_navigator` 就绪的启动时序);
- 进入 `WAIT_START` 时 `reset()` 清缓存,使下一场重回同坐标也能重新发布。

## 战术策略

运行时由 `strategy` 参数选择(替代旧 `target_tree`):

| strategy | 用途 | guard 表(优先级序) |
|---|---|---|
| `rmuc_defend` | RMUC 守家 | ① 守点 `(3.71,-0.61)`: `robot_valid && hp>=151 && ammo>=1` ② 否则 补给 `(-0.27,-3.94)` |
| `a` | 进攻向 | ① 攻击 `(3.71,-0.61)`: `hp>=300 && ammo>=1` ② 否则 补给 `(-0.27,-3.94)` |
| `b` | 防守向(含前哨判断) | 前哨存活(`outpost_hp>=1`): ① 巡逻 `(9.17,4.07)`: `hp>=300 && ammo>=1` ② 否则 补给 `(-0.27,-3.94)`;前哨倒: 点3 `(3.27,-0.90)` |

阈值是节点参数(`rmuc_hp_min` / `attack_hp_min` / `patrol_hp_min` / `ammo_min` / `outpost_hp_min`),不重编即可调。

## ROS 接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `referee/game_status` | `rm_interfaces/msg/GameStatus` | 订阅;`game_progress`(4=比赛中,5=结算)、`stage_remain_time` |
| `referee/robot_status` | `rm_interfaces/msg/RobotStatus` | 订阅;`current_hp`、`projectile_allowance_17mm`(弹量) |
| `referee/all_robot_hp` | `rm_interfaces/msg/GameRobotHP` | 订阅;`ally_outpost_hp`(前哨血量,b 用) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 发布;`frame_id=map`,Nav2 `bt_navigator` 消费 |

订阅 QoS:`reliable + volatile, depth 10`。

## 参数(`params/sentry_behavior.yaml`)

```yaml
sentry_behavior_node:
  ros__parameters:
    use_sim_time: false
    strategy: rmuc_defend     # rmuc_defend / a / b
    tick_frequency: 20.0      # Hz
    viz_enable: true          # 内嵌 TCP NDJSON 可视化协议
    viz_port: 1667            # 可视化端口
    rmuc_hp_min: 151
    attack_hp_min: 300
    patrol_hp_min: 300
    ammo_min: 1
    outpost_hp_min: 1
```

未知 `strategy` 名启动即 `FATAL` 退出(不静默回退)。

## 启动

```bash
# 默认 strategy=rmuc_defend
ros2 launch sentry_behavior sentry_behavior_launch.py

# 指定策略
ros2 launch sentry_behavior sentry_behavior_launch.py strategy:=a
ros2 launch sentry_behavior sentry_behavior_launch.py strategy:=b

# 直接跑节点(无 launch)
ros2 run sentry_behavior sentry_behavior_node --ros-args -p strategy:=rmuc_defend
```

实战中由 `sentry_nav_bringup/launch/rm_sentry_launch.py` 统一拉起(`enable_behavior:=True` 时),`strategy` 经其透传。

## 状态可视化协议（TCP NDJSON）

节点内嵌一个 TCP server(`viz_port`,默认 1667),发逐行 JSON(NDJSON)。独立线程 + 非阻塞写 + 慢客户端丢帧,**永不阻塞 20Hz 决策 tick**;`MSG_NOSIGNAL` 防客户端断开 `SIGPIPE` 杀进程。客户端连接即收 `graph` 帧(静态拓扑),之后持续收 `state`(活动路径/目标/裁判)+ `transition` 事件。供独立 Rust/C GUI 消费(无需 ROS 依赖)。

```jsonc
// 连接时一次:静态拓扑
{"type":"graph","v":1,"strategy":"rmuc_defend","states":[{"id":"WAIT_START","parent":null},{"id":"IN_MATCH","parent":null},{"id":"DEFEND","parent":"IN_MATCH"},{"id":"SUPPLY","parent":"IN_MATCH"}],"transitions":[...]}
// 每次转移:事件
{"type":"transition","t":<ms>,"from":"DEFEND","to":"SUPPLY"}
// 状态变更 + ~10Hz:活动路径 + 目标 + 裁判
{"type":"state","t":<ms>,"active":["IN_MATCH","SUPPLY"],"goal":{"x":-0.27,"y":-3.94},"referee":{"progress":4,"remain":200,"hp":100,"ammo":100,"outpost_hp":0}}
```

```bash
# 调试连法:
nc 127.0.0.1 1667
```

## 回归测试

`tests/inv_smoke.sh` 启动 `sentry_behavior_node`(`strategy=rmuc_defend`),注入裁判数据,从 `/goal_pose` topic 抓坐标序列,验证 7 条行为不变量(需先 `source install/setup.bash`):

| INV | 描述 |
|---|---|
| INV-1 | `game_progress != 4`(未开赛)→ 不发 `/goal_pose` |
| INV-2 | `progress=4` + `hp>=151` + `ammo>=1` → 守点 `(3.71,-0.61)` |
| INV-3 | `progress=4` + `ammo=0` → 补给 `(-0.27,-3.94)` |
| INV-4 | 守 → 弹尽回补 → 恢复返回守(`守→补→守` 流转) |
| INV-5 | 守 → `hp<=150` → 回补给 |
| INV-6 | 比赛结束(`progress` 4→5)→ 转 WAIT_START 静默(守点已发、补给不再出现) |
| INV-7 | 节点重启后仍发守点 |

```bash
source install/setup.bash
tests/inv_smoke.sh                          # 跑全部 7 条, exit = FAIL 数
tests/inv_smoke.sh --baseline tests/baseline   # 录基线
tests/inv_smoke.sh --regress  tests/baseline   # 回归对比
```

> **注意（强制同步）**:`inv_smoke.sh` 必须与策略实现同步三处:策略名(`strategy:=rmuc_defend`)、机制(发 `/goal_pose`)、坐标(守 `(3.71,-0.61)` / 补 `(-0.27,-3.94)`)。改 `strategies.cpp` 的坐标/策略名/启动机制必须同步改本脚本断言。

## 目录结构

```
sentry_behavior/
├── include/sentry_behavior/
│   ├── referee_snapshot.hpp     裁判数据快照
│   ├── goal_publisher.hpp       /goal_pose 发布(去重/重发/复位)
│   ├── behavior_machine.hpp     2 层 HSM (Ctx/TacticalState/Strategy/Phase/BehaviorMachine)
│   ├── strategies.hpp           策略注册表
│   └── viz/state_viz_server.hpp TCP NDJSON 可视化协议
├── src/                         对应实现 + sentry_behavior_node.cpp(main)
├── launch/                      sentry_behavior_launch.py
├── params/                      sentry_behavior.yaml
└── CMakeLists.txt
```

## 添加新策略

1. 在 `src/strategies.cpp` 加一个 `make_<name>(const StrategyParams &)`,用 `TacticalState{id, guard, GoalCmd}` 按优先级填 `Strategy::states`(末项无条件兜底)。
2. 在 `make_strategy()` 注册表加 `if (name == "<name>") return make_<name>(p);`。
3. (可选)需要的阈值加进 `StrategyParams` + 节点参数声明。
4. `strategy:=<name>` 运行。需要历史/记忆的复杂决策可让某个状态内部再挂一个子状态机(框架支持嵌套)。
