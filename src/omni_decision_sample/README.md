# omni_decision_sample

> 哨兵自主导航调度器。只管一个问题：**下一步往哪走。**

| 仓库 | 分支 | 本地路径 |
|------|------|----------|
| [lmy-lmy-666/sentry_decision](https://github.com/lmy-lmy-666/sentry_decision) | `omni_decision_sample` | `/home/lmy/omni_navigation/src/omni_decision_sample` |

## 五个状态

```
比赛开局(可选) → OPENING_STRIKE （先去打点位, 让自瞄摧毁敌方前哨站, 停留一段后转正常, 每局一次）
遇到起伏路段    → BUMP_TRAVERSE  （舵轮纯直线开环冲过波浪地形, 绕过 Nav2, 穿越中不可打断）
有血有弹      → PATROL   （沿巡逻路线循环走点）
hp<150 或 弹药≤50 → RESUPPLY （回唯一补给区，回满 400 且弹药≥100 再出来，永不放弃）
裁判断连      → IDLE   （原地不动）
```

优先级：IDLE > BUMP_TRAVERSE(穿越中) > RESUPPLY > OPENING_STRIKE > PATROL。
- 开局打点会被"血弹不足回补给"打断，且一旦打断/超时就本局不再触发。
- **冲起伏路段一旦开始就不可打断**（受击/补给都不停，中途停=卡在起伏段上），唯一例外是裁判断连/比赛结束→立即零速急停。

弹药靠规则里"占领补给区每分钟自动 +100"被动恢复，决策不主动兑换。
补给区是唯一的恢复点，也是阵亡复活后的回归点——RESUPPLY 只要没恢复满就一直尝试导航回去，绝不中途放弃。

## 过起伏路段（波浪地形）

起伏路段是波浪形颠簸地形，Nav2 在上面会因点云抖动/costmap 误判坡面为障碍而失效。所以这段**不走 Nav2**：当哨兵与当前目标分处某起伏段两侧、且靠近入口时，自动接管——Nav2 先把车开到入口对正朝向 → cancel Nav2 → **舵轮四轮同向、纯直线恒速开环冲过** → 到出口 x 坐标硬停 → 交还原状态。

- **恒速不减速**：波浪地形靠冲量翻越，末端减速会卡在波谷，所以全程恒速直到越过出口才停。
- **舵轮约束**：全程只发 `linear.x`，y/yaw=0、不自旋（四轮必须同向才能过坎）。
- **盲走兜底**：颠簸中定位/TF 短暂丢失是常态，此时保持上一帧速度继续冲，绝不中途停车。
- **失败恢复**：穿越超时 → 反向低速退回入口，并标记该段本局禁用。
- 触发是**纯位置感知**：前压时激进路线的点在高地侧→自然正向过；残血撤退时补给点在低地侧→自然反向过。方向自动判定。

## 快速开始

```bash
# 编译
cd ~/omni_navigation
source install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select omni_decision_sample

# 运行（红方）
ros2 launch omni_decision_sample omni_decision_sample_launch.py profile:=rmuc_red.yaml

# 运行（蓝方）
ros2 launch omni_decision_sample omni_decision_sample_launch.py profile:=rmuc_blue.yaml

# 也可以用完整路径
ros2 launch omni_decision_sample omni_decision_sample_launch.py \
  profile:=~/omni_navigation/src/omni_decision_sample/config/profiles/rmuc_red.yaml

# 运行测试
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --packages-select omni_decision_sample
./build/omni_decision_sample/fsm_test              # 状态机 (32)
./build/omni_decision_sample/arrival_tracker_test  # Nav2到达判定 (11)
./build/omni_decision_sample/bump_test             # 过起伏路段 (11)
```

## 换战术

改 YAML 就行，不用重新编译。关键参数：

```yaml
thresholds:
  hp_low: 150       # 血量低于这个 → 去补给
  ammo_low: 50      # 弹药 ≤ 这个 → 去补给
  ammo_ok: 100      # 弹药 ≥ 这个（且血满）→ 结束补给出去巡逻
  opening_strike_duration_s: 90.0   # 开局打点位停留时长(秒)

opening_strike: {x: 2.0, y: 0.0}   # 开局打前哨站位(自瞄能打到敌方前哨站); 删掉此项=关闭该功能

patrol:               # 默认巡逻路线（前哨站被打掉时用 → 我方半场防守）
  - {x: 3.88, y: 2.67, dwell_s: 10.0}

patrol_aggressive:    # 我方前哨站存活时用 → 前压
  - {x: 8.0, y: 0.0, dwell_s: 10.0}

supply: {x: -1.02, y: -4.91}   # 唯一补给点（也是恢复/复活回归点）

# 过起伏路段（舵轮纯直线开环冲）
thresholds:
  bump_dash_speed: 0.8   # m/s 冲刺恒速; 实测先0.8再往上加(要够翻波浪的动能)
  bump_tol: 0.25         # m   出口x到达容差(比Nav2放宽, 吸收颠簸抖动)
  bump_timeout_s: 20.0   # s   穿越超时→反向退回

bump_segments:           # 起伏段列表; 空=不启用。entry/exit 必须同 y(沿x轴直线)
  - entry: {x: 0.7,   y: -7.086}   # 一侧平地(Nav2先开到这, 对正yaw)
    exit:  {x: 5.458, y: -7.086}   # 另一侧平地(冲刺目标, 只看x判到达)
    yaw: 0.0                        # 冲刺时车头朝向(rad)
```

巡逻路线按**我方前哨站状态**二选一：我方前哨站存活走 `patrol_aggressive`（前压），被打掉走 `patrol`（我方半场防守）。判断只看我方前哨站，与敌方前哨站是否被摧毁无关。

> ⚠️ 起伏段坐标必须实车逐点标定，红蓝分别标。`bump_segments: []`（空）即关闭该功能。

## 不做什么

- 不追敌人（自瞄独立运作）
- 不切换姿态（电控负责）
- 不收雷达/自瞄数据
- 不控小陀螺/急停（`mode` 归电控内部管，导航侧只发速度 lx/ly/az）

## 和电控怎么接

决策发速度 → 串口驱动 → 电控，链路已对齐（详见 [ANALYSIS.md](./ANALYSIS.md) 11.1）：

```
决策/Nav2 → /cmd_vel_chassis (Twist) → rm_serial_driver 打包 lx/ly/az → 电控舵轮
```

- 导航侧**只发速度**（含过起伏的 linear.x）；小陀螺、急停、姿态切换全归电控
- ⚠️ 电控侧：若"受击小陀螺"实装，必须与移动/过起伏互斥(冲坡时旋转=翻车)

## 详细文档

- [ANALYSIS.md](./ANALYSIS.md) —— 完整设计文档
- [docs/状态流转图.md](./docs/状态流转图.md) —— 五态流转图 + 穿越子状态机 + 实战时序
- [docs/坐标标定说明.md](./docs/坐标标定说明.md) —— 上场前坐标标定
- [docs/实车测试指南.md](./docs/实车测试指南.md) —— mock 裁判实车验证
