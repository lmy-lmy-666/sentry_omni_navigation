#!/usr/bin/env python3
"""
开场自检 (preflight) —— 上场前 30 秒确认决策所需的输入/输出链路是否就绪。

它订阅决策节点依赖的全部话题，检查是否在真实地收到数据，并检查 Nav2
action 是否可用。任何一项红灯，都意味着决策上场后会瞎跑或卡死。

用法：
  ros2 run omni_decision_sample preflight_check.py

  # odom 话题默认 odometry(由 odom_bridge 发布)。链路: point_lio(aft_mapped_to_init)
  # → odom_bridge → odometry。一般不用改；只在你的定位栈用了别的名时才覆盖:
  ros2 run omni_decision_sample preflight_check.py --ros-args -p odom_topic:=odometry

  # 自定义检查时长 (默认 5 秒)
  ros2 run omni_decision_sample preflight_check.py --ros-args -p duration:=8.0
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rm_interfaces.msg import GameStatus, RobotStatus, RfidStatus, GameRobotHP

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"


class Preflight(Node):
    def __init__(self):
        super().__init__("decision_preflight")

        self.declare_parameter("odom_topic", "odometry")
        self.declare_parameter("nav_action", "navigate_to_pose")
        self.declare_parameter("duration", 5.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.nav_action = self.get_parameter("nav_action").value
        self.duration = float(self.get_parameter("duration").value)

        # per-topic hit counter + last message snapshot
        self.counts = {}
        self.last = {}

        self._sub(GameStatus, "referee/game_status", "game_status")
        self._sub(RobotStatus, "referee/robot_status", "robot_status")
        self._sub(RfidStatus, "referee/rfidStatus", "rfid")
        self._sub(GameRobotHP, "referee/all_robot_hp", "all_hp")
        self._sub(Odometry, self.odom_topic, "odom")

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)

    def _sub(self, msg_type, topic, key):
        self.counts[key] = 0
        self.last[key] = None

        def cb(msg, k=key):
            self.counts[k] += 1
            self.last[k] = msg

        self.create_subscription(msg_type, topic, cb, 10)

    # ------------------------------------------------------------------
    def report(self):
        """打印每项检查结果，返回 True 表示全部通过。"""
        ok = True

        def line(passed, label, detail=""):
            nonlocal ok
            if not passed:
                ok = False
            tag = f"{GREEN}[ OK ]{RESET}" if passed else f"{RED}[FAIL]{RESET}"
            print(f"  {tag} {label:<28} {detail}")

        print("\n================ 决策开场自检 ================")
        print(f"  检查时长: {self.duration:.0f}s\n")

        # --- 裁判系统输入 ---
        print("  [裁判系统输入]")
        gs = self.counts["game_status"]
        line(gs > 0, "referee/game_status",
             f"{gs} 帧" if gs else "无数据！串口驱动是否在跑?")

        rs = self.counts["robot_status"]
        line(rs > 0, "referee/robot_status",
             f"{rs} 帧" if rs else "无数据！哨兵血量/弹药读不到")

        rf = self.counts["rfid"]
        line(rf > 0, "referee/rfidStatus",
             f"{rf} 帧" if rf else "无数据！补给区到达无法确认")

        hp = self.counts["all_hp"]
        line(hp > 0, "referee/all_robot_hp",
             f"{hp} 帧" if hp else "无数据！前哨站状态读不到")

        # --- 数据内容合理性 ---
        print("\n  [数据内容检查]")
        if self.last["robot_status"] is not None:
            m = self.last["robot_status"]
            detail = f"hp={m.current_hp}/{m.maximum_hp} ammo={m.projectile_allowance_17mm}"
            line(m.maximum_hp > 0, "血量上限非零", detail)
            if m.maximum_hp != 400:
                print(f"       {YELLOW}注意: max_hp={m.maximum_hp}, 全自动哨兵应为 400{RESET}")
        else:
            line(False, "血量上限非零", "无 robot_status，跳过")

        if self.last["game_status"] is not None:
            g = self.last["game_status"]
            prog_name = {0: "未开始", 1: "准备", 2: "自检", 3: "倒计时",
                         4: "比赛中", 5: "结算"}.get(g.game_progress, "?")
            line(True, "比赛阶段可读",
                 f"progress={g.game_progress}({prog_name}) remain={g.stage_remain_time}s")
        else:
            line(False, "比赛阶段可读", "无 game_status，跳过")

        # --- 定位输入 ---
        print("\n  [定位输入]")
        od = self.counts["odom"]
        if od > 0:
            p = self.last["odom"].pose.pose.position
            line(True, f"odom ({self.odom_topic})",
                 f"{od} 帧, 当前位置 x={p.x:.2f} y={p.y:.2f}")
        else:
            line(False, f"odom ({self.odom_topic})",
                 f"无数据！话题名可能不对，用 ros2 topic list 核对")

        # --- 导航输出 ---
        print("\n  [导航输出]")
        nav_ready = self.nav_client.wait_for_server(timeout_sec=2.0)
        line(nav_ready, f"Nav2 action ({self.nav_action})",
             "就绪" if nav_ready else "未就绪！哨兵将降级到 topic 模式")

        # --- 结论 ---
        print("\n  ------------------------------------------")
        if ok:
            print(f"  {GREEN}★ 全部通过，链路就绪，可以上场。{RESET}")
        else:
            print(f"  {RED}✗ 有检查未通过，上场前必须修复上面的 FAIL 项。{RESET}")
        print("  ============================================\n")
        return ok


def main():
    rclpy.init()
    node = Preflight()
    node.get_logger().info(
        f"采集数据中 ({node.duration:.0f}s)... 请确保串口驱动/定位栈已启动")

    # spin for `duration` seconds to accumulate messages
    end = node.get_clock().now().nanoseconds + int(node.duration * 1e9)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    passed = node.report()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
