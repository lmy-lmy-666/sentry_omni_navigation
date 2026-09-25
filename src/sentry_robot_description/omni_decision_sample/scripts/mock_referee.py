#!/usr/bin/env python3
"""
Mock referee publisher — 模拟裁判系统数据，方便实车测试"残血回家"等行为。

用法：
  ros2 run omni_decision_sample mock_referee.py --ros-args \
    -p hp:=400 \          # 初始血量
    -p ammo:=300 \        # 初始弹药 (>50 才是 PATROL, ≤50 会进 RESUPPLY)
    -p max_hp:=400 \      # 最大血量
    -p remain:=300        # 剩余时间

运行后可以用 ros2 topic pub 动态改血量：
  ros2 topic pub /referee/robot_status rm_interfaces/msg/RobotStatus \
    "{current_hp: 40, maximum_hp: 400, projectile_allowance_17mm: 50}" --once
"""

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import GameStatus, RobotStatus, RfidStatus, GameRobotHP


class MockReferee(Node):
    def __init__(self):
        super().__init__("mock_referee")

        self.declare_parameter("hp", 400)
        self.declare_parameter("ammo", 300)   # >50 so default state is PATROL, not RESUPPLY
        self.declare_parameter("max_hp", 400)
        self.declare_parameter("remain", 300)

        self.hp = self.get_parameter("hp").value
        self.ammo = self.get_parameter("ammo").value
        self.max_hp = self.get_parameter("max_hp").value
        self.remain = self.get_parameter("remain").value

        # publishers — topic names must match decision node subscriptions
        self.pub_game = self.create_publisher(GameStatus, "/referee/game_status", 10)
        self.pub_robot = self.create_publisher(RobotStatus, "/referee/robot_status", 10)
        self.pub_rfid = self.create_publisher(RfidStatus, "/referee/rfidStatus", 10)
        self.pub_hp = self.create_publisher(GameRobotHP, "/referee/all_robot_hp", 10)

        # allow external topic pub to override hp/ammo
        self.sub_override = self.create_subscription(
            RobotStatus, "/referee/robot_status",
            self._on_override, 10)

        self.timer = self.create_timer(1.0, self._tick)

    def _on_override(self, msg):
        self.hp = msg.current_hp
        self.ammo = msg.projectile_allowance_17mm
        self.max_hp = msg.maximum_hp
        self.get_logger().info(
            f"override → hp={self.hp}/{self.max_hp} ammo={self.ammo}")

    def _tick(self):
        # game status — always running
        gs = GameStatus()
        gs.game_progress = GameStatus.RUNNING
        gs.stage_remain_time = self.remain
        self.pub_game.publish(gs)

        # robot status
        rs = RobotStatus()
        rs.current_hp = self.hp
        rs.maximum_hp = self.max_hp
        rs.projectile_allowance_17mm = self.ammo
        self.pub_robot.publish(rs)

        # rfid — use RobotStatus subscription as a side channel:
        # publish non-zero rfid_status to trigger on_supply_pad()
        rfid = RfidStatus()
        rfid.friendly_supply_zone_non_exchange = False
        rfid.friendly_supply_zone_exchange = False
        self.pub_rfid.publish(rfid)

        # global hp
        ghp = GameRobotHP()
        ghp.ally_outpost_hp = 500
        ghp.ally_base_hp = 5000
        self.pub_hp.publish(ghp)


def main():
    rclpy.init()
    node = MockReferee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
