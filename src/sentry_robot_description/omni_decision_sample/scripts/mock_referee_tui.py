#!/usr/bin/env python3
"""
交互式 mock 裁判 —— 键盘实时改哨兵状态，实车测试决策导航时用。

它以 10Hz 持续发布裁判数据（保持 referee 新鲜），你按键即可切换血量/弹药/
前哨站等场景，比手打 `ros2 topic pub` 顺手得多。

用法：
  ros2 run omni_decision_sample mock_referee_tui.py

按键：
  1  满血满弹   (hp=400 ammo=300)  → 决策应 PATROL
  2  残血       (hp=100 ammo=300)  → 决策应 RESUPPLY (hp low)
  3  弹药低     (hp=400 ammo=40)   → 决策应 RESUPPLY (ammo low)
  4  阵亡       (hp=0)             → 保持 RESUPPLY, 持续发目标
  5  复活       (hp=40)            → 仍 RESUPPLY, 继续回补给区
  6  恢复满     (hp=400 ammo=300)  → RESUPPLY → PATROL

  o  切换前哨站存活/被击毁 (影响巡逻路线: 前压/半场防守)
  +  血量 +50      -  血量 -50
  ]  弹药 +50      [  弹药 -50
  g  切换比赛 进行中/未开始 (未开始→决策进 IDLE)

  p  打印当前状态
  q  退出
"""

import sys
import termios
import tty
import select
import threading

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import GameStatus, RobotStatus, RfidStatus, GameRobotHP

HELP = __doc__


class MockRefereeTui(Node):
    def __init__(self):
        super().__init__("mock_referee_tui")

        # 初始状态：满血满弹、比赛进行中、前哨站存活
        self.hp = 400
        self.max_hp = 400
        self.ammo = 300
        self.remain = 300
        self.running = True          # 比赛是否进行中
        self.outpost_alive = True

        self.pub_game = self.create_publisher(GameStatus, "/referee/game_status", 10)
        self.pub_robot = self.create_publisher(RobotStatus, "/referee/robot_status", 10)
        self.pub_rfid = self.create_publisher(RfidStatus, "/referee/rfidStatus", 10)
        self.pub_hp = self.create_publisher(GameRobotHP, "/referee/all_robot_hp", 10)

        # 10Hz 持续发布，保证 referee 数据新鲜
        self.timer = self.create_timer(0.1, self._publish)

    # ------------------------------------------------------------------
    def _publish(self):
        gs = GameStatus()
        gs.game_progress = GameStatus.RUNNING if self.running else GameStatus.NOT_START
        gs.stage_remain_time = self.remain
        self.pub_game.publish(gs)

        rs = RobotStatus()
        rs.current_hp = max(0, self.hp)
        rs.maximum_hp = self.max_hp
        rs.projectile_allowance_17mm = max(0, self.ammo)
        self.pub_robot.publish(rs)

        rfid = RfidStatus()
        self.pub_rfid.publish(rfid)   # 到达补给区判定交给实车 RFID，这里全 False

        ghp = GameRobotHP()
        ghp.ally_outpost_hp = 500 if self.outpost_alive else 0
        ghp.ally_base_hp = 5000
        self.pub_hp.publish(ghp)

    # ------------------------------------------------------------------
    def status_line(self):
        prog = "RUNNING" if self.running else "NOT_START"
        outpost = "存活" if self.outpost_alive else "被击毁"
        return (f"hp={max(0, self.hp)}/{self.max_hp}  ammo={max(0, self.ammo)}  "
                f"game={prog}  前哨站={outpost}")

    def handle_key(self, k):
        """返回 False 表示要退出。"""
        if k == "1":
            self.hp, self.ammo = 400, 300
        elif k == "2":
            self.hp, self.ammo = 100, 300
        elif k == "3":
            self.hp, self.ammo = 400, 40
        elif k == "4":
            self.hp = 0
        elif k == "5":
            self.hp = 40
        elif k == "6":
            self.hp, self.ammo = 400, 300
        elif k == "o":
            self.outpost_alive = not self.outpost_alive
        elif k == "+" or k == "=":
            self.hp = min(self.max_hp, self.hp + 50)
        elif k == "-" or k == "_":
            self.hp = self.hp - 50
        elif k == "]":
            self.ammo = self.ammo + 50
        elif k == "[":
            self.ammo = self.ammo - 50
        elif k == "g":
            self.running = not self.running
        elif k == "p":
            pass  # 下面统一打印
        elif k == "q":
            return False
        else:
            return True  # 未知键，忽略，不刷状态行
        print(f"  → {self.status_line()}")
        return True


def _read_key(timeout=0.1):
    """非阻塞读单个按键，无输入返回 None。"""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None


def main():
    rclpy.init()
    node = MockRefereeTui()

    print(HELP)
    print(f"  初始: {node.status_line()}")
    print("  (按键实时生效, q 退出)\n")

    # ROS spin 放后台线程，主线程读键盘
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            k = _read_key(0.1)
            if k is None:
                continue
            if not node.handle_key(k):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
        print("\n  mock 已停止。")


if __name__ == "__main__":
    main()
