#!/usr/bin/env python3
"""
可视化 mock 裁判 —— 带按钮的图形界面，实车测试决策导航时用。

启动后弹出窗口，点按钮/拖滑块即可实时改哨兵状态。它以 10Hz 持续发布裁判
数据（保持 referee 新鲜），比命令行更直观。

用法：
  ros2 run omni_decision_sample mock_referee_gui.py

需要图形环境 (DISPLAY)。若在无头机器上，改用 mock_referee_tui.py (键盘版)。
"""

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rm_interfaces.msg import GameStatus, RobotStatus, RfidStatus, GameRobotHP


class MockRefereeGui(Node):
    def __init__(self):
        super().__init__("mock_referee_gui")

        # 初始状态：满血满弹、比赛进行中、前哨站存活
        self.hp = 400
        self.max_hp = 400
        self.ammo = 300
        self.remain = 300
        self.running = True
        self.outpost_alive = True

        self.pub_game = self.create_publisher(GameStatus, "/referee/game_status", 10)
        self.pub_robot = self.create_publisher(RobotStatus, "/referee/robot_status", 10)
        self.pub_rfid = self.create_publisher(RfidStatus, "/referee/rfidStatus", 10)
        self.pub_hp = self.create_publisher(GameRobotHP, "/referee/all_robot_hp", 10)

        self.timer = self.create_timer(0.1, self._publish)  # 10Hz

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

        self.pub_rfid.publish(RfidStatus())  # 到达补给区判定交给实车 RFID

        ghp = GameRobotHP()
        ghp.ally_outpost_hp = 500 if self.outpost_alive else 0
        ghp.ally_base_hp = 5000
        self.pub_hp.publish(ghp)


# ============================================================================
#  GUI
# ============================================================================

class App:
    def __init__(self, node: MockRefereeGui):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Mock 裁判 — 决策测试控制台")
        self.root.geometry("420x560")

        pad = {"padx": 8, "pady": 4}

        # --- 状态显示区 ---
        status = ttk.LabelFrame(self.root, text="当前状态")
        status.pack(fill="x", **pad)
        self.status_var = tk.StringVar()
        ttk.Label(status, textvariable=self.status_var,
                  font=("monospace", 11)).pack(anchor="w", padx=8, pady=6)

        # --- 场景快捷按钮 ---
        scenes = ttk.LabelFrame(self.root, text="一键场景")
        scenes.pack(fill="x", **pad)
        buttons = [
            ("① 满血满弹 → PATROL", self.s_full),
            ("② 残血 (hp=100) → RESUPPLY", self.s_low_hp),
            ("③ 弹药低 (ammo=40) → RESUPPLY", self.s_low_ammo),
            ("④ 阵亡 (hp=0) → 保持导航不卡死", self.s_dead),
            ("⑤ 复活 (hp=40) → 继续回补给", self.s_revive),
            ("⑥ 恢复满 → PATROL", self.s_full),
        ]
        for text, cmd in buttons:
            ttk.Button(scenes, text=text, command=cmd).pack(
                fill="x", padx=8, pady=2)

        # --- 开关区 ---
        toggles = ttk.LabelFrame(self.root, text="开关")
        toggles.pack(fill="x", **pad)
        ttk.Button(toggles, text="切换前哨站 存活/被击毁 (影响巡逻路线)",
                   command=self.t_outpost).pack(fill="x", padx=8, pady=2)
        ttk.Button(toggles, text="切换比赛 进行中/未开始 (未开始→IDLE)",
                   command=self.t_game).pack(fill="x", padx=8, pady=2)

        # --- 微调滑块 ---
        sliders = ttk.LabelFrame(self.root, text="微调 (拖动实时生效)")
        sliders.pack(fill="both", expand=True, **pad)

        ttk.Label(sliders, text="血量 hp").pack(anchor="w", padx=8)
        self.hp_scale = tk.Scale(sliders, from_=0, to=400, orient="horizontal",
                                 command=self.on_hp)
        self.hp_scale.set(self.node.hp)
        self.hp_scale.pack(fill="x", padx=8)

        ttk.Label(sliders, text="弹药 ammo").pack(anchor="w", padx=8)
        self.ammo_scale = tk.Scale(sliders, from_=0, to=400, orient="horizontal",
                                   command=self.on_ammo)
        self.ammo_scale.set(self.node.ammo)
        self.ammo_scale.pack(fill="x", padx=8)

        self._refresh_status()

    # --- status ---
    def _refresh_status(self):
        n = self.node
        prog = "RUNNING" if n.running else "NOT_START"
        outpost = "存活(前压)" if n.outpost_alive else "被击毁(防守)"
        self.status_var.set(
            f"hp   = {max(0, n.hp)} / {n.max_hp}\n"
            f"ammo = {max(0, n.ammo)}\n"
            f"game = {prog}\n"
            f"前哨站 = {outpost}")

    def _sync_sliders(self):
        # 场景按钮改了值 → 同步滑块位置（避免滑块回调递归，用 set 不触发 command）
        self.hp_scale.set(max(0, self.node.hp))
        self.ammo_scale.set(max(0, self.node.ammo))

    # --- scene buttons ---
    def s_full(self):
        self.node.hp, self.node.ammo = 400, 300
        self._sync_sliders(); self._refresh_status()

    def s_low_hp(self):
        self.node.hp, self.node.ammo = 100, 300
        self._sync_sliders(); self._refresh_status()

    def s_low_ammo(self):
        self.node.hp, self.node.ammo = 400, 40
        self._sync_sliders(); self._refresh_status()

    def s_dead(self):
        self.node.hp = 0
        self._sync_sliders(); self._refresh_status()

    def s_revive(self):
        self.node.hp = 40
        self._sync_sliders(); self._refresh_status()

    # --- toggles ---
    def t_outpost(self):
        self.node.outpost_alive = not self.node.outpost_alive
        self._refresh_status()

    def t_game(self):
        self.node.running = not self.node.running
        self._refresh_status()

    # --- sliders ---
    def on_hp(self, val):
        self.node.hp = int(float(val))
        self._refresh_status()

    def on_ammo(self, val):
        self.node.ammo = int(float(val))
        self._refresh_status()

    def run(self):
        self.root.mainloop()


# ============================================================================
#  main
# ============================================================================

def main():
    rclpy.init()
    node = MockRefereeGui()

    # ROS spin 放后台线程，主线程跑 tkinter
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = App(node)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
