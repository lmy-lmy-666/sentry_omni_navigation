#!/usr/bin/env python3
"""
Capture all topics published by rm_serial_driver and dump each message as JSON.

Default output: NDJSON (one JSON object per line) — easy to tail / pipe to jq.
Each line:
  {"t_wall": 1779579884.123,  # capture wall time (epoch sec, float)
   "t_msg":  1779579884.118,  # message header.stamp if present, else null
   "topic":  "/referee/robot_status",
   "type":   "rm_interfaces/msg/RobotStatus",
   "data":   { ... msg fields ... }}

Usage:
  bash src/scripts/run_serial.sh &           # start the driver
  python3 src/scripts/capture_serial_topics.py
  # writes logs/serial_capture.json (NDJSON, append mode)

  # custom path / pretty array form / topic filter:
  python3 src/scripts/capture_serial_topics.py \
      --output logs/foo.json \
      --pretty \
      --topics /referee/robot_status /referee/game_status
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict


DEFAULT_TOPICS: list[tuple[str, str]] = [
    ("/serial/gimbal_joint_state", "sensor_msgs/msg/JointState"),
    ("/serial/chassis_attitude", "sensor_msgs/msg/JointState"),
    ("/serial/chassis_status", "std_msgs/msg/Float32MultiArray"),
    ("/referee/robot_status", "rm_interfaces/msg/RobotStatus"),
    ("/referee/game_status", "rm_interfaces/msg/GameStatus"),
    ("/referee/all_robot_hp", "rm_interfaces/msg/GameRobotHP"),
    ("/referee/rfidStatus", "rm_interfaces/msg/RfidStatus"),
]


def _stamp_to_epoch(d: Any) -> float | None:
    """If msg dict has ros2 std_msgs/Header at d['header']['stamp'], return epoch sec."""
    try:
        st = d["header"]["stamp"]
        return float(st["sec"]) + float(st["nanosec"]) * 1e-9
    except (KeyError, TypeError):
        return None


class SerialCapture(Node):
    def __init__(
        self,
        topics: list[tuple[str, str]],
        out_path: Path,
        pretty: bool,
        rate_hz: float,
    ):
        super().__init__("serial_topic_capture")
        self.out_path = out_path
        self.pretty = pretty
        # 每 topic 限速: rate_hz<=0 关闭限速; 否则相邻两条记录至少间隔 1/rate_hz 秒.
        self.min_interval = (1.0 / rate_hz) if rate_hz > 0 else 0.0
        self._last_emit: dict[str, float] = {}
        self._lock = threading.Lock()
        self._records: list[dict] = []  # only used when pretty=True
        self._fp = None if pretty else out_path.open("a", buffering=1, encoding="utf-8")
        self._count = 0
        self._dropped = 0

        # 用 RELIABLE + KEEP_LAST(50) 与 rm_serial_driver 默认 QoS (history=10) 兼容
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        for topic, type_str in topics:
            try:
                msg_cls = get_message(type_str)
            except (ImportError, AttributeError) as e:
                self.get_logger().warn(f"skip {topic}: cannot load {type_str}: {e}")
                continue
            self.create_subscription(
                msg_cls,
                topic,
                lambda msg, t=topic, ty=type_str: self._on_msg(t, ty, msg),
                qos,
            )
            self.get_logger().info(f"subscribed {topic} ({type_str})")
        rate_desc = f"{rate_hz}Hz/topic" if rate_hz > 0 else "no rate limit"
        self.get_logger().info(f"writing -> {out_path} (pretty={pretty}, {rate_desc})")

    def _on_msg(self, topic: str, type_str: str, msg) -> None:
        now = time.time()
        if self.min_interval > 0:
            last = self._last_emit.get(topic, 0.0)
            if now - last < self.min_interval:
                self._dropped += 1
                return
            self._last_emit[topic] = now
        d = message_to_ordereddict(msg)
        rec = {
            "t_wall": now,
            "t_msg": _stamp_to_epoch(d),
            "topic": topic,
            "type": type_str,
            "data": d,
        }
        with self._lock:
            self._count += 1
            if self.pretty:
                self._records.append(rec)
            else:
                self._fp.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")

    def flush_and_close(self) -> None:
        with self._lock:
            if self.pretty:
                self.out_path.write_text(
                    json.dumps(self._records, ensure_ascii=False, indent=2, default=str),
                    encoding="utf-8",
                )
            elif self._fp is not None:
                self._fp.flush()
                self._fp.close()
                self._fp = None
            print(
                f"[capture_serial] wrote {self._count} messages, "
                f"dropped {self._dropped} (rate-limited) -> {self.out_path}",
                file=sys.stderr,
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--output", "-o", type=Path,
        default=Path(__file__).resolve().parent.parent.parent / "logs" / "serial_capture.json",
        help="output file path (default: logs/serial_capture.json)",
    )
    parser.add_argument(
        "--pretty", action="store_true",
        help="dump as a single pretty-printed JSON array on exit (overwrites file). "
             "Default is NDJSON (one JSON per line, append mode, real-time)",
    )
    parser.add_argument(
        "--topics", nargs="+", default=None,
        help="only subscribe these topics (must be in known list, otherwise specify with --type)",
    )
    parser.add_argument(
        "--type", action="append", default=[], metavar="TOPIC=TYPE",
        help="extend known topic list, e.g. --type /custom=std_msgs/msg/Int32 (repeatable)",
    )
    parser.add_argument(
        "--rate-hz", type=float, default=10.0,
        help="per-topic max emit rate (default 10Hz). Use 0 to disable rate limiting.",
    )
    args = parser.parse_args()

    known = dict(DEFAULT_TOPICS)
    for spec in args.type:
        if "=" not in spec:
            parser.error(f"--type expects TOPIC=TYPE, got: {spec}")
        t, ty = spec.split("=", 1)
        known[t] = ty

    if args.topics:
        topics = []
        for t in args.topics:
            if t not in known:
                parser.error(f"topic {t} not in known list; pass --type {t}=<msg_type>")
            topics.append((t, known[t]))
    else:
        topics = list(known.items())

    args.output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = SerialCapture(topics, args.output, args.pretty, args.rate_hz)

    stop_event = threading.Event()

    def _on_signal(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while not stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.flush_and_close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
