#!/usr/bin/env bash
# 订阅 rm_serial_driver 发布的所有 topic, 落到 logs/serial_capture.json (NDJSON 默认)
# Ctrl+C 优雅退出. 任意 ros2 / Python 参数可透传.
#
# 用法:
#   bash src/scripts/capture_serial.sh                                # 默认: NDJSON, logs/serial_capture.json
#   bash src/scripts/capture_serial.sh --pretty                       # 单一 JSON 数组 (退出时落盘)
#   bash src/scripts/capture_serial.sh -o logs/foo.json --topics /referee/robot_status

set -u

SENTRY_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="$SENTRY_WS/logs"
mkdir -p "$LOG_DIR"
cd "$SENTRY_WS"

# ROS / colcon 的 setup.bash 会引用 AMENT_TRACE_SETUP_FILES 等未定义变量
# 在 set -u 下会炸. 临时关 -u source 完再恢复.
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source install/setup.bash
set -u

exec python3 src/scripts/capture_serial_topics.py "$@"
