#!/usr/bin/env bash
# 启动裁判系统串口驱动, stdout/stderr 同时显示并落 logs/serial.log
# 用法: bash src/scripts/run_serial.sh [extra ros2 args...]

set -u

SENTRY_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="$SENTRY_WS/logs"
LOG_FILE="$LOG_DIR/serial.log"

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

{
  echo
  echo "===== $(date '+%F %T') run_serial.sh start (cwd=$SENTRY_WS) ====="
} >>"$LOG_FILE"

# stdbuf -oL -eL: 行缓冲, 让日志实时写入而不是退出后才 flush
exec stdbuf -oL -eL ros2 launch rm_serial_driver serial_driver.launch.py "$@" \
  2> >(stdbuf -oL -eL tee -a "$LOG_FILE" >&2) \
  > >(stdbuf -oL -eL tee -a "$LOG_FILE")
