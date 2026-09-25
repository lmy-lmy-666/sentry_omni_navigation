#!/usr/bin/env bash
# 启动导航栈, stdout/stderr 同时显示并落 logs/nav.log
# 用法: bash src/scripts/run_nav.sh [ros2 launch args]
# 环境变量:
#   NAV_LAUNCH 选 launch 文件 (默认 rm_navigation_reality_launch.py)
#   WORLD      选地图 / PCD basename (默认 rmuc_2026)
# 注意: 命令行已显式传 world:= 时, 环境变量被忽略 (用户优先).
#
# 默认 world 写为 rmuc_2026 是因为 rm_navigation_reality_launch.py 上游默认是 "204",
# 跟仓库里实车主线场地不一致, 不加会导致地图加载失败 (controller 无 costmap, 无路径).

set -u

SENTRY_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="$SENTRY_WS/logs"
LOG_FILE="$LOG_DIR/nav.log"
NAV_LAUNCH="${NAV_LAUNCH:-rm_navigation_reality_launch.py}"
WORLD="${WORLD:-rmuc_2026}"

# 如果命令行没显式给 world:= 就追加默认值
WORLD_ALREADY_SET=0
for arg in "$@"; do
  case "$arg" in
    world:=*) WORLD_ALREADY_SET=1; break ;;
  esac
done
if [ "$WORLD_ALREADY_SET" = 0 ]; then
  set -- "world:=$WORLD" "$@"
fi

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
  echo "===== $(date '+%F %T') run_nav.sh start launch=$NAV_LAUNCH args=[$*] ====="
} >>"$LOG_FILE"

exec stdbuf -oL -eL ros2 launch sentry_nav_bringup "$NAV_LAUNCH" "$@" \
  2> >(stdbuf -oL -eL tee -a "$LOG_FILE" >&2) \
  > >(stdbuf -oL -eL tee -a "$LOG_FILE")
