#!/usr/bin/env bash
# 一键启动 serial / nav / behavior 三个组件 + capture_serial JSON 抓包,
# 各自落 logs/<name>.log (capture 落 logs/serial_capture.json, 默认 10Hz/topic).
# 终端输出加 [serial] / [nav] / [behavior] / [capture] 前缀便于区分来源.
# Ctrl+C 一并停止 (先 SIGINT 让 ros2 launch 走 lifecycle, 4s 内兜底 SIGKILL).
#
# 用法:
#   bash src/scripts/run_all.sh                                          # 默认: 实车 nav (world=rmuc_2026) + strategy=rmuc_defend + 10Hz capture
#   WORLD=rmul_2026 bash src/scripts/run_all.sh                          # 换场地; 命令行 world:= 优先级更高
#   STRATEGY=b bash src/scripts/run_all.sh
#   CAPTURE_RATE_HZ=50 bash src/scripts/run_all.sh                       # 提高抓包频率
#   CAPTURE_RATE_HZ=0  bash src/scripts/run_all.sh                       # 关限速 (会变非常大)
#   ENABLE_CAPTURE=0   bash src/scripts/run_all.sh                       # 不抓串口
#
# 透传规则:
#   - 命令行 "$@" 全部传给 run_nav.sh (常用的 slam:= / world:= 等)
#   - 串口 / 行为树 / capture 用环境变量配, 见各 wrapper

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 本脚本不 source ROS / install/setup.bash, 子脚本各自处理.
# (子脚本对 AMENT_TRACE_SETUP_FILES unbound 已用 set +u/set -u 包裹, 见 f7f2c59)

# 子任务可调
ENABLE_CAPTURE="${ENABLE_CAPTURE:-1}"
CAPTURE_RATE_HZ="${CAPTURE_RATE_HZ:-10}"

PIDS=()

# bash 在某些情况下 "${ARR[@]}" 数组未赋值时会触发 set -u nounset; 下面遍历都用 :- 兜底.

launch_with_prefix() {
  local tag="$1"; shift
  # 子进程整组放后台. setsid 让子进程拿独立 session, 杀掉时不会牵连父 shell.
  ( "$@" 2>&1 | stdbuf -oL sed -u "s/^/[$tag] /" ) &
  PIDS+=("$!")
}

cleanup() {
  local rc=${1:-0}
  echo
  echo "[run_all] stopping all subprocesses..."
  for pid in "${PIDS[@]:-}"; do
    [ -z "$pid" ] && continue
    kill -INT "$pid" 2>/dev/null || true
  done
  # 给 ros2 launch 一段时间优雅 shutdown lifecycle nodes (4s)
  for _ in $(seq 1 20); do
    local alive=0
    for pid in "${PIDS[@]:-}"; do
      [ -z "$pid" ] && continue
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    [ "$alive" -eq 0 ] && break
    sleep 0.2
  done
  for pid in "${PIDS[@]:-}"; do
    [ -z "$pid" ] && continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "[run_all] force kill pid=$pid"
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  echo "[run_all] done"
  exit "$rc"
}

trap 'cleanup 0' INT TERM HUP

launch_with_prefix "serial"   bash "$SCRIPT_DIR/run_serial.sh"
launch_with_prefix "nav"      bash "$SCRIPT_DIR/run_nav.sh" "$@"
launch_with_prefix "behavior" bash "$SCRIPT_DIR/run_behavior.sh"

if [ "$ENABLE_CAPTURE" = "1" ]; then
  # 等串口 driver 起来一会儿再启 capture (避免抢 topic 还没注册时刷 warn);
  # 用 subshell 异步等待, 本身仍在 PIDS 里以便 cleanup 一并处理.
  ( sleep 5 && exec bash "$SCRIPT_DIR/capture_serial.sh" --rate-hz "$CAPTURE_RATE_HZ" ) \
    2>&1 | stdbuf -oL sed -u "s/^/[capture] /" &
  PIDS+=("$!")
fi

echo "[run_all] started ${#PIDS[@]} processes"
echo "[run_all] logs: logs/serial.log  logs/nav.log  logs/behavior.log"
if [ "$ENABLE_CAPTURE" = "1" ]; then
  echo "[run_all] capture: logs/serial_capture.json (${CAPTURE_RATE_HZ}Hz/topic, 5s 后启动)"
fi
echo "[run_all] Ctrl+C to stop all."

wait
