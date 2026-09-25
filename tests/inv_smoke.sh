#!/usr/bin/env bash
# sentry_behavior_node (strategy = rmuc_defend) 决策 INV-1~7 行为不变量回归脚本
#
# rmuc_defend 守家逻辑 (无状态互补 guard, 见 strategies.cpp / 方案 §2):
#   - 比赛中 (game_progress==4) 且 hp>=151 且 ammo>=1  -> 驻守防守点 DEFEND=(3.71, -0.61)
#   - 否则 (hp<=150 或 ammo==0)                         -> 回补给点 SUPPLY=(-0.27, -3.94)
#   - game_progress 离开 4 (比赛结束) -> 转 WAIT_START 静默, 等下一场
#
# 目标投递机制: sentry_behavior_node 把 geometry_msgs/PoseStamped publish 到 /goal_pose
# (Nav2 bt_navigator 消费). GoalPublisher 在无订阅者时每 tick 重发, 等订阅者出现再 latch.
# 因此抓取必须在注入裁判数据"之前"就用显式类型挂上订阅 (否则 publisher 尚未创建时
# `ros2 topic echo /goal_pose` 会报 "could not determine type").
#
# 节点启动即运行决策 (无 action 握手), 用 -p strategy:=rmuc_defend 选择策略.
#
# Usage:
#   tests/inv_smoke.sh                          # 跑 7 个用例, 仅 echo PASS/FAIL, exit = FAIL 数
#   tests/inv_smoke.sh --baseline OUTDIR        # 跑用例, 把 /goal_pose 抓取写入 OUTDIR/inv_<N>.log
#   tests/inv_smoke.sh --regress  BASELINE_DIR  # 跑用例, 并将当次输出与 BASELINE_DIR/inv_<N>.log diff
#
# 前置:
#   - 已 source ROS2 jazzy 与 install/setup.bash
#   - sentry_behavior 已编 (install/sentry_behavior 存在)
set -u

# Resolve repo root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( dirname "$SCRIPT_DIR" )"
SHARE="$REPO_ROOT/install/sentry_behavior/share/sentry_behavior"

# rmuc_defend 战术坐标 (来自 strategies.cpp)
DEFEND='3.71,-0.61'
SUPPLY='-0.27,-3.94'
DEFEND_RE='^3\.71,-0\.61$'
SUPPLY_RE='^-0\.27,-3\.94$'

MODE="run"
OUT_DIR=""
BASELINE_DIR=""
while [ $# -gt 0 ]; do
  case "$1" in
    --baseline) MODE="baseline"; OUT_DIR="$2"; shift 2;;
    --regress)  MODE="regress";  BASELINE_DIR="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

if [ "$MODE" = "baseline" ]; then
  mkdir -p "$OUT_DIR"
  RESULT_DIR="$OUT_DIR"            # 只放 inv_*.log
  WORK_DIR="$(mktemp -d -t inv_run_XXXXXX)"  # 中间产物
elif [ "$MODE" = "regress" ]; then
  WORK_DIR="$(mktemp -d -t inv_run_XXXXXX)"
  RESULT_DIR="$WORK_DIR"
else
  WORK_DIR="$(mktemp -d -t inv_run_XXXXXX)"
  RESULT_DIR="$WORK_DIR"
fi

SERVER_LOG="$WORK_DIR/node.log"
SERVER_PID=""
ECHO_PID=""
PASS_COUNT=0
FAIL_COUNT=0

cleanup() {
  if [ -n "$ECHO_PID" ]; then
    kill -KILL "$ECHO_PID" 2>/dev/null || true
  fi
  if [ -n "$SERVER_PID" ]; then
    kill -INT "$SERVER_PID" 2>/dev/null || true
    sleep 1
    kill -KILL "$SERVER_PID" 2>/dev/null || true
  fi
  pkill -KILL -f sentry_behavior_node 2>/dev/null || true
  pkill -KILL -f "ros2 topic echo /goal_pose" 2>/dev/null || true
  pkill -KILL -f "pub_referee.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 启 sentry_behavior_node (执行 rmuc_defend), 参数经 --ros-args -p 注入 (无 params 文件).
# tick_frequency 设高一点 (10Hz) 让 reactive 决策切换 + GoalPublisher "等订阅者后重发"
# 节拍更紧凑; 与生产 params/sentry_behavior.yaml 无关.
start_server() {
  pkill -KILL -f sentry_behavior_node 2>/dev/null || true
  local attempt
  for attempt in 1 2 3; do
    sleep 2
    RCUTILS_LOGGING_BUFFERED_STREAM=0 \
      ros2 run sentry_behavior sentry_behavior_node \
      --ros-args -p strategy:=rmuc_defend -p tick_frequency:=10.0 -p use_sim_time:=false -p viz_enable:=false \
      > "$SERVER_LOG" 2>&1 &
    SERVER_PID=$!
    local waited
    for waited in 1 2 3 4 5 6 7 8 9 10; do
      sleep 1
      if kill -0 "$SERVER_PID" 2>/dev/null && grep -q "sentry_behavior_node ready" "$SERVER_LOG" 2>/dev/null; then
        return 0
      fi
      kill -0 "$SERVER_PID" 2>/dev/null || break
    done
    echo "[WARN] node start attempt $attempt not ready, retrying:" >&2
    tail -5 "$SERVER_LOG" >&2
    kill -KILL "$SERVER_PID" 2>/dev/null || true
    pkill -KILL -f sentry_behavior_node 2>/dev/null || true
    SERVER_PID=""
  done
  echo "[ERR] node did not stay up after retries; log tail:" >&2
  tail -20 "$SERVER_LOG" >&2
  return 1
}

stop_server() {
  if [ -n "$SERVER_PID" ]; then
    kill -INT "$SERVER_PID" 2>/dev/null || true
    sleep 1
    kill -KILL "$SERVER_PID" 2>/dev/null || true
    SERVER_PID=""
  fi
  pkill -KILL -f sentry_behavior_node 2>/dev/null || true
  sleep 1
}

# 用 Python rclpy 发裁判数据 (ros2 topic pub CLI 在 jazzy 对带中文注释的 rm_interfaces
# .msg 解析会触发 rosidl_adapter 内部 bug, 改走 rclpy 兜底). 节点订阅回调把消息写入
# RefereeSnapshot, 值会一直 latch 到下一条覆盖, 故一过性发布即可持续生效.
pub_game_status() {
  local progress="$1"
  python3 "$REPO_ROOT/tests/pub_referee.py" game \
    --progress "$progress" --remain 200 --rate-hz 5 --count 8 \
    > /dev/null 2>&1 &
  disown
}

pub_robot_status() {
  local hp="$1"; local ammo="$2"
  python3 "$REPO_ROOT/tests/pub_referee.py" robot \
    --hp "$hp" --ammo "$ammo" --rate-hz 5 --count 8 \
    > /dev/null 2>&1 &
  disown
}

# 从 /goal_pose TOPIC 抓节点发出的 PoseStamped 坐标序列.
# 用显式类型 echo, 可在 publisher 尚未创建时就挂上订阅 (节点会重发直到有订阅者).
# 解析每个消息块的 x/y -> "%.2f,%.2f", 相邻完全相同坐标折叠成 1 行 (与去重一致).
# 设计: 本函数被调用方以 `&` 后台启动, 它"先"起 echo 再 sleep N 秒; 调用方在 sleep 期间
# 并行做 referee 注入 / 状态切换, 从而保证 echo 早于任何 goal publish 挂上.
capture_goal_pose() {
  local seconds="$1"
  local outfile="$2"
  local raw_file="${outfile%.txt}_raw.txt"
  : > "$raw_file"
  : > "$outfile"
  PYTHONUNBUFFERED=1 stdbuf -oL -eL \
    ros2 topic echo /goal_pose geometry_msgs/msg/PoseStamped --field pose.position \
    > "$raw_file" 2>/dev/null &
  local echo_pid=$!
  ECHO_PID="$echo_pid"
  sleep "$seconds"
  kill -INT "$echo_pid" 2>/dev/null || true
  sleep 0.3
  kill -KILL "$echo_pid" 2>/dev/null || true
  wait "$echo_pid" 2>/dev/null || true
  ECHO_PID=""
  # `ros2 topic echo --field pose.position` 输出形如:
  #   x: 3.71
  #   y: -0.61
  #   z: 0.0
  #   ---
  awk '
    /^[[:space:]]*x:/ { x = $2 + 0; have_x = 1; next }
    /^[[:space:]]*y:/ {
      if (have_x) {
        cur = sprintf("%.2f,%.2f", x, $2 + 0)
        if (cur != last) { print cur; last = cur }
        have_x = 0
      }
      next
    }
  ' "$raw_file" > "$outfile"
}

# regress / baseline 模式公共处理: 处理则计分并返回 0; run 模式返回 1 让调用方继续断言.
_diff_modes() {
  local case_id="$1"; local logfile="$2"
  if [ "$MODE" = "regress" ]; then
    local baseline="$BASELINE_DIR/inv_${case_id}.log"
    if [ ! -f "$baseline" ]; then
      echo "[FAIL] $case_id: baseline missing $baseline"
      FAIL_COUNT=$((FAIL_COUNT+1)); return 0
    fi
    if diff -q "$logfile" "$baseline" > /dev/null; then
      echo "[PASS] $case_id (regress diff clean)"; PASS_COUNT=$((PASS_COUNT+1))
    else
      echo "[FAIL] $case_id (regress diff):"; diff -u "$baseline" "$logfile" | head -20
      FAIL_COUNT=$((FAIL_COUNT+1))
    fi
    return 0
  fi
  if [ "$MODE" = "baseline" ]; then
    local n; n=$(wc -l < "$logfile")
    echo "[REC]  $case_id recorded $n lines (head: $(head -1 "$logfile" 2>/dev/null))"
    PASS_COUNT=$((PASS_COUNT+1)); return 0
  fi
  return 1
}

# 断言 goal 序列中匹配到给定正则 (单坐标用例)
assert_eq_or_diff() {
  local case_id="$1"; local current="$2"; local expected_text="$3"
  local logfile="$RESULT_DIR/inv_${case_id}.log"
  cp "$current" "$logfile"
  _diff_modes "$case_id" "$logfile" && return
  if grep -qE "$expected_text" "$current"; then
    echo "[PASS] $case_id matched: $expected_text"
    PASS_COUNT=$((PASS_COUNT+1))
  else
    echo "[FAIL] $case_id expected /$expected_text/ but got:"
    head -10 "$current"
    FAIL_COUNT=$((FAIL_COUNT+1))
  fi
}

# 断言 goal 序列为空 (无 /goal_pose)
assert_empty_or_diff() {
  local case_id="$1"; local current="$2"
  local logfile="$RESULT_DIR/inv_${case_id}.log"
  cp "$current" "$logfile"
  _diff_modes "$case_id" "$logfile" && return
  if [ ! -s "$current" ]; then
    echo "[PASS] $case_id no goal_pose (file empty)"
    PASS_COUNT=$((PASS_COUNT+1))
  else
    echo "[FAIL] $case_id expected empty but got:"
    head -5 "$current"
    FAIL_COUNT=$((FAIL_COUNT+1))
  fi
}

# 断言给定坐标按顺序作为子序列出现 (允许中间夹其它行; 多坐标流转用例)
assert_subseq_or_diff() {
  local case_id="$1"; local current="$2"; shift 2
  local logfile="$RESULT_DIR/inv_${case_id}.log"
  cp "$current" "$logfile"
  _diff_modes "$case_id" "$logfile" && return
  local -a want=( "$@" )
  local wi=0
  local line
  while IFS= read -r line; do
    if [ "$wi" -lt "${#want[@]}" ] && [ "$line" = "${want[$wi]}" ]; then
      wi=$((wi+1))
    fi
  done < "$current"
  if [ "$wi" -ge "${#want[@]}" ]; then
    echo "[PASS] $case_id ordered subsequence: ${want[*]}"
    PASS_COUNT=$((PASS_COUNT+1))
  else
    echo "[FAIL] $case_id expected ordered subsequence [${want[*]}] but got:"
    cat "$current"
    FAIL_COUNT=$((FAIL_COUNT+1))
  fi
}

# 断言 present 坐标出现 且 absent 坐标不出现 (比赛结束 halt 用例)
assert_present_absent_or_diff() {
  local case_id="$1"; local current="$2"; local present="$3"; local absent="$4"
  local logfile="$RESULT_DIR/inv_${case_id}.log"
  cp "$current" "$logfile"
  _diff_modes "$case_id" "$logfile" && return
  if grep -qxF -e "$present" "$current" && ! grep -qxF -e "$absent" "$current"; then
    echo "[PASS] $case_id present='$present' absent='$absent'"
    PASS_COUNT=$((PASS_COUNT+1))
  else
    echo "[FAIL] $case_id expected present='$present' & absent='$absent' but got:"
    cat "$current"
    FAIL_COUNT=$((FAIL_COUNT+1))
  fi
}

run_case_inv1() {
  echo "=== INV-1: game_progress != 4 -> 等比赛开始, 不发 /goal_pose ==="
  local goal_log="$WORK_DIR/_inv1_goal.txt"
  capture_goal_pose 7 "$goal_log" &
  local cap_pid=$!
  sleep 1
  pub_robot_status 300 100   # 即便状态健康
  sleep 3
  pub_game_status 2          # 非比赛中阶段 -> 不该发 goal
  sleep 2
  wait "$cap_pid" 2>/dev/null || true
  assert_empty_or_diff "1" "$goal_log"
}

run_case_inv2() {
  echo "=== INV-2: progress=4 + hp=300 + ammo=100 -> 驻守防守点 ($DEFEND) ==="
  local goal_log="$WORK_DIR/_inv2_goal.txt"
  capture_goal_pose 12 "$goal_log" &
  local cap_pid=$!
  sleep 1
  pub_robot_status 300 100
  sleep 3
  pub_game_status 4
  sleep 4
  wait "$cap_pid" 2>/dev/null || true
  assert_eq_or_diff "2" "$goal_log" "$DEFEND_RE"
}

run_case_inv3() {
  echo "=== INV-3: progress=4 + ammo=0 -> 切到补给点 ($SUPPLY) ==="
  local goal_log="$WORK_DIR/_inv3_goal.txt"
  capture_goal_pose 12 "$goal_log" &
  local cap_pid=$!
  sleep 1
  pub_robot_status 300 0
  sleep 3
  pub_game_status 4
  sleep 4
  wait "$cap_pid" 2>/dev/null || true
  assert_eq_or_diff "3" "$goal_log" "$SUPPLY_RE"
}

run_case_inv4() {
  echo "=== INV-4: 守点 -> 弹尽回补 -> 恢复返回守点 (DEFEND -> SUPPLY -> DEFEND) ==="
  local goal_log="$WORK_DIR/_inv4_goal.txt"
  capture_goal_pose 26 "$goal_log" &
  local cap_pid=$!
  sleep 1
  pub_robot_status 300 100    # 守点
  sleep 3
  pub_game_status 4
  sleep 5
  pub_robot_status 300 0      # 弹尽 -> 回补给
  sleep 6
  pub_robot_status 400 100    # 补满/恢复 -> 返回守点
  sleep 6
  wait "$cap_pid" 2>/dev/null || true
  assert_subseq_or_diff "4" "$goal_log" "$DEFEND" "$SUPPLY" "$DEFEND"
}

run_case_inv5() {
  echo "=== INV-5: 守点 -> hp<=150 -> 回补给 (DEFEND -> SUPPLY) ==="
  local goal_log="$WORK_DIR/_inv5_goal.txt"
  capture_goal_pose 20 "$goal_log" &
  local cap_pid=$!
  sleep 1
  pub_robot_status 300 100    # 守点
  sleep 3
  pub_game_status 4
  sleep 5
  pub_robot_status 100 100    # 低血 (hp<=150) -> 回补给
  sleep 6
  wait "$cap_pid" 2>/dev/null || true
  assert_subseq_or_diff "5" "$goal_log" "$DEFEND" "$SUPPLY"
}

run_case_inv6() {
  echo "=== INV-6: 比赛结束 (progress 4->5) 停止主决策, 战术状态变化被忽略 ==="
  # 先在比赛中拿到 DEFEND, 再 progress=5 结束比赛 -> 主决策被生命周期父 guard 抢占,
  # 转 WAIT_START 静默. 随后注入 ammo=0 (比赛中本会切 SUPPLY) 用以证明主决策确已停止:
  # SUPPLY 绝不应出现; DEFEND (比赛中已发) 仍在抓取流里.
  local goal_log="$WORK_DIR/_inv6_goal.txt"
  capture_goal_pose 22 "$goal_log" &
  local cap_pid=$!
  sleep 1
  pub_robot_status 300 100
  sleep 3
  pub_game_status 4
  sleep 5                     # 比赛中: 抓到 DEFEND
  pub_game_status 5           # 比赛结束 -> 停止主决策
  sleep 3                     # 等 halt 生效
  pub_robot_status 300 0      # 比赛中本会触发 SUPPLY; 已结束应被忽略
  sleep 4                     # 观察窗口: 不应出现新 goal
  wait "$cap_pid" 2>/dev/null || true
  assert_present_absent_or_diff "6" "$goal_log" "$DEFEND" "$SUPPLY"
}

run_case_inv7() {
  echo "=== INV-7: 节点重启后仍能执行决策并发守点 ($DEFEND) ==="
  stop_server
  start_server || { echo "[FAIL] 7 node restart"; FAIL_COUNT=$((FAIL_COUNT+1)); return; }
  local goal_log="$WORK_DIR/_inv7_goal.txt"
  capture_goal_pose 12 "$goal_log" &
  local cap_pid=$!
  sleep 1
  pub_robot_status 300 100
  sleep 3
  pub_game_status 4
  sleep 4
  wait "$cap_pid" 2>/dev/null || true
  assert_eq_or_diff "7" "$goal_log" "$DEFEND_RE"
}

# Main
echo "Mode: $MODE; Work: $WORK_DIR"
# 每个用例前 restart node: 重置 GoalPublisher 去重状态与 RefereeSnapshot, 保证用例之间
# 互不串扰 (否则上一用例 publish 过的坐标会被去重屏蔽).
start_server || exit 99
run_case_inv1
stop_server; start_server || exit 99
run_case_inv2
stop_server; start_server || exit 99
run_case_inv3
stop_server; start_server || exit 99
run_case_inv4
stop_server; start_server || exit 99
run_case_inv5
stop_server; start_server || exit 99
run_case_inv6
run_case_inv7

echo "=== Summary: PASS=$PASS_COUNT FAIL=$FAIL_COUNT ==="
exit "$FAIL_COUNT"
