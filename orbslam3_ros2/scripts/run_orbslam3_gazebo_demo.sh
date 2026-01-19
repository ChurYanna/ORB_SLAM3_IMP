#!/usr/bin/env bash
set -eo pipefail

WS="/home/aaa/learn/class/ros/project/ros2_21_tutorials-master"
ORB="/home/aaa/learn/class/ros/project/ORB_SLAM3"
OPENCV48="/home/aaa/opt/opencv-4.8.0/lib"

VOCAB="$ORB/Vocabulary/ORBvoc.txt"
SETTINGS="$WS/orbslam3_ros2/config/mbot_camera.yaml"

GUI_MODE="${1:-headless}"   # headless | gui
VIEWER="${2:-on}"           # on | off

usage(){
  cat <<'USAGE'
Usage:
  run_orbslam3_gazebo_demo.sh <headless|gui|client> <on|off>

Modes:
  headless : start gzserver only (fastest, recommended)
  gui      : start gzserver + gzclient via launch
  client   : start gzserver headless, then start gzclient separately (useful to keep SLAM stable)

Viewer:
  on/off controls ORB-SLAM3 Pangolin viewer (off recommended for performance)
USAGE
}

if [[ "$GUI_MODE" == "-h" || "$GUI_MODE" == "--help" ]]; then
  usage
  exit 0
fi

if [[ "$GUI_MODE" != "headless" && "$GUI_MODE" != "gui" && "$GUI_MODE" != "client" ]]; then
  echo "[error] unknown GUI mode: $GUI_MODE" >&2
  usage
  exit 1
fi

if [[ ! -f "$VOCAB" ]]; then
  echo "[error] ORB vocabulary not found: $VOCAB" >&2
  exit 1
fi

if [[ ! -f "$SETTINGS" ]]; then
  echo "[error] settings not found: $SETTINGS" >&2
  exit 1
fi

# Safe sourcing: some setup scripts reference unset variables which breaks when
# `set -u` (nounset) is active. Temporarily disable nounset when sourcing.
safe_source(){
  local f="$1"
  local had_nounset=0
  case "$-" in
    *u*) had_nounset=1 ;;
  esac
  if [[ -f "$f" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$f"
    if [[ $had_nounset -eq 1 ]]; then
      set -u
    fi
  else
    echo "[warn] source file not found: $f"
  fi
}

echo "[env] sourcing ROS/Gazebo/workspace overlays"
safe_source /opt/ros/humble/setup.bash
safe_source /usr/share/gazebo-11/setup.bash
safe_source "$WS/install/setup.bash"

echo "[env] preparing OpenCV 4.8 + ORB-SLAM3 runtime libs (only for SLAM node)"
ORB_SLAM3_LD_LIBRARY_PATH="$OPENCV48:$ORB/lib:$ORB/Thirdparty/DBoW2/lib:$ORB/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"

echo "[env] GUI_MODE=$GUI_MODE  VIEWER=$VIEWER"

# Use a per-run Gazebo master port to avoid conflicts with stale gzserver instances.
# Default Gazebo master is 11345; if that port is already taken, gzserver may exit(255).
pick_gazebo_port(){
  local base=11345
  local port
  for port in $(seq "$base" $((base+20))); do
    if ! ss -ltn 2>/dev/null | awk '{print $4}' | grep -q ":${port}$"; then
      echo "$port"
      return 0
    fi
  done
  echo "$base"
  return 0
}

GAZEBO_PORT="$(pick_gazebo_port)"
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"
echo "[env] GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"

# Persist master uri for convenience (so you can open gzclient in a new terminal).
echo "$GAZEBO_MASTER_URI" > "$WS/.gazebo_master_uri"
echo "[env] wrote $WS/.gazebo_master_uri"

# 0) cleanup
"$WS/orbslam3_ros2/scripts/cleanup_orbslam3_gazebo.sh"

# 1) (optional) build once
# If you already built successfully, you can comment this out.
echo "[build] colcon build (orbslam3_ros2 + learning_gazebo)"
cd "$WS"
colcon build --packages-select learning_gazebo orbslam3_ros2 --symlink-install

# Re-source overlay after build (use safe_source to avoid nounset failures)
safe_source "$WS/install/setup.bash"

# 2) start Gazebo
if [[ "$GUI_MODE" == "gui" ]]; then
  echo "[gazebo] launching with GUI"
  ros2 launch learning_gazebo load_mbot_camera_into_gazebo.launch.py gui:=true &
else
  echo "[gazebo] launching headless (recommended for stable camera Hz)"
  ros2 launch learning_gazebo load_mbot_camera_into_gazebo.launch.py gui:=false &
fi
GZ_PID=$!

GZCLIENT_PID=""

# Ensure we stop Gazebo on exit
cleanup() {
  echo "[exit] stopping processes"
  if [[ -n "${GZCLIENT_PID}" ]]; then
    kill "$GZCLIENT_PID" 2>/dev/null || true
  fi
  kill "$GZ_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 6

# If requested, start gzclient as a separate process (keeps gzserver launch minimal).
if [[ "$GUI_MODE" == "client" ]]; then
  echo "[gazebo] starting gzclient (separate process)"
  gzclient &
  GZCLIENT_PID=$!
  echo "[gazebo] gzclient pid=$GZCLIENT_PID"
fi

# 3) verify topics
echo "[check] topics"
ros2 topic list | egrep '/camera/image_raw|/camera/camera_info|/clock|/cmd_vel|/odom' | sort || true

echo "[check] /camera/image_raw publishers"
ros2 topic info /camera/image_raw -v || true

echo "[check] /camera/image_raw hz (8s)"
(timeout 8 ros2 topic hz /camera/image_raw) || true

echo "[check] /camera/camera_info hz (8s)"
(timeout 8 ros2 topic hz /camera/camera_info) || true

# Bail out early if Gazebo died (common when LD_LIBRARY_PATH breaks plugins)
if ! kill -0 "$GZ_PID" 2>/dev/null; then
  echo "[error] Gazebo process exited early (gzserver/gazebo crashed)." >&2
  echo "        Try GUI mode: $0 gui" >&2
  exit 2
fi

echo "[check] waiting for first /camera/image_raw message (timeout 15s)"
if ! timeout 15 ros2 topic echo --once /camera/image_raw >/dev/null 2>&1; then
  echo "[error] No /camera/image_raw messages received within 15s." >&2
  echo "        Gazebo may be running but camera sensor is not publishing." >&2
  echo "        Try GUI mode: $0 gui" >&2
  exit 3
fi

# 4) start ORB-SLAM3 node
USE_VIEWER=false
if [[ "$VIEWER" == "on" ]]; then
  USE_VIEWER=true
fi

echo "[slam] starting orbslam3 mono (viewer=$USE_VIEWER)"
env LD_LIBRARY_PATH="$ORB_SLAM3_LD_LIBRARY_PATH" \
ros2 run orbslam3_ros2 orbslam3_mono_node --ros-args \
  --log-level info \
  -p vocabulary_file:="$VOCAB" \
  -p settings_file:="$SETTINGS" \
  -p image_topic:=/camera/image_raw \
  -p target_width:=320 \
  -p target_height:=180 \
  -p publish_pose:=true \
  -p use_viewer:="$USE_VIEWER" &
SLAM_PID=$!

echo "[slam] pid=$SLAM_PID"

echo "[check] /camera_pose info (may be empty before initialization)"
ros2 topic info /camera_pose || true

cat <<'MSG'

============================================================
下一步（需要你手动操作）：
1) 打开一个新终端运行键盘控制：
   source /opt/ros/humble/setup.bash
   source /home/aaa/learn/class/ros/project/ros2_21_tutorials-master/install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard

2) 让机器人连续运动 10~20 秒（产生视差），单目初始化后：
   - Pangolin Viewer 会开始更新（如果 viewer=on）
   - /camera_pose 会开始连续输出

3) 新终端验证：
   ros2 topic echo /camera_pose
   ros2 topic hz /camera_pose

4) 推荐用 RViz2 看 current frame + map points（比 Pangolin 更稳，尤其是 headless）：
  rviz2 -d /home/aaa/learn/class/ros/project/ros2_21_tutorials-master/orbslam3_ros2/rviz/orbslam3_view.rviz
============================================================

MSG

echo "[run] press Ctrl-C to stop demo"
wait "$SLAM_PID"
