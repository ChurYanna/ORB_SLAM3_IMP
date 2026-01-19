#!/usr/bin/env bash
set -euo pipefail

# Cleanup stale processes that commonly cause:
# - duplicate node name warnings
# - very low /camera/image_raw rate
# - confused subscriptions

echo "[cleanup] killing stale processes (ignore 'no process found')"

kill_matches(){
  local pattern="${1}"
  # find matching pids, exclude this script and its parent shell
  local pids=()
  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    pids+=("$pid")
  done < <(pgrep -f "${pattern}" 2>/dev/null || true)

  for pid in "${pids[@]}"; do
    if [[ "${pid}" == "$$" ]]; then
      continue
    fi
    if [[ "${pid}" == "${PPID}" ]]; then
      continue
    fi
    echo "[cleanup] killing pid ${pid} (match: ${pattern})"
    kill -TERM "${pid}" 2>/dev/null || true
  done

  # Escalate if still alive (common for gzserver/spawn_entity hanging on shutdown)
  sleep 0.5
  for pid in "${pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      echo "[cleanup] killing pid ${pid} with SIGKILL (still alive)"
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
}

# safer kills for known patterns
kill_matches orbslam3_mono_node
kill_matches "ros2 run orbslam3_ros2 orbslam3_mono_node"
kill_matches robot_state_publisher
kill_matches spawn_entity.py
kill_matches gzserver
kill_matches gzclient

# Avoid overly-broad matches like "gazebo" which can kill this project's scripts
# (their filenames include 'gazebo'). Prefer killing the actual Gazebo processes
# and the specific launch processes.
kill_matches "ros2 launch learning_gazebo"
kill_matches "load_mbot_camera_into_gazebo.launch.py"

sleep 1

if command -v ros2 >/dev/null 2>&1; then
  echo "[cleanup] restarting ros2 daemon"
  ros2 daemon stop || true
  ros2 daemon start || true
fi

echo "[cleanup] done"
