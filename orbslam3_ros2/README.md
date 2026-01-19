# orbslam3_ros2

ROS 2 Humble 下将 `sensor_msgs/Image` 输入 ORB-SLAM3（单目 `TrackMonocular`）的最小可跑通示例：

Gazebo/相机 → `/camera/image_raw` → 本节点订阅 → ORB-SLAM3 跟踪 → 发布 `/camera_pose` + `/camera_path` → RViz2 可视化；可选 Pangolin Viewer。

> 当前实现刻意 **不依赖 `cv_bridge`**，避免 ROS 自带 OpenCV 与 ORB-SLAM3 使用的 OpenCV 版本混用导致的段错误。

---

## 1. 目录与依赖

### 1.1 关键路径（按本仓库默认位置）

- Workspace：`/home/aaa/learn/class/ros/project/ros2_21_tutorials-master`
- ORB-SLAM3：`/home/aaa/learn/class/ros/project/ORB_SLAM3`
- Vocabulary：`/home/aaa/learn/class/ros/project/ORB_SLAM3/Vocabulary/ORBvoc.txt`
- 相机内参配置：`orbslam3_ros2/config/mbot_camera.yaml`

### 1.2 运行依赖

- ROS 2 Humble
- Gazebo Classic 11（本项目仿真侧）
- ORB-SLAM3 已编译成功（生成 `ORB_SLAM3/lib/libORB_SLAM3.so` 等）
- OpenCV：ORB-SLAM3 使用的 OpenCV（示例环境为 `OpenCV 4.8` 安装在 `/home/aaa/opt/opencv-4.8.0`）
- Pangolin（仅当 `use_viewer:=true` 时需要）

建议额外安装（用于验证/可视化）：
- `rviz2`
- `rqt_image_view`（可选）
- `ros-humble-image-tools`（可选，用 `showimage` 看图）
-
## 2. 为什么要这样做（关键设计点）

### 2.1 为什么移除 `cv_bridge`

**现象**：节点启动后在 ORB-SLAM3 内部崩溃（SIGSEGV），gdb 栈中常见 `cv::Mat::Mat` / `ORBextractor::ComputePyramid()`。

**原因**：ROS 2 Humble 的 `cv_bridge` 通常链接 ROS 自带 OpenCV（例如 4.5），而 ORB-SLAM3 可能链接你本地安装的 OpenCV（例如 4.8）。同一进程中同时加载两个 OpenCV 版本极易 ABI 冲突，导致运行时段错误。

**解决**：本包直接从 `sensor_msgs/Image` 构造 `cv::Mat`，并对 `rgb8/bgr8/mono8` 统一转灰度，避免引入 `cv_bridge`。

### 2.2 为什么需要设置 `LD_LIBRARY_PATH`

运行时需要确保动态链接器优先加载 ORB-SLAM3 所依赖的 `.so`（以及你指定的 OpenCV 版本），否则可能会误加载系统/ROS 的 OpenCV，重新引入版本冲突。

---

## 3. 一键式执行流程（从零到跑通）

最推荐：直接用脚本（已处理 OpenCV ABI 与 Gazebo 崩溃问题）：

```bash
cd /home/aaa/learn/class/ros/project/ros2_21_tutorials-master
/home/aaa/learn/class/ros/project/ros2_21_tutorials-master/orbslam3_ros2/scripts/run_orbslam3_gazebo_demo.sh headless off
```

如果你还想同时打开 Gazebo 窗口（gzclient）：

```bash
# 方式 A：直接 GUI（最吃性能）
/home/aaa/learn/class/ros/project/ros2_21_tutorials-master/orbslam3_ros2/scripts/run_orbslam3_gazebo_demo.sh gui off

# 方式 B：推荐：gzserver 先 headless 跑稳，再单独起 gzclient
/home/aaa/learn/class/ros/project/ros2_21_tutorials-master/orbslam3_ros2/scripts/run_orbslam3_gazebo_demo.sh client off
```

脚本会把本次运行用到的 master 地址写到：
- `/home/aaa/learn/class/ros/project/ros2_21_tutorials-master/.gazebo_master_uri`

你也可以在任意新终端手动打开 Gazebo GUI（连接到同一个 gzserver）：

```bash
export GAZEBO_MASTER_URI="$(cat /home/aaa/learn/class/ros/project/ros2_21_tutorials-master/.gazebo_master_uri)"
gzclient
```

> 建议按“终端 A/B/C/D”分开执行；每个新终端都需要 `source`。

### 3.1 终端 A：构建（首次/改代码后执行）

```bash
# 1) 清理残留 Gazebo 进程（避免 master 端口占用）
killall -9 gzserver gzclient 2>/dev/null || true

# 2) ROS2 + Gazebo 环境
source /opt/ros/humble/setup.bash
source /usr/share/gazebo-11/setup.bash

# 3) 构建（只构建必要包）
cd /home/aaa/learn/class/ros/project/ros2_21_tutorials-master
colcon build --packages-select learning_gazebo orbslam3_ros2 --symlink-install

# 4) 加载 overlay
source install/setup.bash
```

### 3.2 终端 B：启动 Gazebo + 带相机的 mbot

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo-11/setup.bash
source /home/aaa/learn/class/ros/project/ros2_21_tutorials-master/install/setup.bash

# 带 GUI
ros2 launch learning_gazebo load_mbot_camera_into_gazebo.launch.py gui:=true

# 无 GUI（服务器/无桌面）
# ros2 launch learning_gazebo load_mbot_camera_into_gazebo.launch.py gui:=false
```

### 3.3 终端 C：验证相机话题是否在发布

```bash
source /opt/ros/humble/setup.bash
source /home/aaa/learn/class/ros/project/ros2_21_tutorials-master/install/setup.bash

ros2 topic list | grep -E "/camera/image_raw|/camera/camera_info" || true
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw

# 读取一次消息（确认 encoding/尺寸）
ros2 topic echo --once /camera/image_raw
ros2 topic echo --once /camera/camera_info
```

期望看到：
- `/camera/image_raw`：`encoding=rgb8`、`640x360`，频率通常在约 5–10Hz（`gui:=true` 或机器负载高时会更低）

如果你看到 `ros2 node list` 报“同名节点”警告，或 `/camera/image_raw` 掉到 < 1Hz：

---

## 4. 关于“很卡/想用 GPU”

- ORB-SLAM3 的核心跟踪/建图（特征提取、匹配、BA 等）主要是 CPU 计算；本项目目前没有一键切到 GPU 的开关。
- GPU 能明显帮到的主要是“渲染端”：Gazebo GUI、RViz2、Pangolin Viewer（OpenGL）。如果你机器在用软件渲染（llvmpipe），会非常卡。

立刻能缓解卡顿的做法（推荐从上到下逐条试）：
- 关闭 ORB Viewer：脚本第二个参数用 `off`（Pangolin 也会占 GPU/CPU）。
- Gazebo 用 `headless` 或 `client`：SLAM 最稳、相机频率更高；需要时再开 gzclient。
- 降低 SLAM 负载：把 `max_track_fps` 调到 5、把 `map_publish_fps` 调到 1（点云和 RViz 渲染都更轻）。
- 降低相机负载：保持 640×360、10Hz（更低也可，但单目初始化会更难）。

如果你怀疑自己没用到硬件 GPU（渲染很慢），可以检查：
- `glxinfo | grep -E "OpenGL renderer|OpenGL vendor"`
- 先停掉所有旧进程：`pkill -f robot_state_publisher; pkill -f spawn_entity.py; pkill -f gzserver; pkill -f gzclient; pkill -f orbslam3_mono_node`
- 建议先用 `gui:=false` 运行，确认仿真 `gz stats -p` 的 real-time factor 接近 1.0

### 3.4 终端 D：运行 ORB-SLAM3 单目节点

```bash
source /opt/ros/humble/setup.bash
source /home/aaa/learn/class/ros/project/ros2_21_tutorials-master/install/setup.bash

# 关键：不要把 ORB-SLAM3/OpenCV 的 LD_LIBRARY_PATH 传染给 Gazebo（会导致 gzserver exit 255）
# 这里只对 SLAM 进程单独设置：
env LD_LIBRARY_PATH=/home/aaa/opt/opencv-4.8.0/lib:/home/aaa/learn/class/ros/project/ORB_SLAM3/lib:/home/aaa/learn/class/ros/project/ORB_SLAM3/Thirdparty/DBoW2/lib:/home/aaa/learn/class/ros/project/ORB_SLAM3/Thirdparty/g2o/lib:${LD_LIBRARY_PATH} \
ros2 run orbslam3_ros2 orbslam3_mono_node --ros-args \
  -p vocabulary_file:=/home/aaa/learn/class/ros/project/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  -p settings_file:=/home/aaa/learn/class/ros/project/ros2_21_tutorials-master/orbslam3_ros2/config/mbot_camera.yaml \
  -p image_topic:=/camera/image_raw \
  -p publish_pose:=true \
  -p use_viewer:=true

# 如果你只需要 RViz2，不开 Pangolin Viewer：
#   -p use_viewer:=false
```

### 3.5 终端 C（或新终端）：让机器人运动以完成单目初始化

单目 SLAM 常见现象：静止不出位姿，需要移动产生视差。

```bash
# 连续发 5 秒 /cmd_vel
python3 - <<'PY'
import time, subprocess
p = subprocess.Popen(["ros2","topic","pub","/cmd_vel","geometry_msgs/msg/Twist","--rate","10","{linear: {x: 0.15}, angular: {z: 0.1}}"])
time.sleep(5)
p.terminate()
PY
```

### 3.6 验证 `/camera_pose` 与 `/camera_path`

```bash
source /opt/ros/humble/setup.bash
source /home/aaa/learn/class/ros/project/ros2_21_tutorials-master/install/setup.bash

ros2 topic info /camera_pose
ros2 topic echo /camera_pose

# 可选：轨迹
ros2 topic info /camera_path
ros2 topic echo --once /camera_path
```

---

## 4. RViz2 可视化（Current Frame + Map Viewer）

本包额外发布两个可视化话题，避免 Pangolin/EGL 黑屏时“完全没输出”：

- `/orbslam3/current_frame`：当前灰度帧（`sensor_msgs/msg/Image`）
- `/orbslam3/map_points`：当前跟踪到的地图点（`sensor_msgs/msg/PointCloud2`，最多 2000 点，默认 2Hz）

```bash
rviz2 -d /home/aaa/learn/class/ros/project/ros2_21_tutorials-master/orbslam3_ros2/rviz/orbslam3_view.rviz
```

如果你手动配置 RViz2：
- `Global Options -> Fixed Frame` 设为 `map`
- `Add -> Image`，Topic 选 `/orbslam3/current_frame`
- `Add -> PointCloud2`，Topic 选 `/orbslam3/map_points`
- `Add -> Path`，Topic 选 `/camera_path`

---

## 5. 保存轨迹（可选）

### 5.1 录制 rosbag

```bash
ros2 bag record -o orbslam_run /camera_pose /camera_path
# Ctrl+C 停止
```

### 5.2 快速导出为 YAML（简单文本）

```bash
ros2 topic echo /camera_path > camera_path.yaml
```

---

## 6. 常见报错/现象与排查（Troubleshooting）

### 6.1 Gazebo 起不来 / `gzserver exit 255`

- **类型**：启动失败
- **常见原因**：
  - Gazebo 环境未 source、资源/插件路径缺失；或 master 端口/残留进程冲突
  - 把 ORB-SLAM3/OpenCV 的 `LD_LIBRARY_PATH` 提前 `export` 到 Gazebo 进程，导致 Gazebo 插件/依赖库加载错版本直接崩溃
- **解决**：

```bash
killall -9 gzserver gzclient 2>/dev/null || true
source /usr/share/gazebo-11/setup.bash

# 不要在启动 Gazebo 之前 export ORB/OpenCV 的 LD_LIBRARY_PATH
# 改为仅在启动 SLAM 节点时用 `env LD_LIBRARY_PATH=... ros2 run ...`
```

### 6.2 `/camera/image_raw` 存在但 `echo --once` 卡住

- **类型**：无消息/误判
- **原因**：可能没有发布者，或 Gazebo 没真的运行
- **排查**：

```bash
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw
```

如果 `Publisher count: 0` 或 `hz` 不输出，说明没有发布者。

### 6.3 ORB-SLAM3 启动后段错误（SIGSEGV）

- **类型**：运行时崩溃
- **根因**：OpenCV 版本混用（ROS/cv_bridge 的 OpenCV 与 ORB-SLAM3 的 OpenCV）造成 ABI 冲突
- **解决**：
  - 本包已移除 `cv_bridge`
  - 运行前设置 `LD_LIBRARY_PATH` 指向 OpenCV 4.8 与 ORB-SLAM3 动态库目录（见 3.4）

### 6.4 `/camera_pose` 有 publisher 但没有消息

- **类型**：算法未初始化（正常现象）
- **原因**：单目 SLAM 需要运动/视差才能初始化
- **解决**：发送 `/cmd_vel` 让机器人运动 5 秒（见 3.5）

### 6.5 退出时偶发崩溃

- **类型**：退出阶段崩溃（已知问题）
- **建议**：优先用 `-p use_viewer:=false` 运行更稳；若要彻底修复，需抓退出 backtrace 后再针对 ORB-SLAM3 线程/资源释放顺序做优化。

---

## 7. 话题与参数速查

### 7.1 订阅

- `/camera/image_raw` (`sensor_msgs/msg/Image`)

### 7.2 发布

- `/camera_pose` (`geometry_msgs/msg/PoseStamped`)：tracking OK 时发布
- `/camera_path` (`nav_msgs/msg/Path`)：tracking OK 时发布
- `/orbslam3/current_frame` (`sensor_msgs/msg/Image`)：始终发布（有订阅者时）
- `/orbslam3/map_points` (`sensor_msgs/msg/PointCloud2`)：tracking OK 时发布（有订阅者时，默认 2Hz）

### 7.3 参数（启动时 `-p`）

- `vocabulary_file`：ORBvoc.txt 路径
- `settings_file`：相机/ORB 参数 yaml
- `image_topic`：订阅图像话题（默认 `/camera/image_raw`）
- `publish_pose`：是否发布 `/camera_pose` 与 `/camera_path`
- `use_viewer`：是否启动 Pangolin Viewer
