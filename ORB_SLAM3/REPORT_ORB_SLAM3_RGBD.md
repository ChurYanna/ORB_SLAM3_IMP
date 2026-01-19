# 模块A：ORB-SLAM3 安装与跑通（RGB-D Demo）

日期：2026-01-06

## 1. 源码获取（Git 仓库地址 + commit）

- 仓库地址：https://github.com/UZ-SLAMLab/ORB_SLAM3.git
- 本次跑通使用的 commit：`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`

> 说明：报告中 commit 用于精确复现实验环境。

## 2. 环境配置说明（OS / ROS2 / 主要依赖版本）

- 操作系统：Ubuntu 22.04.5 LTS（jammy）
- ROS2：本机未安装（`ros2: not found`）
- OpenCV：4.5.4（`pkg-config --modversion opencv4` 与 `python3 -c "import cv2"` 一致）
- Pangolin：安装在 `/usr/local/lib`（不在系统 `ldconfig` cache 中）

> 注：本实验跑通的是 ORB-SLAM3 的纯 C++ Examples（非 ROS2 wrapper）。

## 3. 主要安装/编译步骤（按条列出流程）

> 说明：Linux 上的 `/tmp` 属于临时目录，可能会在重启或系统定期清理时被删除；为了避免第二天需要重复下载数据集，建议把数据集放在 `~/datasets/` 这类持久目录。

下面命令块可直接复制执行（路径按你的 workspace 设定）：

```bash
# 0) 进入仓库
cd /home/aaa/learn/class/ros/project/ORB_SLAM3

# 1) 编译（会在 build/ 下生成库与 Examples 可执行文件）
./build.sh

# 2) 运行真实 TUM RGB-D 数据集（freiburg1_xyz）
#    - 自动下载 tgz、解压、用 evaluation/associate.py 生成 associate.txt
#    - 自动设置 LD_LIBRARY_PATH 以找到 /usr/local/lib 下的 Pangolin 动态库
#    - 数据目录建议使用持久路径（避免 /tmp 被系统清理导致重复下载）
./tools/run_rgbd_tum_freiburg.sh freiburg1_xyz ~/datasets/tum_rgbd

# 3) 轨迹输出位置
ls -l ~/datasets/tum_rgbd/rgbd_dataset_freiburg1_xyz/CameraTrajectory.txt \
  ~/datasets/tum_rgbd/rgbd_dataset_freiburg1_xyz/KeyFrameTrajectory.txt
```

如果你在有图形界面的桌面环境想打开可视化 Viewer，可用下面方式覆盖：

```bash
cd /home/aaa/learn/class/ros/project/ORB_SLAM3
export ORB_SLAM3_NO_VIEWER=0
./tools/run_rgbd_tum_freiburg.sh freiburg1_xyz ~/datasets/tum_rgbd
```

## 4. Demo 跑通结果（可作为报告证据）

- 数据集：TUM RGB-D benchmark：`freiburg1_xyz`（共 792 帧）
- 输出文件：
  - `~/datasets/tum_rgbd/rgbd_dataset_freiburg1_xyz/CameraTrajectory.txt`
  - `~/datasets/tum_rgbd/rgbd_dataset_freiburg1_xyz/KeyFrameTrajectory.txt`
- 轨迹文件样例（前 3 行，证明非空且格式正确）：

```text
CameraTrajectory.txt (head)
1305031102.175304 0.000000000 0.000000000 0.000000000 -0.000000000 -0.000000000 -0.000000000 1.000000000
1305031102.211214 -0.001322251 0.008667253 0.011147271 -0.001824690 -0.003155949 -0.001504217 0.999992251
1305031102.275326 -0.003141040 0.008454057 0.036259249 -0.013670949 -0.008930951 -0.000897793 0.999866307

KeyFrameTrajectory.txt (head)
1305031102.175304 0.0000000 0.0000000 0.0000000 -0.0000000 -0.0000000 -0.0000000 1.0000000
1305031102.475318 -0.0015984 0.0187180 0.1088336 -0.0352173 -0.0249851 0.0007542 0.9990670
1305031103.175452 -0.0196317 0.0690337 0.3492214 -0.0496498 -0.0237251 0.0040401 0.9984767
```

## 5. 真实数据集下载与关联文件生成（过程说明）

### 5.1 选择数据集

- ORB-SLAM3 的 RGB-D 示例程序：Examples/RGB-D/rgbd_tum
- 该程序期望输入：
  - settings yaml（本仓库提供 TUM1.yaml）
  - sequence 目录（包含 rgb/ depth/ 与 rgb.txt depth.txt）
  - association 文件（把 rgb 与 depth 按时间戳配对）

因此选择公开常用且文档完备的 **TUM RGB-D benchmark** 的 `freiburg1_xyz` 作为真实数据序列。

### 5.2 下载链接

脚本使用直链（会自动跟随重定向）：

- https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz

### 5.3 生成 associate.txt

仓库自带工具：evaluation/associate.py

- 输入：`rgb.txt` 与 `depth.txt`
- 输出：每行形如：

```text
<timestamp_rgb> <rgb_path> <timestamp_depth> <depth_path>
```

脚本 tools/run_rgbd_tum_freiburg.sh 会自动执行：

```bash
python3 evaluation/associate.py <seq>/rgb.txt <seq>/depth.txt > <seq>/associate.txt
```

## 6. 至少 2-3 个真实问题与解决方法（含原因分析与最终修复）

### 问题 1：编译报错（C++11 与 Pangolin/sigslot 不兼容）

**现象**：编译时在 `/usr/local/include/sigslot/signal.hpp` 出现类似错误（示例关键词）：
- `std::decay_t` / `std::enable_if_t` / `auto` 返回类型等 C++14 特性缺失

**原因分析**：
- ORB-SLAM3 顶层 CMake 原先强制 C++11
- 新版本 Pangolin 依赖的 sigslot 头文件使用了 C++14 特性

**解决方案**：
- 将工程编译标准提升到 C++14
- 清理 build 并重编译

**最终结果**：
- 成功生成 `lib/libORB_SLAM3.so` 与 `Examples/RGB-D/rgbd_tum`

（对应改动：CMakeLists.txt）

### 问题 2：运行时报错找不到 Pangolin 动态库

**现象**：启动 demo 时动态链接器报错找不到 Pangolin 相关 so（例如 `libpango_windowing.so.0`）。

**原因分析**：
- Pangolin 安装在 `/usr/local/lib`
- 系统默认运行时库搜索路径未包含 `/usr/local/lib`

**解决方案**：
- 运行前临时设置：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
```

**最终结果**：
- demo 可以正常启动并处理数据

（本次已将该环境变量写入 tools/run_rgbd_tum_freiburg.sh）

### 问题 3：保存轨迹阶段段错误（KeyFrame 为空越界）

**现象**：在极短序列或初始化失败时，退出阶段调用保存轨迹函数崩溃。

**原因分析**：
- System::SaveTrajectoryTUM / SaveKeyFrameTrajectoryTUM 内部假设 `vpKFs[0]` 存在
- 当系统没有产生任何 KeyFrame 时会发生越界访问

**解决方案**：
- 在保存函数中加入空 KeyFrame 防护：无 KeyFrame 时直接返回并提示

（对应改动：src/System.cc）

### 问题 4：Viewer（Pangolin/GDK）窗口崩溃及最终修复

**最初现象**：在早期尝试中，程序处理完序列后非正常退出并出现如下错误：
- `Gdk-ERROR ... BadWindow (invalid Window parameter)`（随后 `core dumped`）

**进一步定位**：
- 崩溃高概率发生在主线程释放资源或退出时，Viewer 线程仍在调用 Pangolin/OpenGL/GTK（例如 `cv::imshow` / `pangolin::FinishFrame`），导致异步的 X11/GDK 错误。
- 原因并不是 Pangolin 本身无法工作，而是 Viewer 线程没有被主线程安全地终止和回收：主线程可能已经释放了与窗口相关的对象或上下文，但 Viewer 仍在访问它们。

**最终解决方案（步骤与代码改动）**：
1. 临时/早期策略：为了先保证在无图形服务器上稳定产出轨迹，曾在示例层面增加 headless 检测并在无 DISPLAY 或环境变量 `ORB_SLAM3_NO_VIEWER=1` 时禁用 Viewer；并在 `tools/run_rgbd_tum_freiburg.sh` 默认导出 `ORB_SLAM3_NO_VIEWER=1`，避免直接触发崩溃，便于在服务器上跑通。
2. 永久修复（代码层面，避免仅靠禁用 Viewer）：
   - 在 `System::Shutdown()` 中：
     - 调用 `mpViewer->RequestFinish()` 通知 Viewer 结束循环；
     - 等待 `mpViewer->isFinished()` 为 true；
     - `join()` Viewer 线程（`mptViewer->join()`），确保线程退出后再继续资源回收；
     - `delete` 并置空 `mpViewer` 和 `mptViewer`，避免悬指针。
   - 这样保证 Viewer 的退出与主线程的对象释放有序同步，显著降低 `BadWindow` 类崩溃概率。
3. 脚本默认行为调整：在确认修复后，将 `tools/run_rgbd_tum_freiburg.sh` 的默认 `ORB_SLAM3_NO_VIEWER` 改回 `0`（即默认尝试弹出可视化窗口），同时保留可通过环境变量关闭 Viewer 的灵活性。

**对应改动文件**：
- `src/System.cc`：在 `System::Shutdown()` 中 RequestFinish/wait/join/delete Viewer（线程安全回收）
- `Examples/RGB-D/rgbd_tum.cc`：保留 headless 检测逻辑（当用户确实没有 DISPLAY 时可以不启动 Viewer）
- `tools/run_rgbd_tum_freiburg.sh`：将默认 `ORB_SLAM3_NO_VIEWER` 改为 `0`（可视化默认开启），并仍导出 `LD_LIBRARY_PATH`。

**最终结果**：
- 在有图形环境（或正确的 X11 转发）下，运行脚本会弹出 Pangolin/Viewer 窗口进行交互与可视化；
- 在无图形环境下仍可通过 `export ORB_SLAM3_NO_VIEWER=1` 关闭 Viewer 保证稳定运行并输出轨迹；
- 修复后在本次环境中使用 `./tools/run_rgbd_tum_freiburg.sh freiburg1_xyz /tmp/tum_rgbd`（可视化默认开启）能够正常运行且没有再次出现 `Gdk-ERROR BadWindow` 崩溃。

## 7. 本次为跑通 Demo 的代码/脚本改动清单（便于答辩说明）

- Examples/RGB-D/rgbd_tum.cc
  - 新增 headless 检测：无 DISPLAY 或 ORB_SLAM3_NO_VIEWER=1 时禁用 Viewer
- evaluation/associate.py
  - 修复为兼容 Python3（字典 keys 视图、默认参数等）
- tools/run_rgbd_tum_freiburg.sh
  - 自动下载/解压 TUM RGB-D 数据集
  - 自动生成 associate.txt
  - 设置 LD_LIBRARY_PATH
  - 默认 ORB_SLAM3_NO_VIEWER=1
- src/System.cc
  - 保存轨迹时无 KeyFrame 的防护，避免段错误

---

如果需要把这个报告进一步“对标课程模板”（例如加截图位置、把每个报错贴完整输出、或者补上依赖安装的 apt/pip 命令），我也可以继续帮你补齐。
