# 模块 A：ORB‑SLAM3 安装与跑通

## 1 引言
本模块目标：在指定环境下（不依赖系统级全局安装）完成 ORB‑SLAM3 的源码获取、编译、运行至少一个官方 demo（本报告以 RGB‑D demo 为示例），并完整记录环境配置、关键安装步骤、遇到的问题与解决过程，提供可复现的运行命令与关键修改说明。

## 2 源码信息
- 仓库地址：https://github.com/UZ-SLAMLab/ORB_SLAM3.git
- 使用提交：`4452a3c`
- 工作路径：仓库根目录（后续命令均假定在该目录下执行）

建议插入：`git remote -v` 与 `git rev-parse --short HEAD` 输出截图。

## 3 环境配置（现有系统）
- 操作系统：Ubuntu 22.04.5 LTS
- OpenCV：4.5.4
- C++ 标准：C++14（必须）
- 其他依赖：Pangolin（已安装到 `/usr/local/lib`）、Eigen、g2o、Sophus（按仓库 `Dependencies.md` 安装）

[表格: 环境信息（列：项目 / 版本 / 备注）]

## 4 主要安装与构建步骤（简要、可复制）
1. 克隆源码并切到指定 commit：

```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
git checkout 4452a3c
```

2. 安装系统依赖（示例）：

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
  libopencv-dev libeigen3-dev libboost-all-dev libglew-dev \
  libpython3-dev libturbojpeg0-dev
```

3. 安装第三方库（Pangolin、DBoW2、g2o 等），确保 Pangolin 的动态库安装到 `/usr/local/lib`，或记录安装路径。

4. 配置并编译（必须使用 C++14）：

```bash
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_CXX_STANDARD=14 ..
cmake --build . -j$(nproc)
```

5. 运行示例（建议先设置库路径）：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# 无显示（headless）运行示例：
export ORB_SLAM3_NO_VIEWER=1
./tools/run_rgbd_tum_freiburg.sh freiburg1_xyz ~/datasets/tum_rgbd
```

建议插入：编译成功时 `cmake` 与 `make` 的关键输出截图；运行 demo 的 terminal 输出截图或日志片段。

## 5 运行示例（两种模式）
 - Headless（无 GUI）：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ORB_SLAM3_NO_VIEWER=1
./tools/run_rgbd_tum_freiburg.sh freiburg1_xyz ~/datasets/tum_rgbd
```

 - 启用 Viewer（需有 X DISPLAY，例如 ssh -X 或本机有 X server）：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ORB_SLAM3_NO_VIEWER=0
./tools/run_rgbd_tum_freiburg.sh freiburg1_xyz ~/datasets/tum_rgbd
```

运行结束后应在数据目录看到：`CameraTrajectory.txt`、`KeyFrameTrajectory.txt` 等文件。

建议插入：运行时关键日志片段与生成文件的 `ls -lh` 截图。

## 6 遇到的问题、分析与解决（按“报错 → 分析 → 解决”给出）

### 问题 1 — 编译期：C++ 标准/符号不匹配（Pangolin/sigslot 引起）
- 报错示例：

```text
error: ‘make_unique’ is not a member of ‘std’
或与 Pangolin / sigslot 相关的编译模板错误
```

- 分析：默认编译标准低于 C++14，Pangolin/sigslot 使用 C++14 特性。
- 解决：在 CMake 配置中强制指定 `CMAKE_CXX_STANDARD=14`（见第 4 节）。

### 问题 2 — 运行期：动态链接库找不到 Pangolin
- 报错示例：

```text
error while loading shared libraries: libpangolin.so: cannot open shared object file: No such file or directory
```

- 分析：系统未在动态库搜索路径中找到 `/usr/local/lib` 下的库，或 ld cache 未更新。
- 解决：运行前导出 `LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH`，或将路径加入 `/etc/ld.so.conf.d/` 并 `sudo ldconfig`。

建议插入：`ldconfig -p | grep pangolin` 输出与修复前后对比截图。

### 问题 3 — 运行期：轨迹保存阶段崩溃（`SaveTrajectoryTUM` 访问空 KeyFrames）
- 症状：在程序退出并保存轨迹时崩溃，backtrace 指向 `SaveTrajectoryTUM` 内 `vpKFs[0]` 访问。
- 分析：当程序未成功初始化或运行时间过短时，关键帧列表可能为空，直接索引越界。
- 解决：在 `SaveTrajectoryTUM` 中加入空检查，若 `vpKFs.empty()` 则直接返回或写空文件，避免越界。

修改文件：[src/System.cc](src/System.cc)

建议插入：崩溃的 stacktrace 截图与修改前后代码片段对比。

### 问题 4 — 运行期：Viewer 关闭时的 GDK BadWindow 崩溃
- 症状：在窗口关闭或程序退出时出现 `Gdk-ERROR BadWindow`，程序 core dump。
- 分析：Viewer 渲染线程与主线程没有正确同步关闭；资源在 Viewer 仍在使用时被释放。
- 解决：在 `System::Shutdown()` 中按顺序发出 Viewer 结束请求 → 等待 `viewer->isFinished()` → `join()` Viewer 线程 → 删除 `viewer` 和线程对象；同时在示例程序中增加 headless 检测（检查 `DISPLAY` 环境或 `ORB_SLAM3_NO_VIEWER`）。

修改文件：[src/System.cc](src/System.cc)、[Examples/RGB-D/rgbd_tum.cc](Examples/RGB-D/rgbd_tum.cc)

建议插入：崩溃日志、修改前后关键代码片段与运行成功的 Viewer 截图（若可用）。

### 问题 5 — 数据下载被本地代理阻断（网络问题）
- 报错示例：

```text
curl: (7) Failed to connect to 127.0.0.1 port 7897: Connection refused
```

- 分析：系统环境变量配置了本地代理（127.0.0.1:7897）但代理服务未运行，导致下载失败。
- 解决：临时 `unset http_proxy https_proxy ALL_PROXY` 或使用 `curl --noproxy '*'`；或在其他可联网机器上下载后 `scp` 到本机的 `~/datasets/tum_rgbd`。

建议插入：curl 报错截图与修复后下载日志。

## 7 关键修改与脚本（便于审阅）
- 我们在仓库中做了下列修改 / 新增脚本（请在报告附录提交 diff 或列出补丁）：
  - [Examples/RGB-D/rgbd_tum.cc](Examples/RGB-D/rgbd_tum.cc) —— 增加 headless 检测并传入 `useViewer` 参数。
  - [src/System.cc](src/System.cc) —— 修复 `SaveTrajectoryTUM` 的空列表检查；改进 `Shutdown()` 以安全关闭 Viewer。
  - [evaluation/associate.py](evaluation/associate.py) —— 修复 Python3 兼容性，生成 `associate.txt`。
  - `tools/run_rgbd_tum_freiburg.sh` —— 一键下载/解压/associate/运行脚本，默认数据目录 `~/datasets/tum_rgbd`，并设置 `LD_LIBRARY_PATH`。
  - `tools/make_synth_tum_rgbd.py`, `tools/run_rgbd_synth.sh` —— 合成数据与快速验证脚本。

建议插入：每个文件的关键修改片段（代码块），以及完整 patch 附录。

## 8 复现清单（简短、按步骤）
1. 克隆并切到 `4452a3c`。  
2. 安装系统依赖并编译第三方库（Pangolin、g2o 等）。  
3. 在仓库 `build/` 中运行 `cmake -D CMAKE_CXX_STANDARD=14 ..` 并 `make -j`。  
4. 设置库路径并运行脚本：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ORB_SLAM3_NO_VIEWER=1
./tools/run_rgbd_tum_freiburg.sh freiburg1_xyz ~/datasets/tum_rgbd
```

5. 检查输出：`CameraTrajectory.txt`, `KeyFrameTrajectory.txt`。

## 9 总结与建议
- 已解决的关键问题：C++ 标准强制为 C++14、Pangolin 动態库路径、轨迹保存越界、Viewer 线程安全关闭、网络代理导致的数据下载失败。
- 建议：将常用运行环境与路径写成 `env_setup.sh`，并将 `LD_LIBRARY_PATH` / headless 检测集成到运行脚本；把修改的补丁提交为分支或 patch 以供授课组审阅；如需 GUI，请确保 `DISPLAY` 可用或采用 X11 转发。

## 附录 A — 应放入报告的图片与表格清单（占位）
- 系统信息截图（`lsb_release -a`）  [插图: 系统信息]  
- 编译输出关键截图  [插图: cmake/make 输出]  
- 运行日志（vocabulary 加载、地图点数、轨迹保存）  [插图: 运行日志片段]  
- Viewer 截图（若启用）  [插图: Viewer 可视化]  
- 依赖版本表  [表格: 依赖 / 版本 / 备注]  

## 附录 B — 关键补丁位置（供评阅）
- `src/System.cc`  （SaveTrajectoryTUM 检查、Shutdown 改进）  
- `Examples/RGB-D/rgbd_tum.cc`（headless 检测）  
- `evaluation/associate.py`（Python3 兼容）  
- `tools/run_rgbd_tum_freiburg.sh`（一键运行脚本）

---
报告由仓库内实际修改与运行记录整理，若需要我可以：
- 将上述 Markdown 转成 PDF；
- 将“建议插入”的截图位置替换为实际截图（请上传对应图片或允许我在本机运行并截取）；
- 提交完整的 patch 文件或把修改 commit 到新分支（需你确认并授权提交）。
