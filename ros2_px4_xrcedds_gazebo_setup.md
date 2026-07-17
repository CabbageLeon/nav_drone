# ROS2 + PX4 + Micro XRCE-DDS + Gazebo 仿真环境搭建指南（Ubuntu 22.04）

> **方案A：Micro XRCE-DDS 方案（PX4 官方推荐）**
>
> 最后更新：2026-07-11
>
> 适配组合：**Ubuntu 22.04 + ROS2 Humble + Gazebo Garden + PX4 v1.15/v1.16 + Micro XRCE-DDS**

---

## 目录

1. [方案概述：为什么选 Micro XRCE-DDS](#1-方案概述为什么选-micro-xrce-dds)
2. [兼容性总览](#2-兼容性总览)
3. [系统基础配置](#3-系统基础配置)
4. [安装 ROS2 Humble](#4-安装-ros2-humble)
5. [安装 Gazebo Garden](#5-安装-gazebo-garden)
6. [安装 PX4 Autopilot](#6-安装-px4-autopilot)
7. [安装 Micro XRCE-DDS Agent](#7-安装-micro-xrce-dds-agent)
8. [搭建 PX4-ROS2 工作空间](#8-搭建-px4-ros2-工作空间)
9. [安装 QGroundControl（可选）](#9-安装-qgroundcontrol可选)
10. [启动仿真](#10-启动仿真)
11. [验证与测试](#11-验证与测试)
12. [Offboard 控制示例](#12-offboard-控制示例)
13. [常见问题与避坑](#13-常见问题与避坑)
14. [参考来源](#14-参考来源)

---

## 1. 方案概述：为什么选 Micro XRCE-DDS？

### 方案对比

| | **Micro XRCE-DDS（本方案）** | **MAVROS（传统方案）** |
|---|---|---|
| 官方推荐 | :white_check_mark: PX4 官方推荐 | :x: 标记为 Legacy |
| 通信机制 | PX4 内置 uXRCE-DDS 客户端 ↔ ROS2 DDS | MAVLink ↔ ROS2 协议转换网关 |
| 架构 | 点对点，PX4 直连 ROS2 中间件 | 中间桥梁，额外进程 |
| 客户端大小 | ~75KB | 多个独立插件节点 |
| Topic 命名 | `/fmu/out/*`（传感器数据）, `/fmu/in/*`（控制指令） | `/mavros/*` |
| 实时性 | :white_check_mark: 更优，DDS QoS 可配 | :yellow_circle: 经 MAVLink 序列化 |
| 多机支持 | :white_check_mark: DDS 分区天然支持 | :yellow_circle: 需配置不同 MAVLink ID |
| ROS2 Humble 支持 | :white_check_mark: 原生 | :white_check_mark: apt 安装 |
| 学习曲线 | :yellow_circle: 较陡，文档偏少 | :white_check_mark: 社区资料丰富 |

### 架构数据流

```
┌──────────┐   DDS Topic    ┌──────────────────┐   UDP:8888    ┌────────────┐   Gazebo Transport   ┌────────────────┐
│ ROS2 Node │ ◄────────────► │ Micro XRCE-DDS    │ ◄──────────► │ PX4 SITL    │ ◄──────────────────► │ Gazebo Garden  │
│           │                │ Agent             │              │ (uXRCE-DDS  │                      │ (gz-sim)       │
│ /fmu/out/ │                │ (独立进程)         │              │  Client)    │                      │                │
│ /fmu/in/  │                │ MicroXRCEAgent    │              │             │                      │                │
└──────────┘                └──────────────────┘              └──────┬─────┘                      └────────────────┘
                                                                     │ UDP:14550
                                                                     ▼
                                                              ┌──────────────┐
                                                              │ QGroundControl│
                                                              └──────────────┘
```

**核心原理：**
- PX4 固件编译时内置了 uXRCE-DDS **客户端**（源码位于 `src/modules/uxrce_dds_client/`）
- ROS2 侧运行 **Micro XRCE-DDS Agent**，作为 DDS 网络中的一员
- Agent 和 Client 之间通过 UDP（默认 8888 端口）通信
- Agent 将 PX4 消息发布为 ROS2 Topic，同时将 ROS2 Topic 转发给 PX4
- 这是 PX4 开发团队推动的现代化通信架构，MAVROS 方案已标记为 Legacy

---

## 2. 兼容性总览

| 组件 | 兼容版本 | 约束条件 |
|------|----------|----------|
| **Ubuntu** | **22.04 LTS (Jammy)** | 唯一推荐，与 ROS2 Humble 同周期 |
| **ROS 2** | **Humble Hawksbill** | LTS 至 2027.5，依赖 Python 3.10 |
| **Python** | **3.10.x** | :x: 不得升级至 3.11/3.12 |
| **PX4** | **v1.14 / v1.15 / v1.16** | v1.16 最新 LTS；`main` 分支也可用；v1.13+ 即内置 uXRCE-DDS Client |
| **Gazebo** | **Gazebo Garden (`gz-garden`)** | :white_check_mark: **首选**，与 Humble ros_gz_bridge 兼容性最好 |
| | Gazebo Harmonic (`gz-harmonic`) | :yellow_circle: 可用但 `ros-humble-ros-gzharmonic-bridge` 与 `ros-humble-ros-gz-bridge` 存在包冲突（[gazebosim/ros_gz#755](https://github.com/gazebosim/ros_gz/issues/755)） |
| | Gazebo Classic (`gazebo11`) | :x: **已废弃**，PX4 v1.15+ 不再支持 |
| **Empy** | **==3.3.4** | :x: 4.x 会使 PX4 宏模板编译失败 |
| **setuptools** | **packaging==22.0** | 高版本导致 `colcon build` 报错 |
| **Micro XRCE-DDS Agent** | **v2.4.x+（main 分支）** | 从 eProsima 官方仓库编译 |
| **px4_msgs** | PX4 `main` 分支对应版本 | 需与 PX4 版本匹配，msg 定义有时会更新 |
| **QGroundControl** | 最新 AppImage | 通过 UDP 14550 连接 |

### ROS2 发行版兼容

| ROS2 发行版 | Ubuntu 版本 | Micro XRCE-DDS 支持 |
|---|---|---|
| **Humble** (LTS) | **22.04 Jammy** | :white_check_mark: **首选** |
| Iron (EOL) | 22.04 Jammy | :yellow_circle: 可用但已停止维护 |
| **Jazzy** (LTS) | 24.04 Noble | :white_check_mark: 可用（需要 Ubuntu 24.04） |
| Rolling | 24.04 Noble | :white_check_mark: 滚动发布 |

---

## 3. 系统基础配置

### 3.1 系统更新与基础工具

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础开发工具链
sudo apt install -y \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    ninja-build \
    python3-pip \
    python3-colcon-common-extensions \
    python3-argcomplete \
    python3-rosdep \
    python3-vcstool \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev \
    libgazebo11-dev  # PX4 编译可能依赖部分 Gazebo 头文件
```

### 3.2 设置 Locale

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3.3 确认 Python 版本

```bash
python3 --version
```

> :warning: **必须输出 `Python 3.10.x`**。Ubuntu 22.04 默认就是 3.10，千万不要手动升级。ROS2 Humble 工具链、PX4 构建系统、colcon 都与 Python 3.10 绑定。

确认 pip 指向正确版本：

```bash
pip3 --version
# 应显示: pip ... from .../python3.10/...
```

### 3.4 安装并初始化 rosdep

```bash
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
```

> 若网络慢导致 `rosdep update` 超时，使用中科大镜像：
> ```bash
> export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/ros/rosdistro/index-v4.yaml
> rosdep update
> ```

---

## 4. 安装 ROS2 Humble

### 4.1 添加 ROS2 软件源

```bash
# 启用 Universe 仓库
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# 添加 ROS2 GPG 密钥
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加 ROS2 软件源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

> :bulb: **国内加速：** 网络慢时使用清华镜像：
> ```bash
> sudo curl -sSL https://mirrors.tuna.tsinghua.edu.cn/ros2/ros.key \
>     -o /usr/share/keyrings/ros-archive-keyring.gpg
> echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
>     https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | \
>     sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
> ```

### 4.2 安装 ROS2 Humble 桌面版

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

### 4.3 配置环境变量

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4.4 验证安装

打开两个终端测试经典小乌龟：

```bash
# 终端 1
ros2 run turtlesim turtlesim_node

# 终端 2
ros2 run turtlesim turtle_teleop_key
```

能用键盘控制乌龟移动即成功。

### 4.5 安装 colcon 及相关依赖

```bash
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool

# 可选：colcon 构建加速
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update
```

---

## 5. 安装 Gazebo Garden

### 5.1 版本选择说明

PX4 官方已经将仿真迁移至新一代 Gazebo（原 Ignition Gazebo），Gazebo Classic 已废弃。

| Gazebo 版本 | ROS2 Humble 兼容性 | 推荐 |
|---|---|---|
| **Garden (`gz-garden`)** | :white_check_mark: 最佳 | :star: **本方案选择** |
| Harmonic (`gz-harmonic`) | :yellow_circle: 有包冲突风险 | 不推荐新手 |
| Fortress (`ignition-fortress`) | :yellow_circle: 旧版 | 不推荐 |
| Classic (`gazebo11`) | :x: PX4 v1.15+ 不再支持 | 绝对不要用 |

Garden 的 `ros-humble-ros-gzgarden` 二进制包可直接安装，无冲突——这是最省心的选择。

### 5.2 安装步骤

```bash
# 添加 OSRF Gazebo 软件源
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 安装 Gazebo Garden
sudo apt update
sudo apt install -y gz-garden
```

### 5.3 验证安装

```bash
gz sim --version    # 应显示 Garden 版本号
gz sim --help       # 应输出帮助信息
```

### 5.4 （可选）安装 ros_gz_bridge

如果需要将 Gazebo 传感器数据（如相机、激光雷达）桥接到 ROS2 Topic：

```bash
# Garden 版本桥接包
sudo apt install -y ros-humble-ros-gzgarden
```

> `ros_gz_bridge` 可以将 Gazebo Transport 消息与 ROS2 消息双向转换。例如摄像头图像在 Gazebo 中是 `gz.msgs.Image`，桥接后变为 `sensor_msgs/Image`，可直接在 RVIZ2 中显示。

---

## 6. 安装 PX4 Autopilot

### 6.1 克隆源码（含全部子模块）

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

> :warning: **`--recursive` 是必须的。** PX4 依赖大量 git 子模块（mavlink、gazebo 模型、uXRCE-DDS 等），缺少子模块将无法编译。

若网络不稳定导致子模块下载失败，补全：

```bash
cd ~/PX4-Autopilot
git submodule update --init --recursive
```

> :bulb: 国内加速：
> ```bash
> git config --global url."https://ghproxy.com/".insteadOf "https://github.com/"
> ```

### 6.2 运行官方依赖安装脚本

```bash
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

该脚本会自动完成：
- 安装 PX4 编译所需的所有系统包
- 安装 ARM 交叉编译工具链（用于实机部署）
- 安装 Gazebo Garden 相关依赖
- 配置 udev 规则（飞控 USB 权限）

> 若仅需 SITL 仿真（不部署实机），可跳过部分内容：
> ```bash
> bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
> ```
> 但首次搭建建议不加参数全量安装，确保不丢包。

### 6.3 固定 empy 版本（关键！）

```bash
pip3 uninstall empy -y
pip3 install empy==3.3.4
pip3 install packaging==22.0
```

> :warning: **最高频踩坑点。** EmPy 4.x 改变了宏展开语法，与 PX4 的 `.em` 模板文件不兼容，会直接导致编译失败。必须锁死在 3.3.4。

验证：

```bash
empy --version
# 必须输出: 3.3.4
```

### 6.4 首次编译验证

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

**预期结果：**
1. 编译过程无错误（首次编译约 15-30 分钟，视机器性能）
2. Gazebo Garden 窗口自动打开，显示 X500 四旋翼模型
3. 终端出现 `pxh>` NuttShell 提示符
4. `pxh>` 下可输入命令（如 `commander status`）

### 6.5 可用仿真目标

| 命令 | 说明 |
|------|------|
| `make px4_sitl gz_x500` | 标准 X500 四旋翼 |
| `make px4_sitl gz_x500_depth` | X500 + 深度相机 |
| `make px4_sitl gz_x500_vision` | X500 + 视觉里程计 |
| `make px4_sitl gz_x500_mono_cam` | X500 + 单目相机 |
| `make px4_sitl gz_standard_vtol` | 标准 VTOL（垂直起降固定翼） |
| `make px4_sitl gz_rc_cessna` | 固定翼（Cessna 模型） |
| `HEADLESS=1 make px4_sitl gz_x500` | 无 GUI 模式（服务器/低配机） |

---

## 7. 安装 Micro XRCE-DDS Agent

### 7.1 Agent 与 Client 的关系

```
PX4 固件编译时              → 自动包含 uXRCE-DDS Client（src/modules/uxrce_dds_client/）
Agent（本步骤安装的独立进程） → 运行在 ROS2 侧，监听 UDP 8888，桥接至 DDS 网络
```

Agent 必须**先于 PX4 启动**，PX4 启动后 Client 会自动连接 Agent。

### 7.2 从源码编译安装

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### 7.3 验证安装

```bash
MicroXRCEAgent --help
```

应输出 Agent 的帮助信息。如果提示 `command not found`，检查 `/usr/local/bin/` 是否在 PATH 中：

```bash
echo $PATH | grep /usr/local/bin
# 若不在，手动添加:
echo 'export PATH=/usr/local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## 8. 搭建 PX4-ROS2 工作空间

PX4 官方提供了两个关键 ROS2 包：

| 包名 | 作用 |
|------|------|
| `px4_msgs` | PX4-ROS2 消息定义（`.msg` 文件），所有 Topic 的类型定义 |
| `px4_ros_com` | PX4-ROS2 通信示例代码，含 offboard 控制、传感器监听等 |

### 8.1 创建工作空间并克隆包

```bash
mkdir -p ~/ws_px4_ros2/src
cd ~/ws_px4_ros2/src

git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

### 8.2 安装 ROS 依赖并编译

```bash
cd ~/ws_px4_ros2
source /opt/ros/humble/setup.bash

# 安装工作空间中所有包的 ROS 依赖
rosdep install --from-paths src --ignore-src -y

# 编译
colcon build
```

### 8.3 配置环境变量

```bash
echo "source ~/ws_px4_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 8.4 验证编译结果

```bash
ros2 pkg list | grep px4
```

应输出：
```
px4_msgs
px4_ros_com
```

---

## 9. 安装 QGroundControl（可选）

```bash
# 移除 modemmanager（会干扰飞控串口）
sudo apt remove modemmanager -y

# 添加串口权限
sudo usermod -a -G dialout $USER

# 下载 QGC
cd ~/Downloads
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage

# 启动
./QGroundControl.AppImage
```

> **注意：** `usermod` 添加组权限后需**注销重新登录**才能生效。仿真场景中 QGC 通过 UDP 14550 端口自动连接 PX4 SITL。

---

## 10. 启动仿真

### 10.1 标准启动流程（3 个终端）

必须严格按照顺序启动：

#### :one: 终端 1：启动 Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

**预期输出：**
```
[1680000000.123456] info     | UDPAgent            | ... | Running at port 8888
```

Agent 将持续运行等待 PX4 Client 连接。

#### :two: 终端 2：启动 PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

启动后，Agent 终端应出现类似输出，表示 Client 已连接：
```
[1680000001.234567] info     | ProxyClient.cpp     | create_publisher    | publisher created
[1680000001.234568] info     | ProxyClient.cpp     | create_subscriber   | subscriber created
```

#### :three: 终端 3：ROS2 操作

```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4_ros2/install/setup.bash

# 列出所有 PX4 相关 Topic
ros2 topic list | grep /fmu/
```

**预期输出类似于：**
```
/fmu/out/sensor_combined
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
/fmu/out/vehicle_gps_position
/fmu/out/vehicle_attitude
/fmu/out/battery_status
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
...
```

### 10.2 可选终端：QGroundControl

```bash
cd ~/Downloads
./QGroundControl.AppImage
```

### 10.3 一键启动脚本

将以下脚本保存为 `~/start_simulation_xrce.sh`：

```bash
#!/bin/bash
# 一键启动 PX4 + Micro XRCE-DDS + Gazebo 仿真

set -e

echo "========================================="
echo "  PX4 + XRCE-DDS + Gazebo 仿真启动脚本"
echo "========================================="

# 检查必要的可执行文件
command -v MicroXRCEAgent >/dev/null 2>&1 || { echo "错误: MicroXRCEAgent 未安装"; exit 1; }

# 启动 Micro XRCE-DDS Agent
echo "[1/3] 启动 Micro XRCE-DDS Agent (UDP:8888)..."
gnome-terminal --tab --title="XRCE Agent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
sleep 2

# 启动 PX4 SITL + Gazebo
echo "[2/3] 启动 PX4 SITL + Gazebo Garden..."
gnome-terminal --tab --title="PX4 SITL" -- bash -c "cd ~/PX4-Autopilot && make px4_sitl gz_x500; exec bash"

echo "[3/3] 等待 PX4 完全启动..."
echo "      观察 Agent 终端出现 'publisher created' 即表示连接成功"
sleep 10

# 准备 ROS2 终端
gnome-terminal --tab --title="ROS2 Shell" -- bash -c "source /opt/ros/humble/setup.bash && source ~/ws_px4_ros2/install/setup.bash && echo '环境已就绪，输入 ros2 topic list 查看话题'; exec bash"

echo "========================================="
echo "  终端布局:"
echo "  1. XRCE Agent  (UDP 8888)"
echo "  2. PX4 SITL    (Gazebo Garden)"
echo "  3. ROS2 Shell  (已 source 工作空间)"
echo "========================================="
```

```bash
chmod +x ~/start_simulation_xrce.sh
```

---

## 11. 验证与测试

### 11.1 确认 Agent ↔ PX4 通信正常

在 Agent 终端中观察日志，应无错误信息，且出现 `publisher/subscriber created` 消息。

在 PX4 终端中检查 uXRCE-DDS 客户端状态：

```
pxh> uxrce_dds_client status
```

### 11.2 查看 ROS2 Topic

```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4_ros2/install/setup.bash

# 列出所有 fmu 话题
ros2 topic list | grep /fmu/
```

### 11.3 查看传感器数据

```bash
# 查看 IMU + 磁力计 + 气压计融合数据
ros2 topic echo /fmu/out/sensor_combined

# 查看 GPS 数据
ros2 topic echo /fmu/out/vehicle_gps_position

# 查看姿态
ros2 topic echo /fmu/out/vehicle_attitude

# 查看电池
ros2 topic echo /fmu/out/battery_status
```

### 11.4 查看 Topic 详细信息

```bash
# 查看某 Topic 的消息类型和 QoS
ros2 topic info /fmu/out/sensor_combined

# 查看消息定义
ros2 interface show px4_msgs/msg/SensorCombined

# 查看发布频率
ros2 topic hz /fmu/out/sensor_combined
```

### 11.5 确认 QGC 连接

QGC 启动后应自动连接仿真无人机：
- 左上角显示 "Ready to Fly"
- 地图上出现无人机位置图标
- 仪表盘显示姿态、高度、电量等数据

### 11.6 运行官方传感器监听示例

```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4_ros2/install/setup.bash

# 运行官方 SensorCombined 监听器
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

应输出持续的传感器数据打印。

---

## 12. Offboard 控制示例

### 12.1 运行官方 Offboard 控制

```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4_ros2/install/setup.bash

ros2 run px4_ros_com offboard_control
```

**预期行为：** 无人机自动解锁 → 原地旋转 90° → 上升至 5 米高度悬停。

### 12.2 自定义 Offboard 控制节点（Python 示例）

将以下代码保存为 `~/ws_px4_ros2/src/my_offboard/my_offboard/takeoff_and_land.py`：

```python
#!/usr/bin/env python3
"""
PX4 Offboard 控制示例 —— 基于 Micro XRCE-DDS
实现：解锁 → 起飞到 5m → 悬停 10s → 降落 → 上锁
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)

class TakeoffAndLand(Node):
    """基于 XRCE-DDS 的 PX4 Offboard 控制节点"""

    def __init__(self):
        super().__init__('takeoff_and_land')

        # --- QoS 配置 ---
        # PX4 要求 Best-Effort + Volatile 的 QoS（DDS 特性决定）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Publishers ---
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # --- Subscribers ---
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_cb, qos)

        # --- 内部状态 ---
        self.vehicle_status = None
        self.offboard_setpoint_counter = 0

        # --- 定时器：以固定频率发送控制指令 ---
        self.timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.start_time = self.get_clock().now()

    def vehicle_status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        """切换到 Offboard 模式"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True       # 使用位置控制
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=-5.0, yaw=0.0):
        """发布轨迹设定点（NED 坐标系）"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # NED 坐标系：z 为负表示向上
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """发送飞控命令（解锁/上锁/起飞等）"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        """解锁"""
        self.get_logger().info("Sending ARM command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("ARM command sent.")

    def disarm(self):
        """上锁"""
        self.get_logger().info("Sending DISARM command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("DISARM command sent.")

    def timer_callback(self):
        if self.vehicle_status is None:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # 阶段 1：发送 offboard 模式 + 悬停设定点（前 2 秒）
        if elapsed < 2.0:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(z=-1.0)  # 初始保持在 1m

        # 阶段 2：切换到 Offboard 模式并解锁
        elif elapsed < 4.0:
            self.publish_offboard_control_mode()
            # 请求切换到 Offboard 模式
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_trajectory_setpoint(z=-1.0)

        # 阶段 3：解锁
        elif elapsed < 6.0:
            self.publish_offboard_control_mode()
            if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
            self.publish_trajectory_setpoint(z=-1.0)

        # 阶段 4：起飞到 5m
        elif elapsed < 16.0:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(z=-5.0)

        # 阶段 5：降落
        elif elapsed < 26.0:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(z=-0.5)

        # 阶段 6：上锁
        elif elapsed < 30.0:
            self.publish_offboard_control_mode()
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.disarm()
            self.publish_trajectory_setpoint(z=-0.5)

        # 退出
        else:
            self.get_logger().info("Mission complete. Shutting down.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffAndLand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

配套的 `setup.py`（`~/ws_px4_ros2/src/my_offboard/setup.py`）：

```python
from setuptools import setup

package_name = 'my_offboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='PX4 Offboard Control Examples via XRCE-DDS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'takeoff_and_land = my_offboard.takeoff_and_land:main',
        ],
    },
)
```

配套的 `package.xml`（`~/ws_px4_ros2/src/my_offboard/package.xml`）：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_offboard</name>
  <version>0.1.0</version>
  <description>PX4 Offboard Control via XRCE-DDS</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>px4_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

创建 `resource` 目录并编译：

```bash
mkdir -p ~/ws_px4_ros2/src/my_offboard/resource
touch ~/ws_px4_ros2/src/my_offboard/resource/my_offboard

cd ~/ws_px4_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select my_offboard
source install/setup.bash

# 启动仿真后运行
ros2 run my_offboard takeoff_and_land
```

---

## 13. 常见问题与避坑

### 坑 1：em.py 编译错误

**错误信息：**
```
ModuleNotFoundError: No module named 'em'
ImportError: cannot import name 'Interpreter' from 'em'
```

**原因：** EmPy 4.x 与 PX4 模板语法不兼容。

**解决：**
```bash
pip3 uninstall empy -y
pip3 install empy==3.3.4
```

### 坑 2：colcon build 报 `canonicalize_version()` 错误

**错误信息：**
```
TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
```

**原因：** setuptools 版本与 packaging 不兼容。

**解决：**
```bash
pip3 install packaging==22.0
# 清除构建缓存后重新编译
cd ~/ws_px4_ros2 && rm -rf build install log && colcon build
```

### 坑 3：Agent 启动后 PX4 连不上

**现象：** Agent 终端一直显示 "Running at port 8888" 但无后续 "publisher created" 输出。

**排查：**
1. 确认 Agent 先于 PX4 启动
2. 检查防火墙：`sudo ufw status`（UDP 8888 不能被拦截）
3. 确认 PX4 编译时包含了 uXRCE-DDS Client：
   ```
   pxh> uxrce_dds_client status
   ```
4. 在 PX4 SITL 启动命令中显式指定：
   ```bash
   PX4_UXRCE_DDS_AGENT_IP=127.0.0.1 make px4_sitl gz_x500
   ```

### 坑 4：无法看到 `/fmu/out/*` Topic

**原因：** ROS2 环境变量未正确 source；或者 Agent 未成功连接。

**解决：**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4_ros2/install/setup.bash
ros2 topic list | grep /fmu/
```

如果仍然没有：
1. 检查 Agent 终端是否有 "publisher created" 日志
2. 检查 PX4 参数：`pxh> param show UXRCE_DDS_*`
3. 尝试重启 Agent 和 PX4（严格按启动顺序）

### 坑 5：Gazebo 模型不显示或 `ekf2 missing data`

**原因：** Gazebo 版本不匹配；模型缺少必要传感器。

**解决：**
```bash
# 确认 Gazebo 版本
gz sim --version

# 清除缓存
rm -rf ~/.gz/sim/cache/*
rm -rf ~/.gz/sim/log/*

# 确保用正确目标编译
cd ~/PX4-Autopilot
make distclean
make px4_sitl gz_x500
```

### 坑 6：QGC 无法连接仿真

**解决：**
- QGC 通过 UDP 14550 自动连接 SITL
- 断开物理飞控 USB 连接后再启动 QGC
- 检查端口占用：`ss -lnpu | grep 14550`
- 移除 modemmanager：`sudo apt remove modemmanager -y`

### 坑 7：`rosdep update` 超时

**解决：** 换镜像源：
```bash
export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/ros/rosdistro/index-v4.yaml
rosdep update
```

### 坑 8：git clone 子模块下载慢/失败

**解决：**
```bash
# 方案 A：使用代理
git config --global url."https://ghproxy.com/".insteadOf "https://github.com/"

# 方案 B：浅克隆子模块
git submodule update --init --recursive --depth 1
```

### 坑 9：Gazebo 启动黑屏或无 GUI

**解决：**
```bash
# 检查显卡驱动
glxinfo | grep "OpenGL renderer"

# 软件渲染（无 GPU / 虚拟机）
export LIBGL_ALWAYS_SOFTWARE=1

# 确认 DISPLAY 变量
echo $DISPLAY  # 应为 :0 或 :1
```

### 坑 10：仿真帧率低 / 实时因子 < 1

**解决：**
```bash
# 查看实时因子
pxh> commander status
# 关注 "realtime factor" 字段，< 1 表示仿真慢于实时

# 提升性能：
# 1. 降低 Gazebo 渲染质量
# 2. 使用 HEADLESS 模式：HEADLESS=1 make px4_sitl gz_x500
# 3. 关闭非必要 GUI 插件
# 4. 虚拟机中启用 3D 加速，分配更多 CPU 核心
```

---

## 14. 参考来源

### 官方文档（优先级最高）

| 资源 | 链接 | 说明 |
|------|------|------|
| PX4 ROS2 User Guide | https://docs.px4.io/main/en/ros2/user_guide | :star: **核心文档**，架构、安装、示例 |
| PX4 Ubuntu Dev Environment | https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html | PX4 官方 Ubuntu 开发环境搭建 |
| PX4 Gazebo Simulation (New) | https://docs.px4.io/main/en/sim_gazebo_gz/ | :star: 新 Gazebo (Garden/Harmonic) 仿真文档 |
| PX4 ROS2 Interface | https://docs.px4.io/main/en/ros2/px4_ros2_interface.html | ROS2 接口和控制架构 |
| Micro XRCE-DDS Agent | https://github.com/eProsima/Micro-XRCE-DDS-Agent | :star: Agent 官方仓库 |
| PX4 px4_msgs | https://github.com/PX4/px4_msgs | ROS2 消息定义包 |
| PX4 px4_ros_com | https://github.com/PX4/px4_ros_com | ROS2 通信示例包 |
| PX4-Autopilot GitHub | https://github.com/PX4/PX4-Autopilot | :star: 飞控固件源码 |
| ROS2 Humble 安装指南 | https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html | ROS2 官方安装指南 |
| Gazebo Garden 安装 | https://gazebosim.org/docs/garden/install_ubuntu | Gazebo 官方安装 |

### 国外博客/社区

| 资源 | 链接 | 说明 |
|------|------|------|
| ARK Electronics Offboard Example | https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example | 可工作的 Offboard 示例（Harmonic 适配） |
| VinayMatade/Simulation | https://github.com/VinayMatade/Simulation | 一键安装脚本（ROS2 Humble + PX4 + Gazebo） |
| melodylylin/px4-gz-docker | https://github.com/melodylylin/px4-gz-docker | Docker 完整环境（含 MAVROS） |
| ros_gz Issue #755 | https://github.com/gazebosim/ros_gz/issues/755 | Harmonic + Humble 包冲突讨论 |
| PX4 Discuss | https://discuss.px4.io/ | :star: 官方社区论坛 |
| PX4 Discord | https://discord.gg/dronecode | 实时讨论频道 |

### 国内博客/社区

| 资源 | 链接 | 说明 |
|------|------|------|
| 阿木实验室 — PX4×ROS2 保姆级教程 | https://amovlab.com/news/detail?id=329 | 中文详细教程 |
| CSDN 避坑指南 — Gazebo 版本冲突解决 | https://wenku.csdn.net/column/nt5g6wf9ns2 | PX4 v1.14 + Humble + Gazebo 完整流程 |
| CSDN PX4 + ROS2 + Gazebo 全栈搭建 | https://blog.csdn.net/swift5iosmith/article/details/149557400 | 2025 实测可用的中文教程 |
| CSDN 从零到一：PX4 仿真避坑 | https://blog.csdn.net/c2d3e4f/article/details/155583176 | 常见陷阱汇总 |
| WSL2 Ubuntu22.04 ROS2+PX4 环境配置 | https://www.cnblogs.com/soapen/articles/19186593 | WSL2 环境下的 PX4 配置 |

---

## 附录：环境验证清单

完成搭建后逐项检查：

- [ ] `python3 --version` → `Python 3.10.x`
- [ ] `ros2 run turtlesim turtlesim_node` → 小乌龟正常显示、可控制
- [ ] `gz sim --version` → 显示 Garden 版本号
- [ ] `empy --version` → `3.3.4`
- [ ] `cd ~/PX4-Autopilot && make px4_sitl gz_x500` → 编译成功，Gazebo 显示 X500 模型
- [ ] `pxh>` 提示符出现，可输入 `commander status` 查看状态
- [ ] `MicroXRCEAgent udp4 -p 8888` → Agent 正常启动
- [ ] Agent 终端在 PX4 启动后出现 `publisher created` 日志
- [ ] `ros2 topic list | grep /fmu/` → 显示 `/fmu/out/sensor_combined` 等话题
- [ ] `ros2 topic echo /fmu/out/sensor_combined` → 持续输出传感器数据
- [ ] `ros2 launch px4_ros_com sensor_combined_listener.launch.py` → 正常运行
- [ ] QGC 启动后自动连接仿真无人机
- [ ] `groups $USER | grep dialout` → 包含 `dialout`（实机部署需要）

全部 :white_check_mark: 即表示环境搭建成功！
