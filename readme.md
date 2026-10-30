# 广西大学 ROBOT-Z 战队 27 赛季自瞄算法

27 赛季的算法仓库，以 26 赛季为基线重写了构建系统与通信层。
26 赛季基于同济 SuperPower 战队的 [sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) 开发。

## 硬件环境

| 项 | 型号 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| 运算平台 | NUC12WSKI7（i7-1260P / 16GB）；狗洞步兵为 Jetson Nano（aarch64） |
| 相机 | 海康 MV-CS016-10UC + 官方 6mm 镜头；哨兵另有后置 USB 相机 |
| 下位机 | RoboMaster 开发板 C 型（STM32F407） |
| 通信 | MicroUSB 虚拟串口（云台协议；哨兵同一个串口上还有导航帧） |

## 依赖

**必需**

- [HikRobot SDK](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_231116.zip)
- [OpenVINO](https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-archive-linux.html)（apt 装即可，CMake 走标准搜索；特殊路径用 `-DOpenVINO_DIR=`）
- 系统包：

```bash
sudo apt install -y git g++ cmake libopencv-dev libfmt-dev libeigen3-dev \
    libspdlog-dev libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev \
    openssh-server screen
```

**按需**

| 开关 | 额外依赖 |
|---|---|
| `BUILD_DEBUG=ON` | rerun_sdk（**系统安装**，CMake 用 `find_package` 找；不要用 FetchContent，校园网连不上 GitHub） |
| `BUILD_NAV=ON` | ROS2 humble + `sp_msgs`（构建方式见下） |
| `BUILD_AUTO_BUFF=ON` | [Ceres](http://ceres-solver.org/installation.html) |

> ⚠️ 目前 5 个预设**默认都开了 `BUILD_DEBUG`**，所以都需要先装好 rerun_sdk。

## 编译

### 兵种预设

```bash
cmake --preset infantry        # 或 hero / sentry / sim / dev
cmake --build --preset infantry -j$(nproc)
```

| 预设 | 自瞄 | 打符 | 全向感知 | ROS2 | 仿真 | rerun | 测试 | 标定 | 云台协议 |
|---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|---|
| `hero`（重装 22mm） | ✓ | | | | | ✓ | ✓ | ✓ | standard |
| `infantry`（步兵 42mm，轮腿/狗洞共用） | ✓ | | | | | ✓ | ✓ | ✓ | standard |
| `sentry`（哨兵 42mm） | ✓ | ✓ | ✓ | ✓ | | ✓ | ✓ | ✓ | sentry |
| `sim`（仿真） | ✓ | | | | ✓ | ✓ | | | standard |
| `dev`（开发机） | ✓ | | | | | ✓ | ✓ | | standard |

两个步兵的差异只在 `configs/infantry3.yaml` / `configs/infantry4.yaml`（按车号），
构建配置完全一致。

新增兵种：往 `cmake/presets/` 丢一个 `<名字>.cmake` 即可，顶层会自动发现，`-DROBOT=<名字>` 立刻可用。

也可以用 `-DROBOT=xxx` 手动指定（等价）：

```bash
cmake -B build -DROBOT=sentry
cmake --build build -j$(nproc)
```

### 功能开关

命令行 `-D<开关>=ON/OFF` 优先于预设。

> 例外：同一条命令里**又换了兵种**时，切换过程会重置预设管理的开关（configure 时
> 会打印提示），`-D` 会被一起清掉。做法是**先切兵种，再补 `-D`**。

| 开关 | 默认 | 作用 |
|---|:-:|---|
| `BUILD_AUTO_AIM` | ON | 自瞄核心（需要 OpenVINO） |
| `BUILD_AUTO_BUFF` | ON | 打符（需要 OpenVINO + Ceres） |
| `BUILD_OMNIPERCEPTION` | OFF | 全向感知（哨兵） |
| `BUILD_NAV` | OFF | ROS2 通信（哨兵导航 / ROS2 仿真桥） |
| `BUILD_SIM` | OFF | 共享内存 + 仿真 IMU |
| `BUILD_DEBUG` | OFF | rerun 可视化（需要 rerun_sdk） |
| `BUILD_TESTS` | OFF | 测试与调试工具 |
| `BUILD_CALIBRATION` | ON | 标定程序 |

重依赖是按需查找的：不用打符就不找 Ceres，不开 ROS2 就不找 rclcpp，不开 rerun 就不找 rerun_sdk。

### 云台串口协议（编译期）

| 开关 | 取值 | 说明 |
|---|---|---|
| `GIMBAL_PROTOCOL` | `standard` / `sentry` | 标准协议 / 标准协议 + 导航帧通道 |
| `GIMBAL_HAS_OFFSET` | `ON` / `OFF` | 反馈帧带不带 yaw/pitch 稳态补偿偏置 |

`GIMBAL_HAS_OFFSET` 会改变**上行帧长度**：带偏置 45 字节，不带 43 字节。
**电控没实现那两个字节时必须关掉，否则整帧错位**（症状：收不到帧、或 CRC 一直校验失败）。

```bash
cmake -B build -DROBOT=infantry -DGIMBAL_HAS_OFFSET=OFF
```

### 哨兵：额外构建 sp_msgs

`sp_msgs` 是本仓库的 ROS2 消息源码包，**生成物不入库**，第一次用（或改了 `.msg`）需要先构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select sp_msgs --base-paths sp_msgs \
             --install-base sp_msgs_install --build-base sp_msgs_build
```

没构建的话 CMake 会直接失败并打印上面这条命令。

## 运行

所有程序的第一个位置参数都是配置路径，不传则用源码里写的默认值：

```bash
./build/<程序> configs/<兵种>.yaml
```

例如 `./build/sentry configs/sentry.yaml`。

> 仓库里同时留着 26 赛季的多套入口，具体哪个程序对应哪个兵种以实际在用的为准：
>
> - 走 `io::Gimbal`（串口）的：`*_mpc` 系列、`sentry` —— 27 赛季的方向
> - 走 `io::CBoard`（CAN）的：`standard` / `mt_standard` / `uav*` / `auto_buff_debug` —— 同济开源遗留，27 赛季不用

### 哨兵（`sentry`）的结构

```
主线程（相机帧率）      主海康 → yolo → tracker → 目标队列
Perceptron（内部线程）  后置 USB → yolo → 检测队列
控制线程（100Hz）       规划云台指令 → gimbal.send
导航线程（10Hz）        ROS2 cmd_vel / 姿态 ⇄ 串口 nav 帧
```

- 主相机丢目标后先原地保持 25 帧，再用后置相机找；仍找不到就匀速扫掠
- 导航超过 300ms 没给新指令就发零停车
- 扫描/选靶都在控制线程里，`io::Gimbal` 只负责收发，不掺决策

## 配置说明

每个兵种一份 `configs/<兵种>.yaml`，共用参数不多所以没有 common 层。

与**电控约定**相关的几项。`com_port` 必填；另外两个是**可选键，需要时才自己加进
yaml**，不写就用默认值（`little` / `1`）：

```yaml
com_port: "/dev/ttyACM0"   # 必填：云台串口设备
crc_byte_order: little     # 可选：帧尾 CRC 两字节的字节序，little / big
pitch_sign: 1              # 可选：电控把 pitch 正负搞反了就写 -1（收发两个方向都会翻）
```

- **CRC 字节序配错**时 `io::Gimbal` 会自动用另一种顺序再试一次，命中就打 warn 告诉你该改成什么，不用猜也不用重编译
- **pitch 符号**在 `io::Gimbal` 内部统一翻译，业务代码一律按"抬头为正"写，不要手动加负号

启动时会把协商结果打出来，配错了先看这一行：

```
[Gimbal] 协议=sentry, crc_byte_order=little, pitch_sign=1, 偏置=带, 导航通道=有
```

## 当前状态

- 哨兵主程序（`src/sentry.cpp`）已按 27 方案重写完成，但**目前只验证了编译，尚未上机**
- 哨兵的 nav 帧（`NavToGimbal` / `GimbalToNav`）沿用 26 赛季的布局，
  **27 电控是重写的，这两帧的字段和帧头要和电控重新对一遍**，否则 nav 通道一帧都不通
- 调试可视化版本等主程序上机跑通后再加
- `src/standard.cpp`、`src/mt_standard.cpp`、`src/uav*.cpp`、`src/auto_buff_debug.cpp` 等
  仍走 `io::CBoard`（CAN），是同济开源遗留、27 赛季不用的程序
- `pitch_sign` 各兵种的实际取值需要上机确认后再写进对应 yaml

## 其它

<details>
<summary>注册自启</summary>

```bash
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/sp_vision.desktop <<'EOF'
[Desktop Entry]
Type=Application
Exec=/home/rm/Desktop/gxu_robotz_27_auto_aim/autostart.sh configs/infantry3.yaml
Name=sp_vision
EOF
chmod +x autostart.sh
```

`Exec` 必须是绝对路径。**配置路径要作为参数传进来** —— "这台机器跑哪份配置"写在
各自的桌面项里，仓库中的 `autostart.sh` 谁都不用改，也就不会 pull 冲突。
</details>

<details>
<summary>串口 udev 固定设备名</summary>

1. 加入 dialout 组：`sudo usermod -a -G dialout $USER`
2. 查设备的 ID：`udevadm info -a -n /dev/ttyACM0 | grep -E '({serial}|{idVendor}|{idProduct})'`
3. 写规则 `/etc/udev/rules.d/99-usb-serial.rules`：

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="1234", ATTRS{serial}=="A1234567", SYMLINK+="gimbal"
```

4. 生效：`sudo udevadm control --reload-rules && sudo udevadm trigger`
5. 检查：`ls -l /dev/gimbal`
</details>

<details>
<summary>OpenVINO GPU 推理（可选）</summary>

```bash
mkdir neo && cd neo
wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.13463.18/intel-igc-core_1.0.13463.18_amd64.deb
wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.13463.18/intel-igc-opencl_1.0.13463.18_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-level-zero-gpu-dbgsym_1.3.25812.14_amd64.ddeb
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-level-zero-gpu_1.3.25812.14_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-opencl-icd-dbgsym_23.09.25812.14_amd64.ddeb
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/intel-opencl-icd_23.09.25812.14_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/libigdgmm12_22.3.0_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/23.09.25812.14/ww09.sum
sha256sum -c ww09.sum
sudo dpkg -i *.deb
```

用 GPU 异步推理时，最高显示分辨率限制 1920×1080 (24Hz)。
</details>
