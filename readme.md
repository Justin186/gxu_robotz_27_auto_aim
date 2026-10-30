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
| `BUILD_DEBUG=ON` | rerun_sdk（**系统安装**，CMake 用 `find_package` 找，装法见下） |
| `BUILD_NAV=ON` | ROS2 humble + `sp_msgs`（构建方式见下） |
| `BUILD_AUTO_BUFF=ON` | [Ceres](http://ceres-solver.org/installation.html) |

> ⚠️ 目前 5 个预设**默认都开了 `BUILD_DEBUG`**，所以都需要先装好 rerun_sdk。

### rerun C++ SDK 安装（`BUILD_DEBUG=ON` 需要）

官方的接入方式是 `FetchContent` 直接拉 GitHub，校园网下不来，所以这里**手工装进系统**，
CMake 侧仍然只是一句 `find_package(rerun_sdk REQUIRED)`，不用改任何构建脚本。

当前用的版本：**rerun_cpp_sdk 0.41.0**，它强依赖 **Arrow 18.0.0** —— Arrow 的版本号是
SDK 里写死的，不是我们自己挑的，换成别的版本 `find_package` 过不去。

GitHub 直连不通，下面统一走 `gh-proxy.com` 代理（`ghfast.top` 也可以；`mirror.ghproxy.com`
实测不通）。先下两个包：

```bash
mkdir -p ~/sdk && cd ~/sdk

# SDK 本体（约 60MB）
curl -L -o rerun_cpp_sdk.zip \
  "https://gh-proxy.com/https://github.com/rerun-io/rerun/releases/download/0.41.0/rerun_cpp_sdk.zip"
unzip -q rerun_cpp_sdk.zip

# Arrow 源码（约 100MB；SDK 的构建脚本默认要去 GitHub 下来自己编，这里提前下好直接喂给它）
curl -L -o apache-arrow-18.0.0.tar.gz \
  "https://gh-proxy.com/https://github.com/apache/arrow/releases/download/apache-arrow-18.0.0/apache-arrow-18.0.0.tar.gz"
```

编译安装 SDK（`RERUN_ARROW_URL` 指本地包，绕开下载）：

```bash
cmake -S ~/sdk/rerun_cpp_sdk -B ~/sdk/build-rerun \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DRERUN_ARROW_URL=$HOME/sdk/apache-arrow-18.0.0.tar.gz
cmake --build ~/sdk/build-rerun -j$(nproc)   # 头一次会顺带编 Arrow，10~30 分钟
sudo cmake --install ~/sdk/build-rerun
```

装到 `/usr/local` 是因为那是 CMake 的默认搜索前缀，装完 `find_package(rerun_sdk)` 直接命中，
不用设 `CMAKE_PREFIX_PATH`、也不用往 `CMakePresets.json` 里写路径。

> 机器上已经有 libarrow 18.0.0 的话（比如之前编过），可以省掉 Arrow 那趟编译：
> 把上面 configure 那行换成
> `-DRERUN_DOWNLOAD_AND_BUILD_ARROW=OFF -DCMAKE_PREFIX_PATH=<libarrow 安装前缀>`。
> 这时 Arrow 不随 SDK 装，`/usr/local` 里也要有它的 cmake 配置，否则下游
> `find_package` 会卡在 `find_dependency(Arrow)`。

装完的效果（`find_package` 用到的就这几样）：

```
/usr/local/include/rerun.hpp
/usr/local/include/rerun/
/usr/local/lib/librerun_sdk.a
/usr/local/lib/librerun_c__linux_x64.a
/usr/local/lib/cmake/rerun_sdk/
```

Arrow 的部分看走哪条路：

| 方案 | Arrow 装在哪 | 下游要不要额外找 Arrow |
|---|---|---|
| SDK 顺带编（`RERUN_ARROW_URL` / 默认） | `/usr/local/lib/libarrow.a`、`libarrow_bundled_dependencies.a` | 不要，SDK 的 cmake 配置自己带上了 |
| 用系统已有的（`...=OFF`） | 还要 `include/arrow/`、`lib/cmake/Arrow/` | 要，配置里有一句 `find_dependency(Arrow)` |

验证（任意空目录，不带任何 `-D`）：

```bash
mkdir -p /tmp/fp && cd /tmp/fp
printf 'cmake_minimum_required(VERSION 3.16)\nproject(t CXX)\nfind_package(rerun_sdk REQUIRED)\n' > CMakeLists.txt
cmake -S . -B b     # 只要不报 "Could not find a package configuration file" 就算装好了
```

> viewer **不在车上装**。`BUILD_DEBUG` 的程序默认不连 viewer，是运行时用 `--rerun --ip`
> 连开发机上开着的那个，见"运行"一节。

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
| `sentry`（哨兵 42mm） | ✓ | ✓ | ✓ | | | ✓ | ✓ | ✓ | sentry |
| `sim`（仿真） | ✓ | | | | ✓ | ✓ | | | standard |
| `dev`（开发机） | ✓ | | | | | ✓ | ✓ | | standard |

> `sentry` 的 `BUILD_NAV` 目前是**关的**（开发机上没装 ROS2）。真要编 `sentry` 程序时，
> 把 `cmake/presets/sentry.cmake` 里那行改回 `ON`，并先 `source /opt/ros/humble/setup.bash`。

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

> 例外：下面两种情况会重置预设管理的开关（configure 时会打印提示），同一批 `-D`
> 也会被一起清掉，做法都是**重置之后再补 `-D`**：
>
> 1. 同一个 build 目录里**换了兵种**（`-DROBOT` 从 A 改成 B）；
> 2. **改了 `cmake/presets/<兵种>.cmake`**。
>
> 除此之外 `-D` 会一直粘在 `build/CMakeCache.txt` 里。第 2 条是专门加的：CMake 的
> `set(X ... CACHE ...)` 遇到已存在的 cache 项会静默跳过，不处理的话改预设永远不生效；
> 现在每次 configure 都比对预设文件的内容哈希，变了就把旧值清掉重新写。

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

### 调试可视化（rerun）

`BUILD_DEBUG=ON` 编出来的程序（`auto_aim_debug_mpc` / `auto_aim_debug_offline`）默认**不连 viewer**，
要用 `--rerun` 显式打开，`--ip` 填跑 viewer 那台机器：

```bash
./build/auto_aim_debug_mpc configs/infantry3.yaml --rerun --ip 192.168.1.18
```

程序内部走 `rerun+http://<ip>:9876/proxy`，所以开发机上的 viewer 要在这个端口上监听
（`rerun` 起来默认就是 9876）。viewer 只在开发机装：`pip install rerun-sdk`，或从
[rerun.io](https://rerun.io) 下安装包。

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
