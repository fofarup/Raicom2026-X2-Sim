# Raicom2026 X2 比赛地图仿真

本仓库用于在 Ubuntu 22.04（x86_64）上，通过 Docker 从零搭建睿抗 2026 智慧养老赛项的 X2 仿真环境，利用 NVIDIA GPU 加速 MuJoCo 渲染。

**第一阶段验收目标**：

> 加载官方比赛地图与 X2 模型 → 稳定站立 → 低速前进 → 安全停止

---

## 目录结构

```text
~/x2_ws/x2_biao/
├── config/project.env              # 项目配置（ROS_DOMAIN_ID、机器人名等）
├── control/safe_forward.py         # 安全的低速前进控制脚本
├── docker/Dockerfile               # Ubuntu 22.04 + ROS 2 Humble 镜像
├── docs/
│   ├── 01-从零安装.md              # 从零开始的环境安装教程
│   ├── 02-首次启动与控制.md        # 首次启动仿真和控制
│   ├── 03-tmux使用.md              # tmux 快捷键参考
│   └── 04-故障排查.md              # 常见故障及解决方案
├── scripts/
│   ├── bootstrap_assets.sh         # 解压官方压缩包，生成 .runtime/
│   ├── build_image.sh              # 构建 Docker 镜像
│   ├── check_host.sh               # 宿主机环境检查
│   ├── start_container.sh          # 启动 Docker 容器
│   ├── enter_container.sh          # 进入容器终端
│   ├── stop_all.sh                 # 停止仿真并关闭容器
│   ├── status.sh                   # 查看运行状态
│   ├── in_container/               # 容器内脚本
│   │   ├── build_aimdk.sh          # 编译 aimdk_msgs
│   │   ├── start_sim.sh            # 启动 MuJoCo 仿真
│   │   ├── start_mc.sh             # 启动运动控制模块
│   │   ├── set_stand.sh            # 设置站立模式
│   │   ├── set_locomotion.sh       # 设置运动模式
│   │   └── run_safe_demo.sh        # 运行安全前进演示
│   └── tmux/
│       ├── start.sh                # tmux 多窗口启动
│       ├── start_split.sh          # tmux 单窗口三分屏启动（推荐）
│       └── stop.sh                 # 停止 tmux 会话
└── vendor/
    └── link_u_os_competition-main.tar.gz  # 官方压缩包（Git LFS，~512MB）
```

`.runtime/`（运行后生成，已加入 `.gitignore`）：

```text
.runtime/
├── archive.sha256                  # 压缩包 SHA256 校验
├── raicom2026 -> official/...      # 符号链接 → 官方资源
├── official/                       # 官方压缩包解压内容
│   └── link_u_os_competition-main/Raicom2026/
│       ├── sim_mujoco/             # MuJoCo 仿真程序
│       ├── mc/                     # 运动控制模块
│       ├── example/py/             # 官方 Python 示例
│       └── aimdk-aarch64-.../      # 消息定义包
└── ros/                            # ROS 2 编译产物
    ├── build/
    ├── install/
    └── log/
```

---

## 环境要求

| 项目 | 要求 | 备注 |
|------|------|------|
| 操作系统 | Ubuntu 22.04 LTS x86_64 | |
| 桌面会话 | X11（Ubuntu on Xorg） | 注销后在登录界面齿轮选择 |
| Docker | ≥ 24.0 | 需安装 |
| Git LFS | ≥ 3.0 | 用于下载 512MB 压缩包 |
| tmux | ≥ 3.0 | 终端多路复用 |
| GPU | NVIDIA 显卡（推荐） | 若无 GPU 则用 CPU 软渲染，会非常卡 |
| NVIDIA 驱动 | ≥ 525 | |
| nvidia-container-toolkit | ≥ 1.14 | Docker 调用 GPU 所需 |
| 网络代理 | 需要 | Docker Hub 在中国大陆需代理访问 |

---

## 从零安装

### 1. 安装系统依赖

```bash
# Git LFS + tmux + Docker（详细步骤见 docs/01-从零安装.md）
sudo apt update
sudo apt install -y git git-lfs tmux
git lfs install
```

### 2. 安装 Docker

参考 [Docker 官方文档](https://docs.docker.com/engine/install/ubuntu/) 或运行：

```bash
bash scripts/install_host_tools.sh
```

> 安装后将用户加入 docker 组：`sudo usermod -aG docker $USER`，然后**注销重新登录**。

### 3. 安装 NVIDIA Container Toolkit（GPU 必需）

```bash
# 添加 NVIDIA 仓库
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 4. 克隆仓库

```bash
mkdir -p ~/x2_ws
cd ~/x2_ws
git clone git@github.com:fofarup/Raicom2026-X2-Sim.git x2_biao
cd x2_biao
git lfs pull
```

### 5. 构建并初始化

```bash
cd ~/x2_ws/x2_biao

# 环境检查
bash scripts/check_host.sh

# 解压官方资源
bash scripts/bootstrap_assets.sh

# 构建 Docker 镜像（约 15-30 分钟）
bash scripts/build_image.sh

# 编译 aimdk_msgs
bash scripts/start_container.sh
docker exec raicom2026-x2-sim bash -lc '/workspace/scripts/in_container/build_aimdk.sh'
```

---

## 使用方式

### 一键启动（推荐：三分屏布局）

```bash
cd ~/x2_ws/x2_biao
bash scripts/tmux/start_split.sh
```

窗口布局：

```
┌──────────────────────────┬──────────────┐
│                          │              │
│   MuJoCo 仿真（图形窗口） │  MC 控制台   │
│                          │              │
│                          ├──────────────┤
│                          │  控制终端    │
│                          │  (输入命令)  │
└──────────────────────────┴──────────────┘
```

### 让机器人站立

在控制终端（右下窗格）中：

```bash
cd /workspace/.runtime/raicom2026/example/py
python3 set_mode.py JD    # 关节位置控制
python3 set_mode.py SD    # 稳定站立
```

然后去 **MuJoCo 图形窗口** 点击右上角的 **Reset** 按钮。机器人会站起并自动保持平衡。

### 让机器人前进

```bash
python3 set_mode.py LD    # 运动模式
cd /workspace/control
python3 safe_forward.py --speed 0.12 --duration 2.0
```

### 停止

```bash
# 按 Ctrl+B 然后 D 退出 tmux
# 然后在宿主机：
cd ~/x2_ws/x2_biao
bash scripts/tmux/stop.sh
```

---

## safe_forward.py 参数

| 参数 | 默认值 | 上限 | 说明 |
|------|--------|------|------|
| `--speed` | 0.10 | 0.20 m/s | 前进速度 |
| `--duration` | 1.0 | 3.0 s | 前进时长 |
| `--stop-only` | — | — | 只发送停止指令 |

正常结束、异常中断或 `Ctrl+C` 都会自动发送零速度制动。

---

## 机器人模式说明

| 模式 | 缩写 | 说明 |
|------|------|------|
| PASSIVE_DEFAULT | PD | 关节无力矩（软趴趴） |
| DAMPING_DEFAULT | DD | 阻尼模式 |
| JOINT_DEFAULT | JD | 关节位置控制（锁住关节） |
| STAND_DEFAULT | SD | 稳定站立（自动平衡）|
| LOCOMOTION_DEFAULT | LD | 运动模式（可行走） |
| UPPERBODY_REMOTE_SPLIT | US | 上半身远程控制 |
| HEAD_ONLY | HO | 仅头部控制 |

---

## 配置参数

见 `config/project.env`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ROS_DOMAIN_ID` | 26 | ROS 2 域 ID |
| `RMW_IMPLEMENTATION` | rmw_cyclonedds_cpp | ROS 2 通信中间件 |
| `DOCKER_IMAGE` | raicom2026-x2-sim:humble | Docker 镜像名 |
| `DOCKER_CONTAINER` | raicom2026-x2-sim | 容器名 |
| `TMUX_SESSION` | raicom-x2 | tmux 会话名 |
| `ROBOT_NAME` | lx2501_3_t2d5 | 机器人型号 |
| `USE_NVIDIA_GPU` | 1 | 是否启用 GPU |

---

## 常见踩坑记录

### 1. MuJoCo 窗口极卡

**原因**：OpenGL 渲染器是 `llvmpipe`（纯 CPU 软渲染）。

**检查**：

```bash
docker exec raicom2026-x2-sim glxinfo | grep "OpenGL renderer"
# 应显示：NVIDIA GeForce RTX 4060 ...
# 若显示：llvmpipe → GPU 未正确配置
```

**解决**：安装 `nvidia-container-toolkit`，确保容器启动时带 `--gpus all`。

### 2. 仿真启动后崩溃：`Get executor [MujocoSimModule/hds] failed`

**原因**：HDS 组件缺少对应的执行器配置。

**解决**：启动仿真前设置环境变量 `AGIBOT_ENABLE_HDS_COMPONENT=0`。已在 `scripts/in_container/start_sim.sh` 中内置。

### 3. Docker 拉取镜像超时

**原因**：Docker Hub 在中国大陆无法直连。

**解决**：
- 配置 HTTP 代理（在 `/etc/systemd/system/docker.service.d/http-proxy.conf` 中设置 `HTTP_PROXY`）
- 或用 Docker 镜像加速器（`/etc/docker/daemon.json` 中配置 `registry-mirrors`）
- 注意：镜像加速器和代理同时使用时会冲突，建议只用代理

### 4. docker 权限报错

**原因**：用户不在 `docker` 组中。

**解决**：`sudo usermod -aG docker $USER`，然后**注销 Ubuntu 重新登录**。

### 5. ROS 2 setup.bash 报 `unbound variable`

**原因**：脚本中 `set -u` 与 ROS 2 setup.bash 不兼容。

**解决**：本项目脚本已统一改为 `set -eo pipefail`（去掉 `-u`），并在 `common.sh` 中显式添加 `set +u`。

### 6. X11 连接失败

**原因**：容器无权访问 X Server。

**解决**：启动容器前执行 `xhost +local:docker` 或 `xhost +`。

### 7. MuJoCo 窗口不显示地图/机器人

**原因**：仿真进程崩溃（通常是上述 HDS 或 X11 问题）。

**排查**：
```bash
docker exec raicom2026-x2-sim ls /workspace/.runtime/raicom2026/sim_mujoco/bin/log/crash/
docker exec raicom2026-x2-sim cat /workspace/.runtime/raicom2026/sim_mujoco/bin/log/crash/*.crash
```

---

## 重要说明

- 官方例程保存在压缩包中，不做修改。
- 本仓库新增的 `control/safe_forward.py` 只用于仿真测试。
- **未经重新安全评估，不要把该测试脚本用于真机。**
- 官方资源许可尚未确认，因此仓库必须保持私有，详见 [NOTICE.md](NOTICE.md)。
