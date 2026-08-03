# Raicom2026 X2 智慧养老国赛项目

> 睿抗机器人开发者大赛（RAICOM 2026）CAIM 工创赛道 · AGI 具身智能服务机器人赛题 · 智慧养老组 · 国赛项目

本仓库包含从零搭建 X2 人形机器人仿真环境到完成国赛全部三个任务的完整代码、工具、测试和文档。

---

## 目录

- [一、项目概览](#一项目概览)
- [二、目录架构](#二目录架构)
- [三、环境搭建](#三环境搭建)
- [四、使用步骤](#四使用步骤)
- [五、代码模块详解](#五代码模块详解)
- [六、国赛任务流程](#六国赛任务流程)
- [七、评分合规矩阵](#七评分合规矩阵)
- [八、测试套件](#八测试套件)
- [九、配置参数](#九配置参数)
- [十、常见问题](#十常见问题)

---

## 一、项目概览

### 赛题简介

围绕"智慧养老陪护"场景，以智元灵犀 X2 人形机器人为本体，重点考察：

- **自主导航**：出发区 → 交互区-I → 作业区，激光建图 + A* 规划
- **基础交互**：时间问答、数字颜色识别、表情切换、手势动作执行
- **场景服务**：语音理解需求 → 导航至作业区 → 视觉定位 → 机械臂抓取物品

### 国赛规则要点

| 项目 | 规则 |
|------|------|
| 限时 | 20 分钟，最多 2 次运行 |
| 总分 | 基础 100 分 + 附加 50 分（自主建图导航） |
| 任务数 | 3 个，顺序连续执行，一次指令全自主 |
| 语音 | 禁止使用机器人自带大模型，必须自行接入 |
| 控制 | 禁止遥控、键盘、手柄、中途 Reset |

### 技术栈

```
Python 3.10 + ROS 2 Humble + MuJoCo + Docker + Pinocchio IK + OpenCV/Pillow
```

---

## 二、目录架构

```text
x2_biao/
│
├── config/project.env              # 项目配置（ROS_DOMAIN_ID、机器人型号等）
│
├── docker/Dockerfile               # Ubuntu 22.04 + ROS 2 Humble 镜像定义
│
├── scripts/                        # 仿真环境管理脚本
│   ├── bootstrap_assets.sh         # 解压官方 512MB 压缩包，生成 .runtime/
│   ├── build_image.sh              # 构建 Docker 镜像（约 15-30 分钟）
│   ├── check_host.sh               # 宿主机环境检查（13 项）
│   ├── start_container.sh          # 启动 Docker 容器（含 GPU 支持）
│   ├── enter_container.sh          # 进入容器终端
│   ├── stop_all.sh                 # 停止仿真 + 关闭容器
│   ├── status.sh                   # 查看运行状态
│   ├── install_host_tools.sh       # 安装宿主机依赖
│   ├── in_container/               # 容器内脚本
│   │   ├── common.sh               # 环境变量（ROS2、aimdk_msgs 等）
│   │   ├── build_aimdk.sh          # 编译 aimdk_msgs 消息包
│   │   ├── start_sim.sh            # 启动 MuJoCo 仿真（支持竞赛模型和激光插件）
│   │   ├── start_mc.sh             # 启动运动控制模块
│   │   ├── set_stand.sh            # 设置站立模式（SD）
│   │   ├── set_locomotion.sh       # 设置运动模式（LD）
│   │   └── run_safe_demo.sh        # 运行安全前进演示
│   ├── lib/common.sh               # 公共函数（require_* 等）
│   └── tmux/                       # tmux 终端管理
│       ├── start.sh                # 多窗口启动
│       ├── start_split.sh          # 三分屏启动（MuJoCo / MC / 控制台）
│       └── stop.sh                 # 停止 tmux 会话
│
├── control/                        # 应用层代码
│   ├── omnipicker_hand.py          # ★ OmniPicker 双夹爪控制（完整版）
│   ├── omnipicker_hand_student.py  # OmniPicker 双夹爪控制（学生模板）
│   ├── OmniPicker双夹爪控制编程任务说明.md  # 夹爪编程说明文档
│   ├── safe_forward.py             # 安全低速前进测试
│   ├── stand.sh / ready.sh / go.sh # 手动便捷控制脚本
│   │
│   └── raicom2026/                 # ★★★ 国赛任务代码 ★★★
│       ├── competition_node.py     # 🏆 国赛统一入口（主控节点）
│       ├── run_all.sh              # 一键运行三个任务
│       │
│       ├── core/                   # 核心模块库
│       │   ├── mode_switch.py      # 模式切换（JD/SD/LD/US/PD/DD）
│       │   ├── locomotion.py       # 速度控制 + MC 输入源注册 + 航向修正
│       │   ├── navigator.py        # TF/激光定位 + A* 路径规划 + 泊车
│       │   ├── mapping.py          # 空白激光建图 + 碰撞监测
│       │   ├── speech.py           # 语音抽象层（仿真键盘 / 真机 ASR-TTS）
│       │   ├── vision.py           # 数字颜色识别（模板匹配 + 15 色色板）
│       │   ├── grasp.py            # IK 求解 + 手臂轨迹 + 夹爪抓取流程
│       │   ├── gesture.py          # 五种抽签手势的双臂关节轨迹
│       │   ├── expression.py       # 面部表情控制（7 种表情）
│       │   └── scenario.py         # 状态机 + 需求解析 + 输入校验
│       │
│       ├── ik_sdk/                 # X2 逆运动学 SDK（Pinocchio）
│       │   ├── x2_ik_sdk/
│       │   │   ├── solver.py       # IK 求解器（阻尼最小二乘）
│       │   │   ├── config.py       # 末端帧、关节序、默认姿态
│       │   │   └── resources/      # X2 OmniPicker URDF 运动学模型
│       │   └── offline_demo.py     # 离线 IK 演示
│       │
│       ├── map_tf/                 # 地图与 TF 定位包（比赛组委会提供）
│       │   └── map_tf_distribution/
│       │       ├── map_tf_distribution/
│       │       │   ├── map_publisher_node.py   # 发布静态 OccupancyGrid
│       │       │   └── localization_tf_node.py # 读取 TF 发布 PoseStamped
│       │       ├── maps/           # 多张比赛地图（PNG + YAML）
│       │       ├── launch/         # ROS 2 launch 文件
│       │       └── rviz/           # RViz 可视化配置
│       │
│       ├── resources/numbers/      # 63 张彩色数字图片（官方素材）
│       │
│       └── tests/                  # 自动化测试套件
│           ├── test_vision.py      # 数字识别正确率测试
│           ├── test_grasp_math.py  # 坐标转换 + 关节拼接数学测试
│           ├── test_scenario.py    # 需求解析 + 状态机测试
│           └── test_mapping.py     # 激光建图 + A* 规划测试
│
├── sim/                            # 仿真调试与校准工具
│   ├── prepare_competition_assets.py   # 生成竞赛场景（夹爪、道具、传感器）
│   ├── arm_state_relay.py              # 手臂状态中继（真机→仿真桥接）
│   ├── x11_window_tool.c               # MuJoCo GUI 自动 Reset（C 程序）
│   ├── raycaster_lidar_fixed.cc        # 修复版激光射线插件
│   ├── build_lidar_plugin.sh           # 编译激光插件
│   ├── validate_competition_model.py   # 竞赛模型验证
│   ├── check_*.py                      # 单项功能检查脚本（约 20 个）
│   ├── calibrate_*.py                  # 参数标定脚本
│   └── run_*_acceptance.py             # 导航/抓取验收测试
│
├── docs/                           # 文档
│   ├── 01-从零安装.md              # 环境安装详细教程
│   ├── 02-首次启动与控制.md        # 仿真首次启动指南
│   ├── 03-tmux使用.md              # tmux 快捷键参考
│   ├── 04-故障排查.md              # 常见问题及解决方案
│   └── 国赛合规矩阵.md              # 评分逐项验收标准
│
├── vendor/                         # 官方资源
│   └── link_u_os_competition-main.tar.gz  # 512MB（Git LFS）
│
├── .runtime/                       # 运行时生成（.gitignore 排除）
│   ├── official/                   # 官方资源解压
│   ├── raicom2026 → official/...   # 符号链接
│   └── ros/                        # colcon 编译产物
│
├── NOTICE.md                       # 许可说明
└── .gitignore                      # Git 排除规则
```

---

## 三、环境搭建

### 3.1 硬件和系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 LTS x86_64 |
| 桌面 | X11（Ubuntu on Xorg） |
| GPU | NVIDIA 显卡（推荐，否则 CPU 软渲染极卡） |
| 驱动 | NVIDIA ≥ 525 |
| 内存 | ≥ 16 GB |
| 磁盘 | ≥ 20 GB 可用 |
| 网络 | 需要能访问 GitHub + Docker Hub（或配置代理） |

### 3.2 从零安装

```bash
# 1. 安装系统依赖
sudo apt update && sudo apt install -y git git-lfs tmux ca-certificates curl x11-xserver-utils
git lfs install

# 2. 安装 Docker（详见 docs/01-从零安装.md，或用项目脚本）
cd ~/x2_ws/x2_biao
bash scripts/install_host_tools.sh

# 3. 注销 Ubuntu 并重新登录（docker 组生效）

# 4. 安装 NVIDIA Container Toolkit（GPU 用户）
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
# … 完整步骤见 docs/01-从零安装.md
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 5. 克隆仓库
mkdir -p ~/x2_ws && cd ~/x2_ws
git clone git@github.com:fofarup/Raicom2026-X2-Sim.git x2_biao
cd x2_biao && git lfs pull

# 6. 环境检查
bash scripts/check_host.sh    # 13 项全部通过才能继续
```

### 3.3 构建与初始化

```bash
cd ~/x2_ws/x2_biao

# 解压官方资源（512MB → ~1.8GB）
bash scripts/bootstrap_assets.sh

# 构建 Docker 镜像（约 15-30 分钟）
bash scripts/build_image.sh

# 编译 aimdk_msgs
bash scripts/start_container.sh
docker exec raicom2026-x2-sim bash -lc \
  '/workspace/scripts/in_container/build_aimdk.sh'
```

---

## 四、使用步骤

### 4.1 启动仿真环境

```bash
cd ~/x2_ws/x2_biao

# 方式一：tmux 三分屏（推荐）
bash scripts/tmux/start_split.sh

# 方式二：手动多终端
# 终端1: docker exec -it raicom2026-x2-sim bash -lc '/workspace/scripts/in_container/start_sim.sh'
# 终端2: docker exec -it raicom2026-x2-sim bash -lc '/workspace/scripts/in_container/start_mc.sh'
# 终端3: docker exec -it raicom2026-x2-sim bash -l
```

### 4.2 准备阶段（赛前）

容器内执行：

```bash
cd /workspace/control/raicom2026

# 一键全流程（仿真模式，键盘交互）
python3 competition_node.py --sim

# 或指定自动参数
python3 competition_node.py --sim \
  --auto-prepare \
  --number-image number_01.png \
  --expression 快乐 \
  --gesture 挥右手 \
  --need "我有点口渴" \
  --hand right
```

### 4.3 参数说明

| 参数 | 取值 | 默认值 | 说明 |
|------|------|--------|------|
| `--sim` | flag | True | 仿真模式（真机不传） |
| `--auto-prepare` | flag | False | 自动 Reset MuJoCo（仿真测试用） |
| `--auto-start` | flag | False | 自动开始（仿真测试用，跳过裁判口令等待） |
| `--number-image` | 文件名 | number_01.png | 数字图片名（从 resources/numbers/ 中选） |
| `--expression` | 悲伤/睡觉/愤怒/快乐/充电 | — | 抽中表情（跳过键盘输入） |
| `--gesture` | 挥左手/挥右手/左手敬礼/右手敬礼/双手打叉 | — | 抽中动作 |
| `--need` | 文本 | — | 服务需求文本 |
| `--hand` | left / right | right | 执行抓取侧 |

### 4.4 国赛真机使用

```bash
# 真机模式（不带 --sim，启用 ASR/TTS/CV）
python3 competition_node.py --hand left
# 按裁判口令操作即可
```

### 4.5 停止

```bash
# Ctrl+C 停止当前任务
# 或在宿主机：
bash scripts/stop_all.sh
```

---

## 五、代码模块详解

### 5.1 competition_node.py — 国赛主控节点

**文件**：`control/raicom2026/competition_node.py`

统一入口，继承 `rclpy.node.Node`，实现三个任务的完整流程：

```text
prepare() → wait_start → task1() → task2() → task3() → complete
```

- `prepare()`：SD 模式 → MuJoCo Reset（仿真自动点击/真机提示）→ 等待站稳 → LD 模式
- `task1()`：语音唤醒 → 自主导航到交互区-I → 正面朝向交互区-II → 语音播报
- `task2()`：时间问答 → 数字颜色识别 → 表情 → 手势（US 模式切换+双臂控制）
- `task3()`：理解需求 → 语音应答 → 导航作业区 → 泊车 → 抓取 → 语音播报

状态机定义在 `core/scenario.py`：

```text
PREPARE → WAIT_START → NAVIGATE_INTERACTION_I → FACE_INTERACTION_II
→ BASIC_INTERACTION → UNDERSTAND_NEED → NAVIGATE_WORK_ZONE
→ GRASP_AND_LIFT → ANNOUNCE_WHILE_HOLDING → COMPLETE / FAILED
```

### 5.2 core/mode_switch.py — 模式切换

支持 6 种运动模式：`PD` / `DD` / `JD` / `SD` / `LD` / `US`。

关键方法：
- `request(mode)`：仅发送请求，不等待 RUNNING（用于 SD+Reset 后恢复）
- `wait(mode, timeout)`：轮询等待模式进入 RUNNING 状态
- `set(mode)`：idempotent 切换（已是目标模式则直接返回；LD→US 自动过桥 SD）

### 5.3 core/locomotion.py — 速度控制

- **InputSource**：注册/管理 MC 二开输入源（先 DELETE 再 ADD 避免残留）
- **MotionController**：
  - `publish(fwd, angular, lateral)`：发布速度指令到 `/aima/mc/locomotion/velocity`
  - `move_toward(x, y)`：航向修正直线行走，支持 CPG 航向标定、起点脱困、边界/高度/障碍三重安全检查
  - `rotate_to(yaw)`：原地转向
  - `stop(duration)`：连续零速度制动

### 5.4 core/navigator.py — 导航与定位

- **定位来源**：
  - 仿真：自定义激光插件的前两天样本（含世界坐标）
  - 真机：`/map_tf_distribution/localization_pose`
- **路径规划**：`LidarMapper` 实时建图 + `goto()` 内部 A* 规划
- **场地坐标**：
  - `START = (-1.5, -1.5)` — 出发区
  - `INTERACT_I = (0.0, 1.55)` — 交互区-I
  - `INTERACT_II = (0.0, 1.00)` — 交互区-II
  - `WORK_ZONE = (0.65, -0.85)` — 作业区（桌前）
- **特殊处理**：交互区-I 进场需先到中间点再精确进入；泊车功能 `dock_for_grasp()`

### 5.5 core/mapping.py — 激光建图

- 空白栅格起始，Bresenham 线扫描更新占据/空闲
- A* 路径规划（对未知区域加代价）
- 20Hz 前向碰撞碎片检测 `safe_to_advance()`

### 5.6 core/vision.py — 视觉识别

- 63 张官方素材模板匹配（`normalized_glyph` 归一化）
- `COLOR_PALETTE`：15 种颜色色板（粉色/青色/绿色/黄色/紫色/深橙/蓝绿/蓝色/浅蓝/红色/深紫/靛蓝/黄绿/橙色/浅绿）
- 前景分离 → 中位数颜色匹配
- `VisionController.recognize_number(path)` → `{"digit": int, "color": str}`

### 5.7 core/grasp.py — 抓取控制

完整的抓取流水线：

1. `wait_for_arm_state()` — 等待 `/aima/hal/joint/arm/state` 反馈
2. `solve_ik(side, target_xyz)` — IK 求解（Pinocchio 阻尼最小二乘）
3. `move_arm(target, duration)` — 余弦平滑关节轨迹
4. `grip(hand, position)` — 夹爪控制（`/aima/hal/joint/hand/command`）
5. `grasp_and_lift(side, target, lift_height)` — 预抓取→接近→闭爪→垂直抬升
6. `hold_grip(side, duration)` — 语音播报期间保持闭爪

辅助函数：
- `world_to_base(world_xyz, base_xy_yaw)`：世界坐标→机器人基座坐标
- `compose_arm_target(current, side, active)`：只替换目标侧 7 关节

### 5.8 core/gesture.py — 手势控制

五种比赛动作的 14 关节目标姿态：

| 动作 | 说明 |
|------|------|
| 挥左手 | 左手高举+手腕摆动 |
| 挥右手 | 右手高举+手腕摆动 |
| 左手敬礼 | 左手抬至额头 |
| 右手敬礼 | 右手抬至额头 |
| 双手打叉 | 双臂交叉于胸前 |

`GestureController`：`perform()` 执行姿态 → `return_to_ready()` 回到安全姿态。

### 5.9 core/speech.py / core/expression.py / core/scenario.py

- **speech.py**：`--sim` 模式用键盘输入+print 日志；真机接入 ASR/TTS API
- **expression.py**：7 种表情（快乐/悲伤/愤怒/睡觉/充电/疑惑/平静-卖萌）
- **scenario.py**：竞赛状态枚举、需求映射（3 种需求+关键词）、抽签校验

---

## 六、国赛任务流程

```
┌──────────────────────────────────────────────────────────┐
│                  竞赛开始（裁判下发指令）                    │
└──────────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────────┐
│  任务 1：自主导航与交互就位（15 分）                        │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐   │
│  │ 语音唤醒     │ →  │ 自主导航     │ →  │ 面向交互区II │   │
│  │ "前往交互区I" │    │ 出发区→交互区I│    │ 航向对准     │   │
│  └─────────────┘    └─────────────┘    └─────────────┘   │
│  评分：进入(10) + 停止(2) + 朝向(3)                        │
└──────────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────────┐
│  任务 2：基础交互（35 分）                                 │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐   │
│  │①时间问答  │ │②数字识别  │ │③表情切换  │ │④动作执行  │   │
│  │   7 分    │ │ 6+5=11分 │ │   7 分    │ │ 7+3=10分 │   │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘   │
│  表情：悲伤/睡觉/愤怒/快乐/充电                              │
│  动作：挥左手/右手/左手敬礼/右手敬礼/双手打叉                  │
└──────────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────────┐
│  任务 3：场景交互与自主服务（50 分）                        │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐   │
│  │理解需求    │ │语音应答    │ │自主导航    │ │抓取播报    │   │
│  │   8 分    │ │   7 分    │ │   8 分    │ │ 17+10=27 │   │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘   │
│  需求：头部不适→药盒 / 口渴→杯子 / 饥饿→面包                 │
│  抓取：IK 求解 → 手臂轨迹 → 夹爪闭合 → 抬升保持               │
└──────────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────────┐
│  附加分（50 分）：全程自主雷达建图导航                       │
│  从空白地图 → 激光建图 → 定位 → 完成任务1和任务3导航          │
└──────────────────────────────────────────────────────────┘
```

---

## 七、评分合规矩阵

### 基础成绩（100 分）

| # | 验收项 | 分值 | 实现位置 | 自动判定 |
|---|--------|:----:|----------|----------|
| 1 | 自主进入交互区-I | 10 | `navigator.goto(INTERACT_I)` | 基座坐标判定 |
| 2 | 到达后停止 | 2 | `move_toward → stop()` | 1s 零速度 |
| 3 | 正面朝向交互区-II | 3 | `navigator.face(INTERACT_II)` | 航向误差 ≤10° |
| 4 | 正确回答时间 | 7 | `datetime.now()` | 系统时间匹配 |
| 5 | 正确识别数字 | 6 | `vision.NormalizedGlyph` 模板匹配 | 63/63 正确 |
| 6 | 正确识别颜色 | 5 | `vision.COLOR_PALETTE` 15 色 | 中位数匹配 |
| 7 | 显示正确表情 | 7 | `expression.show()` 5 种 | 全部映射 |
| 8 | 执行正确动作 | 7 | `gesture.POSES` 5 种轨迹 | 全部完成 |
| 9 | 播报动作名称 | 3 | `speech.say(f"我正在执行{gesture}")` | 内容包含 |
| 10 | 判断需求 | 8 | `scenario.parse_need()` | 6/6 测试通过 |
| 11 | 对应语音应答 | 7 | `need.response` 三段标准话术 | 文本一致 |
| 12 | 自主到达作业区 | 8 | `goto(WORK_ZONE)` + `dock_for_grasp()` | 基座坐标判定 |
| 13 | 识别并抓取正确物品 | 17 | `grasp.grasp_and_lift()` | 夹爪+离开桌面 |
| 14 | 抓取结果播报 | 10 | `speech.say(need.done)` + `hold_grip()` | 一致+不掉落 |

### 附加成绩（50 分）

| 验收项 | 分值 | 实现位置 |
|--------|:----:|----------|
| 从空白地图自主激光建图+定位+完成两段导航 | 50 | `mapping.LidarMapper` + `navigator.goto` A* |

---

## 八、测试套件

```bash
# 在容器内运行全部测试
cd /workspace/control/raicom2026
python3 tests/test_vision.py && \
python3 tests/test_grasp_math.py && \
python3 tests/test_scenario.py && \
python3 tests/test_mapping.py
# 全部 12 项测试通过 → ALL 12 TESTS PASS
```

| 测试文件 | 测试数 | 覆盖内容 |
|----------|:------:|----------|
| `test_vision.py` | 2 | NumberRecognizer 模板匹配，recognize_color 色板测试 |
| `test_grasp_math.py` | 3 | compose_arm_target 关节拼接，world_to_base 坐标变换 |
| `test_scenario.py` | 3 | parse_need 三种需求识别，validate_draw 输入校验 |
| `test_mapping.py` | 4 | Bresenham 线扫描，A* 规划路径存在性和最优性 |

---

## 九、配置参数

### config/project.env

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ROS_DOMAIN_ID` | 26 | ROS 2 域 ID |
| `RMW_IMPLEMENTATION` | rmw_cyclonedds_cpp | 通信中间件 |
| `DOCKER_IMAGE` | raicom2026-x2-sim:humble | Docker 镜像名 |
| `DOCKER_CONTAINER` | raicom2026-x2-sim | Docker 容器名 |
| `TMUX_SESSION` | raicom-x2 | tmux 会话名 |
| `ROBOT_NAME` | lx2501_3_t2d5_raicom | 竞赛机器人型号 |
| `USE_NVIDIA_GPU` | 1 | 是否启用 GPU |

### core/scenario.py 比赛参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `START` | (-1.5, -1.5) | 出发区坐标 |
| `INTERACT_I` | (0.0, 1.55) | 交互区-I 坐标 |
| `INTERACT_II` | (0.0, 1.00) | 交互区-II 坐标 |
| `WORK_ZONE` | (0.65, -0.85) | 作业区（桌前）坐标 |
| 物品坐标（药盒） | (1.27, -1.60, 0.59) | 世界坐标 |
| 物品坐标（水杯） | (1.27, -1.40, 0.59) | 世界坐标 |
| 物品坐标（面包） | (1.27, -1.20, 0.56) | 世界坐标 |

---

## 十、常见问题

### 仿真相关

| 问题 | 解决方案 |
|------|----------|
| MuJoCo 窗口极卡 | 检查 `glxinfo \| grep renderer`，若为 llvmpipe 需配置 GPU |
| Docker 拉镜像超时 | 配置代理 `/etc/systemd/system/docker.service.d/http-proxy.conf` |
| 仿真启动崩溃 HDS | 已内置 `AGIBOT_ENABLE_HDS_COMPONENT=0` |
| 容器 OOM（exit 137） | GPU 加速可降内存至 ~2.4GB |
| `set -u` 与 ROS 冲突 | 已改为 `set -eo pipefail` + 显式 `set +u` |
| 机器人不移动 | 确认 JD→SD→Reset→LD 流程，检查 `pose_is_fresh()` |

### 比赛代码相关

| 问题 | 解决方案 |
|------|----------|
| 数字识别不准确 | 检查 `resources/numbers/` 下 63 张图片是否完整 |
| 需求匹配失败 | 检查 `scenario.py` 中 NEEDS 关键词是否覆盖官方表述 |
| IK 求解不收敛 | 确认 URDF 模型加载成功，检查手臂状态反馈 |
| 语音交互不可用 | 真机需自行接入 ASR/TTS；仿真用 `--sim` 键盘模拟 |
| 激光建图失败 | 确认激光插件编译成功，检查 `build_lidar_plugin.sh` |

---

## 附录

- [AimDK_X2 官方开发手册](https://x2-aimdk.agibot.com/zh-cn/latest/index.html)
- [灵创平台](https://linkcraft.agibot.com)
- [国赛规则文件（修订稿）](docs/../文件/2026睿抗机器人开发者大赛_智慧养老组_国赛规则文件_修订稿.pdf)
- [国赛评分细则（修订稿）](docs/../文件/2026睿抗机器人开发者大赛_智慧养老组_国赛评分细则_修订稿.pdf)
