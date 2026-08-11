# X2 真机代码迁移与部署指南

本目录是现场 X2 真机程序的脱敏快照。它面向 Ubuntu 22.04、ROS 2 Humble、
aarch64 的灵犀 X2/AimDK 环境，不是任意机器人均可直接运行的通用程序。

仓库刻意不包含真实 API 密钥、`app/config/real_robot.json`、豆包密钥文件和
SenseVoice 大模型权重。迁移时必须为新机器人重新建立这些私有文件，不能复制旧
机器的场地坐标和外参后直接运动。

## 1. 能迁移到什么机器

### 另一台灵犀 X2

满足下列条件时可以完整迁移：

- Ubuntu 22.04/aarch64，ROS 2 Humble；
- 安装与目标固件匹配的 AimDK 和 `aimdk_msgs`；
- 机器人 ROS 图提供代码配置的定位、速度、相机、音频、MC 和手爪接口；
- 有现场急停人员，并能先做低速单项标定。

固件、消息字段或服务名不同，必须先写适配层，不能直接运行运动任务。

### 普通 Ubuntu 电脑或非 X2 机器人

普通电脑只能运行不依赖真机 ROS 图的意图、需求和图像算法测试。若要控制其他
机器人，需要替换 `aimdk_msgs`、MC 模式切换、速度话题、定位消息、表情、音频和
机械臂/手爪接口。

## 2. 安全获取代码

只需要真机代码、不下载仓库中的大型仿真 LFS 资源时：

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/fofarup/Raicom2026-X2-Sim.git
cd Raicom2026-X2-Sim/real_robot
```

建议先在开发电脑准备私有副本，再用本目录的 `deploy.sh` 增量部署。不要把密钥和
现场配置提交到 Git。

## 3. 检查目标 X2 基础环境

在目标机器人上执行只读检查：

```bash
uname -m
lsb_release -ds
test -f /opt/ros/humble/setup.bash
test -f /home/agi/aimdk/install/setup.bash
source /opt/ros/humble/setup.bash
source /home/agi/aimdk/install/setup.bash
ros2 node list --no-daemon --spin-time 3
ros2 topic list -t
ros2 service list -t
```

预期架构为 `aarch64`。AimDK 路径若不同，应在新机器的私有配置中填写真实路径。

Python 运行时至少需要：

- `numpy`、OpenCV、Pillow、PyYAML；
- ROS 2 的 `rclpy`、`cv_bridge`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`；
- 目标 AimDK 的 `aimdk_msgs`；
- `sherpa-onnx`（SenseVoice）、`edge-tts` 和 FFmpeg（当前 TTS 方案）。

先检查，不要盲目覆盖机器人系统包：

```bash
python3 -B - <<'PY'
import importlib.util
for name in ("rclpy", "cv2", "cv_bridge", "numpy", "sherpa_onnx"):
    print(f"{name}: {'OK' if importlib.util.find_spec(name) else 'MISSING'}")
PY
command -v edge-tts
command -v ffmpeg
```

缺少纯 Python 依赖时可使用目标平台兼容的 wheel，例如：

```bash
python3 -m pip install --user sherpa-onnx edge-tts
sudo apt install -y ffmpeg python3-numpy python3-pil python3-yaml
```

Jetson/X2 上的 OpenCV、ROS 和 NumPy 往往由系统镜像提供；安装前应确认不会破坏
`cv_bridge` 的 ABI。

## 4. 准备离线 ASR 模型

运行时要求以下路径存在：

```text
app/models/asr/sensevoice/model.int8.onnx
app/models/asr/sensevoice/tokens.txt
```

从你有权使用的私有备份或 SenseVoice/FunASR 官方来源取得文件，然后放入上述目录。
现场备份中模型的参考 SHA-256 为：

```text
c71f0ce00bec95b07744e116345e33d8cbbe08cef896382cf907bf4b51a2cd51  model.int8.onnx
```

校验：

```bash
sha256sum app/models/asr/sensevoice/model.int8.onnx
test -s app/models/asr/sensevoice/tokens.txt
```

模型许可应以 FunASR/SenseVoice 上游项目为准，不能因为本项目可访问就推定可以再次
分发权重。

## 5. 创建新机器私有配置

```bash
./create_config.sh
chmod 600 app/config/real_robot.json
```

编辑 `app/config/real_robot.json`，逐项填写所有 `null`。至少要重新核对：

- `runtime`：ROS Domain、RMW、ROS/AimDK setup 路径；
- `topics`、`services`：目标固件上的真实名称和消息类型；
- `navigation.zones`：新地图的出发区、交互区和作业区坐标；
- Task 1 转向点、最终角度、漂移补偿和速度上限；
- 桌子物理位置、足迹偏置和所有对桌容差；
- RGB-D、CameraInfo、机械臂和手爪接口；
- 相机到机器人外参及抓取标定状态。

定位配置有两种常见模式：

```json
{
  "localization_pose": "/slam/lidar_odom",
  "localization_pose_type": "nav_msgs/msg/Odometry"
}
```

这是 `relocate_agent.py` 和二号场地 `site2_run.sh` 要求的官方模式。若只运行自定义
地图/TF 转发，也可使用：

```json
{
  "localization_pose": "/map_tf_distribution/localization_pose",
  "localization_pose_type": "geometry_msgs/msg/PoseStamped"
}
```

第二种模式可用于 Task 1/3，但当前重定位预检会拒绝它；运行二号场地前必须切回
官方 `/slam/lidar_odom` 配置。

不要用 `0`、`[0, 0]` 或单位矩阵代替尚未测量的坐标和外参。形式校验通过不代表
物理标定完成。

## 6. 配置密钥和联网服务

DeepSeek 密钥只通过环境变量提供：

```bash
read -rsp 'DeepSeek API key: ' DEEPSEEK_API_KEY
export DEEPSEEK_API_KEY
echo
```

豆包视觉从权限为 `600` 的文件读取：

```bash
install -m 600 /dev/null app/config/doubao_api_key
# 使用安全编辑器写入密钥，不要把密钥作为命令行参数或提交到 Git。
```

同时在 `real_robot.json` 的 `vision_cloud.model` 中填写账户可用的模型或接入点 ID。
当前识图会把一帧相机 JPEG 发送到火山引擎；模糊语音文本可能发送到 DeepSeek；
Edge TTS 会发送待播报文本。离线或隐私场景应禁用相应云能力。

当前 TTS 命令配置示例：

```json
["/usr/bin/python3","/home/agi/x2_deploy_workspace/raicom_real_robot/app/edge_tts_wav.py","--text","{text}","--output","{output}"]
```

把这个数组序列化为字符串后填入 `audio.tts_command_json`。

## 7. 从开发电脑部署到新 X2

先预览，确认没有覆盖私有配置：

```bash
./deploy.sh agi@NEW_ROBOT_IP --dry-run
```

确认清单后执行：

```bash
./deploy.sh agi@NEW_ROBOT_IP --execute
```

目标目录固定为：

```text
/home/agi/x2_deploy_workspace/raicom_real_robot
```

`deploy.sh` 不同步 `real_robot.json`、豆包密钥及缓存。模型权重和私有配置应通过受控
渠道单独复制，并在目标机上设置为仅 `agi` 可读。

若使用 SSH 密钥访问 X2 的 PC42 相机侧，设置：

```bash
export RAICOM_PC42_SSH_TARGET=agi@10.0.1.42
```

先把目标主机密钥加入 `known_hosts`，并配置无交互 SSH key。公开版本不使用
`sshpass`，也不会关闭主机密钥检查。

## 8. 分级验证顺序

以下前四项不控制机器人运动：

```bash
export PYTHONDONTWRITEBYTECODE=1
PYTHONPATH=app python3 app/profile_check.py
python3 app/voice_intents.py
python3 app/task3_needs.py
./voice_preflight.sh
```

使用官方 `/slam/lidar_odom` 配置时，再运行完整只读预检：

```bash
./preflight.sh
./relocate.sh check --map-id TARGET_MAP_ID
```

随后必须按以下顺序，由人员持急停进行现场验收：

1. 验证遥控器、急停和软件零速度；
2. 低速、短时直行标定；
3. 单独验证原地转向和定位丢失停车；
4. Task 1 每条路径至少重复多次；
5. Task 2 逐项验证表情、动作和相机识别；
6. 验证 VAD、ASR、TTS 和扬声器回音；
7. 最后才考虑手爪和机械臂。

会让机器人运动的命令不能在无人保护时照抄执行。确认现场安全后，入口形式为：

```bash
./run.sh 1
./run.sh 2
./run.sh 3 --navigate-only
./run.sh 3 --navigate-and-align
```

完整语音链路应最后启用：

```bash
export RAICOM_CONFIRM_REAL_ROBOT=YES
./setup_only_voice.sh only_voice
./voice_preflight.sh
./voice_run.sh
```

`setup_only_voice.sh` 会修改 agent 模式并重启 agent，不是只读命令。

## 9. 二号场地重定位

先只读核对地图：

```bash
./relocate.sh check --map-id TARGET_MAP_ID
```

真正执行前必须测量机器人所在像素位置和朝向，并满足代码要求的双环境变量、
`--confirm-at-pose` 和进程互斥保护。不要把旧机器人或旧场地的地图 ID、像素原点和
角度复制到新机器。

## 10. 当前功能边界

- Task 1/3 直接发布底盘速度，未接入障碍物规划器或代价地图；
- 地图定位失效、输入源优先级和场地边界必须在真机复核；
- Task 2 正式识图目前使用豆包云视觉，本地模板识别器未接入主流程；
- Task 3 正式语音流程只完成需求判断、导航和对桌，仍会拒绝抬臂；
- `bread_grasp_real.py` 是后来加入的固定面包抓取实验轨迹，没有视觉定位，也没有
  接入正式 Task 3；它要求双重环境变量确认及 `grasp.calibrated=true`，只能由有人
  持急停时单独调试；
- `real_grasp_agent.py` 只是接口/手爪测试，不是完整抓取规划器；
- IK SDK 已附带，但尚未接入自主抓取执行链。

在这些边界解决并完成重复性测试前，不应把本目录视为可无人监管运行的成品系统。

## 11. 故障定位

- `profile_check` 报字段缺失：重新从当前模板生成配置并逐项迁移旧值；
- 重定位报告只接受 `Odometry`：切换到 `/slam/lidar_odom` 配置；
- ROS 节点互相不可见：核对 `ROS_DOMAIN_ID`、RMW 和 DDS 配置；
- 相机无帧：核对相机所在 SoC、话题类型、QoS 和内部 SSH key；
- ASR 初始化失败：核对 aarch64 `sherpa-onnx`、模型路径和 SHA-256；
- TTS 无声：核对网络、FFmpeg、音频焦点服务和 PCM 播放话题；
- 动作或模式服务超时：停止继续运动，检查 AimDK 版本和跨板服务状态。

任何运动异常都应先停车、切回稳定站立并由人工确认，而不是增加重试次数继续运行。
