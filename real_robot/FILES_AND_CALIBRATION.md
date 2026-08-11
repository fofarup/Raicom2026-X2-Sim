# 真机文件说明与现场标定清单

## 目录作用

| 文件 | 作用 | 是否会让机器人运动 |
|---|---|---|
| `create_config.sh` | 首次生成私有`real_robot.json`，不会覆盖已有配置 | 否 |
| `app/` | 可独立运行的真机Task1/2/3、语音、视觉、模型和IK代码 | 按入口决定 |
| `app/config/real_robot.template.json` | 真机参数模板，`null`表示必须现场确认 | 否 |
| `app/config/real_robot.json` | 真机私有参数，Git忽略 | 读取后可能用于运动 |
| `preflight.py` | 检查相机、深度、里程计、音频话题及ROS服务类型 | 否 |
| `preflight.sh` | 串联接口、配置和音频预检 | 否 |
| `relocate.sh` | 只读检查或受保护执行官方SLAM重定位 | 仅`execute`会发布重定位消息 |
| `site2_run.sh` | 二号地图重定位成功后启动统一语音总控 | 是 |
| `voice_preflight.sh` | 检查VAD、音频焦点和扬声器接口 | 否 |
| `setup_only_voice.sh` | 关闭/恢复X2原生交互并重启agent，不重启MC | 会修改agent模式 |
| `run.sh` | 强制加载真机配置后进入统一比赛程序 | 是 |
| `voice_run.sh` | 启动真机VAD→ASR→语义→任务→PCM播报完整链路 | 是 |
| `deploy.sh` | 通过SSH/rsync增量同步代码及本地已准备的ASR模型，默认只预览 | 否 |
| `build_bundle.sh` | 生成一个供U盘复制的真机`.tar.zst`压缩包 | 否 |

## 不需要逐文件传输

电脑与真机网络互通时，先预览：

```bash
./deploy.sh agi@真机IP --dry-run
```

确认清单后执行：

```bash
./deploy.sh agi@真机IP --execute
```

它只同步这套独立`real_robot/`，其中包含真机核心和IK SDK，不传上一级仿真目录。
公开仓库不包含 ASR 权重；若已按 `MIGRATION_GUIDE.md` 在本地补齐模型，rsync 会一并
传输。后续只发送变化文件，真机现场已经标定的 `app/config/real_robot.json` 永远
不会被覆盖。

没有SSH时生成单个U盘包：

```bash
./build_bundle.sh
```

把生成的`raicom_real_robot.tar.zst`复制到真机，再执行：

```bash
tar --zstd -xf raicom_real_robot.tar.zst -C /home/agi/x2_deploy_workspace/
```

`app/`是独立真机快照，不读取上一级`competition/`或`simulation/`。以后仿真与
真机修改必须分别回归；真机现场修复应直接修改`app/`并记录。

## 到手后按顺序检查

二号场地当前地图ID为`1786066723179`，地图数据库记录的建图原点像素为
`(465, 200)`。先运行只读检查：

```bash
./relocate.sh check --map-id 1786066723179
```

实际重定位必须显式输入像素位置和朝向，并同时满足环境变量确认、
`--confirm-at-pose`确认及“没有比赛进程正在运行”三项保护。`site2_run.sh`会在收到
连续有效的`/slam/lidar_odom`后才启动`voice_run.sh`。地图原点像素不是米制导航
坐标，机器人不在建图原点时不得照抄。

### 1. 运行环境

- 确认真机CPU架构；`app/`不包含x86仿真wheel，aarch64 ASR运行库需现场安装；
- 确认AimDK安装路径、ROS 2 Humble、`ROS_DOMAIN_ID`和RMW；
- 确认比赛电脑和机器人网络互通，公共网络下DeepSeek可访问，断网时本地规则仍工作；
- 运行`ros2 node list`、`ros2 topic list`和`ros2 service list`。

### 2. 生成配置并检查接口

```bash
cd /home/agi/x2_deploy_workspace/raicom_real_robot
./create_config.sh
# 编辑 app/config/real_robot.json，替换所有 null
./preflight.sh
```

必须确认RGB、深度、CameraInfo、定位、里程计、VAD和播放话题的名称、消息类型、
频率与时间戳。Task2已从`hardware.rgb_topic`读取真机相机，不再使用硬编码话题。

### 3. 场地坐标与导航

- 在同一地图坐标系记录出发区、交互区I、交互区II、作业区圆心；
- 测量桌面中心坐标及机器人最终朝向；
- 分别标定直行、原地转向漂移、脚尖/脚跟足迹偏置；
- 从低速、短距离开始，确认急停、零速度和SD切换后再跑完整Task1；
- 仿真的`-0.35m`转向补偿、区域坐标和容差不得复制到真机。

### 4. Task2摄像头与交互

- 保存每个数字、颜色、距离和倾角的真机截图；
- 检查像素尺寸、曝光、白平衡、运动模糊和背景干扰；
- 用真实截图回归，必要时标定HSV并加入卡片矩形/透视矫正；
- 连续调用时间、识图、表情和五种预设动作，动作间预留MC释放时间；
- 真实相机回归稳定后再决定是否训练模型；当前模板法不要求训练。

### 5. 语音与音频

- 按官方流程切`only_voice`，只重启agent；
- 验证VAD话题能收到完整PCM语句；
- 为aarch64安装匹配的SenseVoice运行库，不能使用仿真wheel；
- 验证自研TTS PCM扬声器链路，不调用官方`PlayTts`；
- 测试同音纠错、DeepSeek兜底、2秒超时和断网回退；
- 现场噪声下检查录音增益、截断、回声和连续十句话识别率。

### 6. 真机抓取（当前最大未完成项）

`app/real_grasp_agent.py`已按官方接口订阅RGB-D和手状态，直接手控制默认锁定。
完整自主抓取必须先完成：

- RGB与深度对齐、相机内参和`camera_to_robot_transform`外参；
- 药盒、纸杯、面包检测及三维中心估计；
- 真机URDF、14关节名称/顺序、零位、方向、限位和速度；
- 腰部/手臂预抓取、接近、闭合、抬升轨迹；
- OmniPicker命令/状态话题、开闭方向、力或电流阈值；
- 抓取成功判定和失败撤退动作；
- 空载低速、软物体、正式物品三级测试，并由人员持急停保护。

官方文档要求直接向`/aima/hal/joint/hand/command`控制手部前停止PC1原生MC；
若仍需MC保持走跑管理，必须向技术支持确认系统侧并行控制方案。接口核对依据见
`OFFICIAL_INTERFACE_AUDIT.md`。

## 安全验收顺序

```text
只读接口检查 → SD/急停 → 低速直行 → 低速转向 → Task1
→ Task2单动作 → 相机识别 → 音频输入输出 → 单关节小幅运动
→ 空载抓取轨迹 → 软物体抓取 → 完整语音比赛流程
```

每一级通过并记录日志后才进入下一级；不要在真机第一次启动时直接跑完整流程。
