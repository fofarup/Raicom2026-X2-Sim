# 睿抗 2026 X2 真机调试指南（Codex）

> 适用项目：`Raicom2026-X2-Sim/control/raicom2026`
>
> 目标：在不把仿真假设带到真机的前提下，逐项建立真实 X2 的连接、感知、交互、运动和比赛闭环。
>
> 文档定位：**调试与验收指南，不是“一键运行”说明书。**
>
> 基准接口：AimDK_X2 1.0.0 官方文档（实际比赛机器人版本必须现场复核）。

---

## 1. 首先明确责任边界

1. Codex 只负责检查代码、文档、接口定义和用户提供的日志，不连接、不操作、不控制真机。
2. 所有真机命令均由现场操作员通过 SSH 在 PC2 上人工执行。
3. 任何可能导致站立、行走、转向、挥臂或夹爪动作的步骤，都必须由安全员确认后再执行。
4. 未通过上一阶段验收，不得进入下一阶段；不得为了赶时间跳过安全门禁。
5. 正式比赛运行期间按竞赛规则执行，不能把调试阶段的键盘输入、人工遥控或多次启动带入计分流程。

本文命令按风险分为三级：

| 标识 | 含义 | 是否可能让机器人运动 |
|---|---|---:|
| `[RO]` | 只读检查，例如查看版本、话题和接口 | 否 |
| `[IO]` | 交互输出，例如屏幕、麦克风、扬声器 | 通常否 |
| `[MOTION]` | 运动控制，例如模式切换、速度、手臂和夹爪 | **是** |

看到 `[MOTION]` 时，必须同时满足：机器人周围清场、安全员就位、急停可用、机器人状态正常、当前阶段允许运动。

---

## 2. 当前代码的真机状态

截至本指南编写时，项目**尚未达到直接运行完整真机流程的条件**。下列阻断项解决并验收前，禁止执行完整比赛节点：

| 编号 | 阻断项 | 当前证据 | 解除条件 |
|---|---|---|---|
| B-01 | 真机启动参数容易用错 | `--sim` 默认值为 `True` | 明确使用 `--no-sim`，并增加启动模式确认 |
| B-02 | 真机准备流程包含 MuJoCo Reset 提示 | `prepare()` 真机路径仍提示点击 MuJoCo Reset | 真机路径不再引用 MuJoCo，且经过无运动逻辑测试 |
| B-03 | 主程序不负责 SLAM 重定位 | 未发布 `/integrated_command` 和 `/relocalization_pose` | 建立独立重定位流程并验证 `/slam/lidar_odom` |
| B-04 | 真机激光未接入导航器 | `LidarMapper` 仍订阅 `/aima/sim/lidar/points` | 接入真机点云、正确 TF 和有效障碍停车测试 |
| B-05 | 相机同步等待可能阻塞回调 | `_capture()` 等待时没有 spin/后台执行器 | 真机连续拍照测试稳定通过 |
| B-06 | PC2 本地播放不等于机器人扬声器播放 | TTS 使用 `paplay/aplay` | 通过 PC3 官方音频通道播放自研生成的 PCM/WAV |
| B-07 | ASR 录音设备写死 | 固定 `plughw:CARD=PCH,DEV=0` | 使用实际麦克风源并完成现场噪声测试 |
| B-08 | 真机没有物品检测 | 物品坐标来自仿真点云或固定坐标 | 真机视觉/深度定位输出真实物品坐标 |
| B-09 | 真机抓取没有成功闭环 | 抬升和掉落验证仅仿真启用 | 读取手爪/物品状态并验证离桌和保持 |
| B-10 | 夹爪参数和控制权存在风险 | `effort=8.0`，官方范围为 `0.0~1.0` | 修正参数，并由技术支持确认 MC 与手爪共存方案 |
| B-11 | AimDK 消息版本未锁定 | 代码混用不同版本字段/请求头 | 在目标机器人逐项完成 `ros2 interface show` |

> **总门禁：B-01 至 B-11 未全部关闭前，不执行 `competition_node.py --no-sim` 的完整任务流程。**

---

## 3. 人员、场地与停止机制

真机调试至少安排两人：

- 操作员：通过 SSH 输入命令、观察日志，一次只执行一个已批准步骤。
- 安全员：观察机器人和周围环境，负责要求立即停止；急停操作遵循 X2 用户手册和现场技术人员要求。

开始前确认：

- [ ] 地面平整、干燥、无电缆和杂物。
- [ ] 机器人运动范围及手臂摆动范围内无人。
- [ ] 机器人电量、关节、手爪和传感器无故障报警。
- [ ] 机器人与桌面、围挡之间保留足够余量。
- [ ] 网络稳定，SSH 至少保留一个专用观察终端。
- [ ] 已明确“停止命令”“急停”和断电的责任人及操作方式。
- [ ] 调试用坐标、速度和关节姿态已经由现场技术支持审核。
- [ ] 日志目录和本次测试编号已经建立。

立即停止条件：

- 定位超过 0.5 秒不更新或发生明显跳变；
- 机器人身体高度、姿态或足底状态异常；
- 实际运动方向与命令方向不一致；
- 靠近人员、围挡、桌腿或未知障碍；
- 手臂抖动、持续顶住限位、碰撞身体或桌面；
- 手爪过流、过温、堵转或状态反馈异常；
- ROS 节点崩溃、SSH 中断或控制源状态不明确。

停止后不要立即重试。先保存终端输出、ROS 时间、最后一条命令和机器人现象，再分析原因。

---

## 4. 阶段 0：代码与版本冻结（不连接真机）

目标：确保部署的是一个可追溯版本，且没有把密钥或本地配置提交到仓库。

在笔记本项目目录执行：

```bash
# [RO] 确认版本和工作区
cd ~/x2_ws/x2_biao
git status -sb
git rev-parse HEAD
git remote -v

# [RO] 确认关键文件存在
test -f control/raicom2026/competition_node.py
test -d control/raicom2026/core
test -d control/raicom2026/ik_sdk
test -d control/raicom2026/map_tf/map_tf_distribution
```

要求：

- `config/asr_keys.json`、模型大文件和现场地图按项目策略管理，不得意外提交密钥。
- 记录本次部署的 commit SHA。
- 不以“仿真通过”代替真机接口验收。

验收：版本可追溯，部署范围明确，秘密文件不在提交范围。

---

## 5. 阶段 1：连接 PC2，只做只读检查

### 5.1 建立有线连接

笔记本连接机器人背部 SDK 二次开发网口。先确认实际网卡名，不要照抄 `eth0`：

```bash
# [RO] 笔记本执行
ip -br link
ip -br addr
```

由操作员在正确接口上配置 `10.0.1.2/24`，随后验证：

```bash
# [RO]
ping -c 3 10.0.1.41
ssh agi@10.0.1.41
```

PC1 `10.0.1.40` 禁止作为二次开发运行环境。除非 X2 官方技术支持明确指导，不在 PC1 执行管理或控制命令。

### 5.2 记录系统版本

SSH 登录 PC2 后执行：

```bash
# [RO]
uname -a
cat /etc/os-release
python3 --version
ros2 --version 2>/dev/null || true
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未设置}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-默认}"
```

然后按机器人现场提供的 AimDK 环境脚本完成 `source`。不要假设一定是 `~/aimdk/install/setup.bash`，先用官方交付说明确认路径。

```bash
# [RO] source 后检查
printenv | grep -E 'ROS_DISTRO|ROS_DOMAIN_ID|RMW_IMPLEMENTATION'
ros2 pkg prefix aimdk_msgs
```

预期：ROS 2 Humble、`ROS_DOMAIN_ID=26`（若现场配置不同，以组委会为准）、`aimdk_msgs` 可查询。

失败处理：只记录，不编译或覆盖系统 AimDK。先向技术支持确认固件、SDK 和环境脚本。

---

## 6. 阶段 2：冻结真实接口契约

AimDK 1.0.0 对消息和服务有过变更，必须在目标机器人上保存真实定义。

```bash
# [RO] 话题和服务快照
mkdir -p ~/raicom_debug_logs
stamp=$(date +%Y%m%d_%H%M%S)
ros2 topic list -t > ~/raicom_debug_logs/topics_${stamp}.txt
ros2 service list -t > ~/raicom_debug_logs/services_${stamp}.txt
ros2 node list > ~/raicom_debug_logs/nodes_${stamp}.txt

# [RO] 关键接口定义
ros2 interface show aimdk_msgs/srv/SetMcInputSource
ros2 interface show aimdk_msgs/srv/SetMcAction
ros2 interface show aimdk_msgs/srv/PlayEmoji
ros2 interface show aimdk_msgs/msg/HandCommandArray
ros2 interface show aimdk_msgs/msg/HandCommand
ros2 interface show aimdk_msgs/msg/UpperBodyCommandArray
ros2 interface show aimdk_msgs/srv/PlayAudioFile
```

同时核对：

```bash
# [RO]
ros2 topic type /aima/mc/locomotion/velocity
ros2 topic type /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
ros2 topic type /aima/hal/sensor/rgbd_head_front/rgb_image
ros2 topic type /slam/lidar_odom
ros2 service type /aimdk_5Fmsgs/srv/SetMcInputSource
ros2 service type /aimdk_5Fmsgs/srv/PlayEmoji
```

对照代码重点检查：

- 消息字段到底是 `position/velocity/effort`，还是 `pos/vel/tor`。
- `PlayEmoji.Request.header` 是 `CommonRequest`、`RequestHeader` 还是其他类型。
- `SetMcInputSource` 的 action 值、超时单位和响应成功条件。
- 上肢命令的话题名、QoS、字段名和 `command_source` 约束。
- OmniPicker 左右手数组长度、名称、取值范围和状态话题。

验收：形成一份接口快照，代码中每个真机 publisher/client 均与目标机器人接口一致。任何一项不一致，先修改代码并回到仿真/静态测试，不进入运动测试。

---

## 7. 阶段 3：部署独立副本与依赖检查

推荐部署到带 commit 标识的新目录，避免覆盖上一版：

```bash
# [RO] 笔记本执行；把 <sha> 替换为本次 commit 短 SHA
scp -r control/raicom2026 agi@10.0.1.41:~/raicom2026_<sha>
```

不要向 `$HOME/aimdk*` 写入项目文件。不要在版本未知时重编机器人系统管理的 AimDK。

在 PC2 项目副本中检查：

```bash
# [RO]
cd ~/raicom2026_<sha>
python3 -m compileall -q competition_node.py core ik_sdk
python3 -c 'import numpy, PIL; print("基础 Python 依赖正常")'
python3 -c 'import pinocchio; print("Pinocchio 正常")'
python3 -c 'import aimdk_msgs; print(aimdk_msgs.__path__)'
```

`ik_sdk/pyproject.toml` 声明了 `numpy` 和 `pin`。Jetson/ARM 环境安装方式应以官方环境和现场技术支持为准，不要为了安装一个包破坏机器人已有 Python/ROS 环境。

如果需要编译 `map_tf_distribution`，使用独立工作空间：

```bash
# [RO/BUILD] 只编译项目自带包，不编译系统 AimDK
mkdir -p ~/raicom_map_ws/src
cp -r ~/raicom2026_<sha>/map_tf/map_tf_distribution ~/raicom_map_ws/src/
cd ~/raicom_map_ws
colcon build --packages-select map_tf_distribution
source install/setup.bash
ros2 pkg prefix map_tf_distribution
```

验收：Python 导入、项目包编译和 AimDK 消息加载全部成功；系统 AimDK 未被覆盖。

---

## 8. 阶段 4：无运动传感器检查

### 8.1 定位和机体状态

```bash
# [RO] 每条只取一帧并限时
timeout 5 ros2 topic echo --once /slam/lidar_odom
timeout 5 ros2 topic hz /slam/lidar_odom
```

不要仅以“有话题名”判断可用，必须看到持续更新、时间戳前进、数值有限且机器人静止时漂移合理。

### 8.2 激光雷达

```bash
# [RO]
ros2 topic info -v /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
timeout 5 ros2 topic hz /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
timeout 5 ros2 topic echo --once \
  /aima/hal/sensor/lidar_chest_front/lidar_pointcloud header
```

验收：点云约定与实际 QoS 匹配，frame_id 可通过 TF 变换到机器人或地图坐标系。禁止把仿真点云中“前几个点编码物体/位姿”的约定用于真机点云。

### 8.3 RGB 相机

```bash
# [RO]
ros2 topic info -v /aima/hal/sensor/rgbd_head_front/rgb_image
timeout 5 ros2 topic hz /aima/hal/sensor/rgbd_head_front/rgb_image
timeout 5 ros2 topic echo --once \
  /aima/hal/sensor/rgbd_head_front/rgb_image header
```

记录实际 `encoding`、`width`、`height`、`step` 和 QoS。数字卡识别必须在真实相机画面上验证 ROI、距离、角度、曝光和现场灯光，不能只验证仓库中的 63 张标准图片。

### 8.4 麦克风

优先核对 AimDK 1.0 的 Mic Source 和 Raw Audio Capture 接口。只有确认 PC2 暴露了可靠 ALSA 设备时，才使用 `arecord`：

```bash
# [RO]
arecord -l
pactl list sources short 2>/dev/null || true
```

当前 `offline_asr.py` 写死 `plughw:CARD=PCH,DEV=0`，在完成适配前不能认为真机 ASR 可用。

验收：四类传感器均有真实、连续、时间戳正确的数据，且代码使用的话题和 QoS 与现场一致。

---

## 9. 阶段 5：无运动交互功能

### 9.1 表情

先只读检查：

```bash
# [RO]
ros2 service type /aimdk_5Fmsgs/srv/PlayEmoji
ros2 interface show aimdk_msgs/srv/PlayEmoji
ros2 topic type /face_ui_proxy/status
```

只有 `test_expression.py` 与现场请求头结构一致、并能检查响应 `success` 后，才进行 `[IO]` 测试。测试时一次只显示一个表情，并同时观察 `/face_ui_proxy/status`，不能仅凭 service future 非空判定成功。

### 9.2 自研 TTS 的机器人扬声器输出

合规链路应拆成两部分：

```text
自研 Piper/Edge TTS 生成 PCM/WAV
              ↓
AimDK Audio Focus + Raw Audio Playback，或 PC3 PlayAudioFile
              ↓
机器人扬声器真实发声
```

使用 `PlayAudioFile` 时，官方要求文件位于 PC3 `10.0.1.42` 且为可读的 WAV PCM/RAW PCM；MP3 不受支持。若使用 `/aima/hal/audio/playback`，需要正确申请和释放 Audio Focus。

当前 `core/tts.py` 只在 PC2 调用 `paplay/aplay`。在完成 PC3 播放适配前，`test_voice.py --tts` 返回成功也不能证明机器人扬声器能播报。

### 9.3 ASR

依次测试：

1. 安静环境下十次固定口令；
2. 现场噪声环境下十次固定口令；
3. 肯定与否定语句，例如“对”“不对”“是”“不是”；
4. 数字、颜色、动作和需求词表；
5. 无网络时的离线回退。

每条记录原始识别文本、纠正后文本、耗时和最终状态机选择。特别检查否定词不能被纠正成肯定词。

验收：表情有状态反馈，扬声器真实出声，ASR 在现场噪声下达到预定正确率，失败时不会静默当成成功。

---

## 10. 阶段 6：SLAM 重定位与坐标标定

官方 SLAM 是可选模块，首先由技术支持确认已启用。标准流程是：

1. 获得准确的 `map_id` 和 `map_name`；
2. 向 `/integrated_command` 发布 `start_relocalization:<map_id>`；
3. 约一秒后向 `/relocalization_pose` 发布像素坐标初始位姿；
4. 收到 `/slam/lidar_odom` 后才判定重定位成功。

`/relocalization_pose` 使用地图像素坐标，不是导航使用的米制坐标。不得把代码中的 `(-1.5, -1.5)` 直接作为重定位像素坐标。

官方地图目录通常包含 `grid_map.png` 和 `grid_map_info.txt`。项目 `map_tf_distribution` 需要标准 ROS map YAML，不能假设官方目录一定存在 `grid_map.yaml`。如需发布 ROS 地图，必须先生成并人工复核：

- PNG 路径；
- 米/像素分辨率；
- ROS 地图原点；
- 图像 Y 轴翻转；
- `map → base_link` TF 的方向和单位。

只读验收：

```bash
# [RO]
timeout 10 ros2 topic hz /slam/lidar_odom
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

静止验收标准：

- 连续收到定位，不间歇消失；
- x、y、yaw 没有大跳变；
- `map → base_link` 与 `/slam/lidar_odom` 一致；
- 起点、交互区、作业区全部通过现场地图测量转换为同一米制坐标系；
- 代码里的硬编码区域边界与真实地图原点一致，否则不得启用边界急停。

---

## 11. 阶段 7：首次运动——只做最小动作

本阶段全部为 `[MOTION]`。开始前再次完成第 3 节检查。

### 11.1 控制权确认

先只读检查 MC 服务、输入源和当前动作状态。所有模式切换必须采用目标固件对应的官方接口和响应判定。

通过条件：

- 输入源注册响应明确成功；
- source 名称和速度消息一致；
- timeout/租约单位已经确认；
- 控制进程退出或失联后能在规定时间内停止输出；
- 不存在另一个高优先级来源争抢控制权。

### 11.2 首次站立与速度方向测试

不要直接运行完整 `competition_node.py`。使用经过审查的最小测试节点，按以下顺序：

1. 只完成官方要求的站立准备，确认站稳；
2. 发布持续零速度并确认机器人静止；
3. 以现场允许的最低有效速度前进极短距离；
4. 停止并测量实际位移方向；
5. 分别验证后退、横移和小角度转向；
6. 每个方向单独批准、单独执行、单独停止。

当前代码带有来自仿真的 `COURSE_YAW_OFFSET=12°`、速度死区和边界常数。这些参数不得直接视为真机标定结果。

验收：命令方向、实际方向、坐标方向一致；停止可靠；定位不中断；无摔倒和异常漂移。

---

## 12. 阶段 8：低速导航与障碍停车

### 12.1 空场点到点

先使用离起点很近、无遮挡、远离围挡的目标点：

- 第一轮只验证 0.2~0.3 m 直行；
- 第二轮验证小角度修正；
- 第三轮才验证多段路径；
- 每轮记录目标、实际终点、最大定位延迟、耗时和停止误差。

### 12.2 障碍检测

只有真机点云已接入、TF 正确且运动循环实际调用安全检查后，才测试障碍停车。障碍物由安全员按批准方案布置，不能由人员站到机器人路径上充当障碍物。

必须验证：

- 前方障碍能使速度归零；
- 点云断流会停车，而不是继续盲走；
- TF 失败会停车；
- 未知区域策略明确；
- A* 路径和实际可通行空间一致；
- 后退及转向也有对应碰撞风险控制。

验收：交互区和作业区导航分别完成多次，停止位置与朝向满足评分要求，且没有触碰围挡。

---

## 13. 阶段 9：手臂和 OmniPicker

### 13.1 先解决控制权问题

AimDK 1.0 官方说明：自主控制手部前通常需要停用原生 MC；若需要同时保留原生行走管理，应联系技术支持做系统级适配。

本项目同时依赖原生 MC 行走和 `/aima/hal/joint/hand/command`。在技术支持书面确认共存方式前：

- 不执行 `test_gripper.py`；
- 不执行 `grasp_and_lift()`；
- 不自行在 PC1 停止 MC；
- 不根据仿真结果猜测控制权切换顺序。

### 13.2 夹爪参数和反馈

OmniPicker 官方标准范围为：

| 字段 | 范围 |
|---|---:|
| position | 0.0~1.0 |
| velocity | 0.0~1.0 |
| acceleration | 0.0~1.0 |
| deceleration | 0.0~1.0 |
| effort | 0.0~1.0 |

当前 `core/grasp.py` 使用 `effort=8.0`，必须修正并完成接口版本核对后才能测试。

夹爪成功必须依据 `/aima/hal/joint/hand/state` 的位置、状态和 faultcode，不能只依据“命令已发布”。

### 13.3 手臂测试顺序

1. 不拿物体，确认真实关节名称和限位；
2. 从当前状态平滑进入保守预备位；
3. 单关节、小角度、低速度；
4. 单臂空载手势；
5. 双臂空载手势；
6. 空载预抓取轨迹；
7. 软质测试物、低高度抓取；
8. 正式道具抓取与保持。

每次动作都要比较目标与真实关节反馈。只按时间等待后直接返回成功不构成验收。

---

## 14. 阶段 10：真机物品感知与抓取闭环

当前项目没有真实物品检测器，固定 `object_world_xyz` 只能用于受控标定，不能宣称“识别到物品”。完整链路至少需要：

```text
RGB/深度数据
  → 物品类别与二维区域
  → 相机内参/深度得到三维点
  → camera frame 转 base/map frame
  → 可达性、桌面和碰撞检查
  → 预抓取、接近、闭爪、抬升
  → 手爪状态 + 物品离桌 + 保持验证
```

AimDK 1.0 已移除旧的 RGB-D depth point cloud 话题；实现时应使用当前固件仍提供的 RGB、深度图和相机内参，不能依赖已删除接口。

抓取成功的最低证据：

- 识别类别与需求一致；
- 三维位置时间戳新鲜且在合理工作空间；
- TF 转换成功；
- IK 解存在且所有关节在限位内；
- 手爪状态无故障并出现夹持证据；
- 物品完全离开桌面；
- 播报结束前物品没有掉落。

任一证据缺失，状态机必须失败并安全停止，不得播报成功。

---

## 15. 单任务验收

完整程序联调前，三个任务分别验收。

### 任务 1

- [ ] 启动前已完成重定位。
- [ ] 自主进入交互区 I。
- [ ] 到达后连续稳定停止。
- [ ] 正面朝向交互区 II。
- [ ] 定位失效、越界或有障碍时安全停止。

### 任务 2

- [ ] 正确回答本地时间。
- [ ] 真实相机下识别数字和颜色。
- [ ] 表情请求返回成功且屏幕状态正确。
- [ ] 手势真实完成，并由关节反馈确认。
- [ ] 自研 TTS 通过机器人扬声器播报动作名称。

### 任务 3

- [ ] ASR 正确判断三类需求及否定/纠正语句。
- [ ] 自主进入作业区并安全停止。
- [ ] 识别真实目标而不是使用仿真点云占位信息。
- [ ] 抓取正确物品并完全离桌。
- [ ] 播报内容与实际夹持物一致。
- [ ] 播报结束前物品保持稳定。

分项至少连续通过三次，且失败时行为符合安全要求，才进入全流程测试。

---

## 16. 完整比赛流程门禁

只有以下条件全部满足，才允许操作员执行真机完整节点：

- [ ] B-01 至 B-11 全部关闭并有证据。
- [ ] 目标机器人 AimDK/固件接口已冻结。
- [ ] SLAM 模块已启用，重定位稳定。
- [ ] 真机激光避障和断流停车通过。
- [ ] 相机、ASR、TTS、表情全部通过真实设备测试。
- [ ] 手臂、夹爪和抓取闭环通过。
- [ ] 三个任务分别连续通过。
- [ ] 速度、坐标、边界和姿态均为真机标定值。
- [ ] 安全员、停止机制和日志记录就绪。
- [ ] 竞赛流程只需一次启动，不包含 MuJoCo Reset 或调试输入。

当前 argparse 定义下，真正的真机参数是：

```bash
# [MOTION] 仅在本节全部门禁通过后，才允许由现场操作员执行
python3 competition_node.py --no-sim
```

不得使用：

```bash
python3 competition_node.py
```

因为当前默认值是 `sim=True`，无参数启动仍为仿真模式。正式运行前建议代码再增加醒目的模式打印和人工确认，防止误启动。

---

## 17. 日志与故障报告模板

每次真机测试保存以下信息：

```text
测试编号：
日期时间：
操作员 / 安全员：
机器人型号、固件、AimDK 版本：
项目 commit SHA：
测试阶段和单项：
执行命令：
预期行为：
实际行为：
是否触发运动：
停止方式：
ROS 日志路径：
rosbag 路径（如现场允许）：
定位/关节/手爪关键数据：
结论：通过 / 失败 / 阻塞
下一步：
```

故障分析必须区分：

- 接口不存在；
- 类型或字段不匹配；
- QoS 不匹配；
- 数据没有更新；
- 坐标系或单位错误；
- 控制权冲突；
- 命令已接收但硬件未完成；
- 感知错误导致错误目标；
- 安全条件触发。

不要用“服务调用返回了对象”代替动作成功，也不要用“日志打印成功”代替物理任务完成。

---

## 18. 官方资料

- [AimDK_X2 连接、系统环境与 PC1 禁止事项](https://x2-aimdk.agibot.com/en/latest/quick_start/prerequisites.html)
- [AimDK_X2 SLAM、地图和重定位接口](https://x2-aimdk.agibot.com/en/latest/Interface/perception/SLAM.html)
- [AimDK_X2 末端执行器接口与安全要求](https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/endeffector.html)
- [AimDK_X2 屏幕与表情接口](https://x2-aimdk.agibot.com/en/latest/Interface/interactor/screen.html)
- [AimDK_X2 Python 接口示例](https://x2-aimdk.agibot.com/en/latest/example/Python.html)
- [AimDK_X2 C++ 接口示例及音频播放说明](https://x2-aimdk.agibot.com/en/latest/example/Cpp.html)
- [AimDK_X2 版本更新记录](https://x2-aimdk.agibot.com/en/latest/changelog.html)

---

## 19. 最终原则

真机调试的正确顺序是：

```text
版本冻结
  → 只读接口核对
  → 无运动传感器检查
  → 无运动交互检查
  → 重定位和坐标标定
  → 最小运动
  → 低速导航与停车
  → 空载手臂和夹爪
  → 物品感知与抓取闭环
  → 单任务验收
  → 完整比赛流程
```

任何阶段失败，都退回本阶段分析，不通过修改仿真模型来假设真机行为，也不在未验证时直接运行下一阶段。
