# X2官方接口核对记录（2026-08-09）

仅依据智元X2 AimDK官方文档和Python示例：

- 接口总览：<https://x2-aimdk.agibot.com/zh-cn/latest/Interface/index.html>
- Python示例：<https://x2-aimdk.agibot.com/zh-cn/latest/example/Python.html>
- 运动模式：<https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/modeswitch.html>
- 走跑控制：<https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/locomotion.html>
- 末端执行器：<https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/endeffector.html>
- 传感器：<https://x2-aimdk.agibot.com/zh-cn/latest/Interface/hal/sensor.html>
- 音频：<https://x2-aimdk.agibot.com/zh-cn/latest/Interface/interactor/voice.html>

## 已写入独立真机模板的官方默认接口

| 功能 | 官方接口/类型 |
|---|---|
| 速度 | `/aima/mc/locomotion/velocity` / `McLocomotionVelocity` |
| 输入源 | `/aimdk_5Fmsgs/srv/SetMcInputSource` |
| 模式 | `/aimdk_5Fmsgs/srv/SetMcAction` |
| 预设动作 | `/aimdk_5Fmsgs/srv/SetMcPresetMotion` |
| 表情 | `/aimdk_5Fmsgs/srv/PlayEmoji` |
| RGB | `/aima/hal/sensor/rgbd_head_front/rgb_image` / `Image` |
| 深度 | `/aima/hal/sensor/rgbd_head_front/depth_image` / `Image` |
| RGB内参 | `/aima/hal/sensor/rgbd_head_front/rgb_camera_info` / `CameraInfo` |
| 深度内参 | `/aima/hal/sensor/rgbd_head_front/depth_camera_info` / `CameraInfo` |
| VAD音频 | `/agent/process_audio_output` / `ProcessedAudioOutput` |
| PCM播放 | `/aima/hal/audio/playback` / `AudioPlayback` |
| 手命令 | `/aima/hal/joint/hand/command` / `HandCommandArray` |
| 手状态 | `/aima/hal/joint/hand/state` / `HandStateArray` |

官方重定位示例从`/slam/lidar_odom`读取`nav_msgs/msg/Odometry`，因此独立真机
Task1支持配置定位消息类型，不再固定假设`PoseStamped`。

## 重要安全限制

官方末端执行器文档明确要求：开发者直接控制手部前，应确保机器人安全并停止
PC1原生MC；如果仍需原生MC保持走跑管理，需要联系技术支持进行系统侧适配。
因此当前`real_grasp_agent.py`默认只读；即使调用夹爪测试，也必须显式设置
`RAICOM_CONFIRM_MC_STOPPED=YES`。在组委会确认比赛允许的MC/手部并行控制方案前，
不得把该接口接入完整自主抓取。

所有默认接口仍须在实际机器人上用`ros2 topic type`、`ros2 service type`和
`preflight.sh`复核，固件版本差异以现场实际ROS图为准。
