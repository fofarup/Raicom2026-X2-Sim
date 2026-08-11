# X2 真机独立目录

这里是一套可单独传输的真机程序。`app/`包含独立Task1/2/3、语音、视觉、音频和
IK SDK，不引用上一级仿真代码。公开仓库不分发 ASR 权重；部署前按
`MIGRATION_GUIDE.md` 准备 SenseVoice 模型和私有配置。

## 真机到手后的第一步

1. 运行`./create_config.sh`生成私有`app/config/real_robot.json`；
2. 根据真机 ROS 图和比赛场地完成所有 `null` 项；
3. 执行只读检查：

```bash
./preflight.sh
```

配置存在`null`、字段缺失、话题类型或路径错误时检查会失败，比赛脚本不得启动。
文件作用和完整现场标定顺序见`FILES_AND_CALIBRATION.md`。

代码不需要逐个复制：联网使用`deploy.sh`增量同步；没有SSH时使用
`build_bundle.sh`生成一个真机压缩包。两种方式都排除仿真文件且不覆盖现场配置。

检查通过后，真机统一从独立入口启动，例如：

```bash
./run.sh 1
./run.sh 2
./run.sh 3 --navigate-and-align
```

二号场地必须先完成参数化重定位。只读检查不会发布任何ROS消息：

```bash
./relocate.sh check --map-id 1786066723179
```

确认机器人位于输入的像素位置和朝向、有人持急停后，使用二号场地安全入口。下面的
`YAW_DEG`必须由现场核对，不能照抄示例值：

```bash
export RAICOM_CONFIRM_REAL_ROBOT=YES
export RAICOM_CONFIRM_RELOCALIZATION=YES
./site2_run.sh --pixel-x 465 --pixel-y 200 --yaw-deg YAW_DEG --confirm-at-pose
```

该入口只有收到连续有效的`/slam/lidar_odom`后才会启动语音总控。机器人不在建图
原点时不得使用`(465, 200)`；应输入现场测得的像素位置并显式添加
`--allow-non-origin`。禁止直接运行AimDK自带的`py_examples relocate`，因为机器人
当前安装的示例写死了另一张地图的参数。

完整真机语音入口：

```bash
./voice_run.sh
```

它直接使用X2 VAD、同目录SenseVoice、语义分类、本地任务进程和PCM扬声器，
不依赖电脑麦克风、Docker或上一级仿真程序。

`run.sh` 会强制选择真机配置，并从配置中设置 ROS Domain 和 RMW。

## 自研语音

正式代码不使用 `PlayTts`。首次切换官方 `only_voice`：

```bash
export RAICOM_CONFIRM_REAL_ROBOT=YES
./setup_only_voice.sh only_voice
./voice_preflight.sh
```

该脚本只重启 `agent`，不会重启 `mc`。恢复原生交互时执行
`./setup_only_voice.sh normal`。完整验证顺序见`../docs/AUDIO_VALIDATION.md`。

## 安全边界

- 不得把 `simulation.json` 的转向漂移、机械臂容差直接复制到真机；
- `app/real_grasp_agent.py`只做官方RGB-D/OmniPicker接口预检和受保护夹爪测试；
- 真机首次行走、转向和抬臂必须分别低速标定，并有人处于急停位置；
- `app/config/real_robot.json`是现场机器参数，不提交Git，模板可以提交；
- 完成真机视觉、外参、OmniPicker和机械臂标定前，语音需求只导航到作业区并
  明确拒绝抬臂，不会调用仿真mug真值或仿真轨迹。
