# Raicom2026 X2 比赛地图仿真

本私有仓库用于在 Ubuntu 22.04（x86_64）上，通过 Docker 从零搭建睿抗
2026 智慧养老赛项的 X2 仿真环境。

第一阶段验收目标：

> 加载官方比赛地图与 X2 模型，使机器人稳定站立，然后以低速向前移动并安全停止。

## 固定约定

| 项目 | 固定值 |
| --- | --- |
| Ubuntu | 22.04 LTS，x86_64 |
| 桌面会话 | X11（Ubuntu on Xorg） |
| 本地目录 | `~/x2_ws/x2_biao` |
| ROS 2 | Humble（安装在 Docker 镜像中） |
| ROS_DOMAIN_ID | `26` |
| Docker 容器 | `raicom2026-x2-sim` |
| tmux 会话 | `raicom-x2` |

## 从零开始

完整教程见：[docs/01-从零安装.md](docs/01-从零安装.md)。

首次实际运行见：[docs/02-首次启动与控制.md](docs/02-首次启动与控制.md)。

tmux 快捷键见：[docs/03-tmux使用.md](docs/03-tmux使用.md)。

故障处理见：[docs/04-故障排查.md](docs/04-故障排查.md)。

## 启动顺序

```text
Git LFS 下载官方压缩包
        ↓
bootstrap_assets.sh 原样解压并保留 Linux 符号链接
        ↓
构建 Ubuntu 22.04 + ROS 2 Humble Docker 镜像
        ↓
构建 aimdk_msgs
        ↓
启动 MuJoCo
        ↓
启动 MC
        ↓
SD 稳定站立 → MuJoCo Reset → LD 运动模式
        ↓
低速前进 2 秒 → 连续零速度停止
```

## 重要说明

- 官方例程保存在压缩包中，不做修改。
- 本仓库新增的 `control/safe_forward.py` 只用于仿真测试。
- 未经重新安全评估，不要直接把该测试脚本用于真机。
- 官方资源许可尚未确认，因此仓库必须保持私有，详见 [NOTICE.md](NOTICE.md)。
