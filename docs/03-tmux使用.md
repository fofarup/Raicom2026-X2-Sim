# tmux 使用方法

## 为什么使用 tmux

仿真至少包含三个长期运行的终端：

1. MuJoCo 仿真；
2. MC 运动控制模块；
3. Python 控制命令。

`tmux` 可以把它们放进同一个终端会话，即使误关终端窗口，进程也可以继续运行。

## 项目提供的一键入口

```bash
cd ~/x2_ws/x2_biao
./scripts/tmux/start.sh
```

脚本创建 `raicom-x2` 会话以及 `sim`、`mc`、`control` 三个窗口。

停止全部组件：

```bash
./scripts/tmux/stop.sh
```

## 常用快捷键

tmux 默认前缀键为 `Ctrl+b`。先按下并松开 `Ctrl+b`，再按后续按键。

| 操作 | 快捷键 |
| --- | --- |
| 新建窗口 | `Ctrl+b`，然后 `c` |
| 下一个窗口 | `Ctrl+b`，然后 `n` |
| 上一个窗口 | `Ctrl+b`，然后 `p` |
| 按编号切换窗口 | `Ctrl+b`，然后 `0`～`9` |
| 显示窗口列表 | `Ctrl+b`，然后 `w` |
| 重命名窗口 | `Ctrl+b`，然后 `,` |
| 垂直分屏 | `Ctrl+b`，然后 `%` |
| 水平分屏 | `Ctrl+b`，然后 `"` |
| 切换面板 | `Ctrl+b`，然后方向键 |
| 关闭当前面板 | 输入 `exit` |
| 暂离会话 | `Ctrl+b`，然后 `d` |
| 进入复制/滚动模式 | `Ctrl+b`，然后 `[` |
| 退出复制模式 | `q` |

重新连接项目会话：

```bash
tmux attach -t raicom-x2
```

查看会话：

```bash
tmux ls
```
