# SO100 绘图

主要程序位于 [`robot/`](robot/README.md)，包含 TCP 位置 IK、路径规划、模型碰撞检查及真实关节反馈补偿。模型和网格已放在 `robot/models/`，无需上游源码目录。

## 启动控制服务

连接机械臂电源和 USB，在单独的终端启动本机已安装的 Phosphobot：

```bash
phosphobot run --no-cameras --no-realsense
```

保持此终端运行，在浏览器打开 http://127.0.0.1:80 ，确认 SO100 已连接并完成标定。如果服务输出了其他端口，以输出为准，并给绘图命令增加 `--url http://127.0.0.1:端口`。

另开终端运行绘图程序，命令见 [`robot/README.md`](robot/README.md)。默认仅规划预检，加 `--execute` 才发送运动。

