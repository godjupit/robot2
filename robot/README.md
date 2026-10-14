```bash
cd /Users/wangqibo/cs3.1/multimedia/robot2/robot
```

```bash
# 读取当前六轴角度
./servo read

# 读取当前模型末端 XYZ，单位 cm，以模型底座为参考（未计入笔尖偏移）
./servo xyz

# 读取指定关节的当前角度
./servo read --ids 1 2
```

```bash
# 预览：底座相对当前角度转动 2°
./servo move --ids 1 --delta 2 --speed 5 --acc 10

# 执行：底座相对当前角度转动 2°
./servo move --ids 1 --delta 2 --speed 5 --acc 10 --execute
```

```bash
# 执行：底座移动到绝对角度 0°
./servo move --ids 1 --angles 0 --speed 5 --acc 10 --execute
```

```bash
# 预览：六轴分别移动到指定角度
./servo move --ids 1 2 3 4 5 6 --angles 0 4 2 0 0 115 --speed 5 --acc 10
```

```bash
# 预览：Phosphobot 原生初始化，六轴回标定零位，包含夹爪
./servo init

# 执行：调用 /move/init，不经过脚本的角度插值规划
./servo init --execute
```

```bash
# 执行移动，并指定日志文件
./servo move --ids 1 --delta 2 --speed 5 --acc 10 --log-file logs/my-move.json --execute
```

```bash
# 开启整台机器人六轴力矩
./servo torque on --execute

# 托住机械臂后，关闭整台机器人六轴力矩
./servo torque off --execute
```

```bash
# 指定 Phosphobot 服务地址，读取当前角度
./servo --url http://127.0.0.1:80 --robot-id 0 read

# 查看命令参数
./servo --help
./servo move --help
```
