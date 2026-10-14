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
# 只规划：Z=15 cm、长度 10 cm，起点/中点/终点三个目标
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --points 3

# 只求解和模型检查，不移动
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --points 3 --solve

# 执行：每点反馈补偿；关节误差≤2°、模型末端误差≤10 mm，连续两次满足后进入下一点
# 三个大目标之间仍按每次最多 0.5° 发送；不要求中途为完美直线
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --points 3 --joint-tolerance 2 --xyz-tolerance-mm 10 --execute

# 五个运动关节回零后，重走已保存的三点路径；夹爪保持当前位置
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --replay 垃圾文件夹/plans/line-20261014-165413-100878/planning.json --return-home --execute

# 原 10 cm 线的起终点之间插入 5 点（共 7 点），画完后末端上抬 3 cm
# 前五轴先回零，夹爪保持；若路径中途失败，不继续执行后续目标
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --replay 垃圾文件夹/plans/line-20261014-170434-683856/planning.json --return-home --insert-points 5 --lift-cm 3 --execute

# 每次运行单独记录：垃圾文件夹/plans/line-时间/ 下的 planning.json、execution.json、actual_feedback.jsonl 和 actual_feedback.csv
# 补偿超限、反馈不变、误差增大、超时及模型碰撞检查仍会停止
```

```bash
# 预览底座反馈实验：目标每次调整 1°，实际角度达到 45°±0.5° 连续三次后停止
./servo base-step

# 执行：仅写 ID1，每步等待 0.5 秒；带超时、停滞和最多 5° 补偿保护
./servo base-step --execute
```

```bash
# 预览：肩部 ID2 相对增加 1°，其余关节目标保持当前角度
./servo direct --ids 2 --delta 1

# 执行：一次调用原版 /joints/write，不做 IK 或速度规划；1 秒后读取实际角度
# 每执行一次，都从当时位置再增加 1°；保持 Phosphobot 已连接
./servo direct --ids 2 --delta 1 --execute

# 反方向移动 1°
./servo direct --ids 2 --delta -1 --execute

# 指定肩部绝对角度：以下只预览
./servo direct --ids 2 --angles 0

# 等待 2 秒后读取反馈（等待不代表一定到位）
./servo direct --ids 2 --delta 1 --wait 2 --execute
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
# 预览：六轴回标定零位，包含夹爪；原生初始化后增加真实反馈补偿
./servo init

# 执行：每次最多补偿 1°，补偿上限 ±15°，六轴真实误差≤0.5° 连续三次才完成
# 超时、停滞、误差增大或补偿越界会停止；日志保存在垃圾文件夹/logs/
./servo init --execute

# 指定误差、补偿范围和总超时
./servo init --tolerance 0.5 --max-compensation 15 --timeout 60 --execute

# 对照：仅执行原生初始化，不做补偿，不验证实际到位
./servo init --native --execute
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
