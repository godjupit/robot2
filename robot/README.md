```bash
cd /Users/wangqibo/cs3.1/multimedia/robot2/robot
```

在上次已完成的 5 cm 竖线上补三横：`--reference-line` 直接继承该竖线规划记录的中心、高度、笔长、补偿步长、到位误差、纸面保护、抬笔和回零设置；`--strokes 1 2 4` 不重画竖。三横沿模型 Y 方向，上/下横长 6 cm，中横长 4.2 cm，X 分别为 29.5、27、24.5 cm。每横结束抬到 Z=5 cm，下一横从上方落笔。

在上述命令追加 `--offset-x-cm 5`，三横整体沿模型 X 远离底座 5 cm，新中心 X=32 cm，上/中/下横 X=34.5、32、29.5 cm。参数在继承 reference-line 后再应用，不改变笔长、保护或补偿配置；旧竖线不会重画。

改变笔长时显式加 `--rod-length-cm 15.2`（数值应由实际安装决定），此参数优先于 reference-line 里的旧笔长；未指定才继承旧值。加 `--no-return-home` 可取消参考记录中的回零动作，从当前实测姿态先抬笔、再转移。回零是关节运动，不是垂直抬笔，可能扫过纸面；位置模型预检不能证明真实笔尖或笔杆不接触纸。

```bash
# 不加 --execute 只做完整预检；该记录笔长 15 cm、各轴步长 0.8°、XYZ 10 mm、Z保护 5 mm
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python run_wang.py --reference-line 垃圾文件夹/plans/line-20261014-213251-913171/planning.json --strokes 1 2 4 --width-cm 6 --execute
```

绘制“王”：`run_wang.py` 默认仅做完整预检，`--execute` 才移动。从当前实测姿态开始，模型 X 为字的上下方向、Y 为横线方向，中心 `(27, 0.199)` cm，高 16 cm、宽 10 cm，笔顺为横、横、竖、横。默认模型笔长 16 cm，纸面 Z=0，每笔上抬 5 cm。

单独排查竖线：加 `--stroke 3` 只保留王字第三笔。当前 8×6 cm、中心 `(37, 0.199)` 的竖线为 `(41, 0.199, 0)` 到 `(33, 0.199, 0)` cm，先抬笔转移到起点上方，再落笔、画竖、抬笔；其余横线不会执行。

复用最后成功 line 参数的 5 cm 竖线：中心 `(27, 0.199)`，起点 `(29.5, 0.199, 0)`，终点 `(24.5, 0.199, 0)`，6 个绘图点。`--joint-step-deg 0.5` 对本次运行统一覆盖五轴补偿上限，包括肩和肘；其余运行的肩/肘默认 1°不变。

```bash
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --rod-length-cm 16 --replay 垃圾文件夹/plans/vertical-5cm-source.json --return-home --spacing 1 --lift-cm 5 --xyz-tolerance-mm 10 --paper-z-cm 0 --z-below-tolerance-mm 1 --joint-step-deg 0.5 --solve
```

```bash
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python run_wang.py --stroke 3 --rod-length-cm 16 --center-x 37 --center-y 0.199 --height-cm 8 --width-cm 6 --paper-z-cm 0 --lift-cm 5 --spacing-mm 10 --xyz-tolerance-mm 10 --z-below-tolerance-mm 5
```

```bash
# 预检与预览
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python run_wang.py
# 真机：每笔落笔、画线、抬笔后转移；最后也抬笔
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python run_wang.py --rod-length-cm 16 --center-x 27 --height-cm 16 --width-cm 10 --paper-z-cm 0 --lift-cm 5 --spacing-mm 10 --xyz-tolerance-mm 10 --execute
```

初始低于转移高度时先垂直抬高；已在高处时保持现有高度转移，到达起点正上方后固定 XY 逐点下降，转移时不同时下降。落笔和抬笔以最大 5 mm 间距规划，绘图与转移点距默认 10 mm。TCP 下压与绘图均使用关节顺序筛选及纸面下方 5 mm 的停止保护；转移阶段反馈高度需不低于纸面上方 1 cm，未达到抬笔高度或发生保护停止时，不继续下一笔。夹爪保持不写入。默认整字总超时 600 秒，日志保存在 `垃圾文件夹/plans/wang-时间/`。纸面/TCP 仍须实测校准，模型预检不证明实体笔尖或整个笔杆有足够间隙。

Position-only TCP IK：棍尖从 `Fixed_Jaw` 坐标原点沿夹爪向前（局部 -Y）延伸约 17 cm，使用 `--rod-length-cm 17`，等价于 `--tcp-offset-cm 0 -17 0`。参数表示末端原点到尖端的距离，不是棍子总长。

```bash
# 虚拟棍尖在 Z=0 绘制 10 cm 直线，点距 10 mm（共 11 点），最后上抬 5 cm；只预检
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --rod-length-cm 17 --replay 垃圾文件夹/plans/line-tcp-z0-source.json --spacing 1 --lift-cm 5 --paper-z-cm 0 --z-below-tolerance-mm 5 --solve
```

加入 TCP 参数后，规划 XYZ、IK、实测关节角计算出的 XYZ、结束上抬都以棍尖为参考。工具朝向自由，仅用很弱的关节连续性正则化选择邻近解。绘图/接近/上抬只要求 TCP XYZ 误差≤10 mm 连续两次；回零仍同时检查关节误差≤2°。夹爪不写入。约 17 cm 是估计值，日志位置是模型计算值，尚未直接测量尖端。

原机器人网格碰撞、限位、关节跳变、停滞、补偿范围与超时保护保留；棍子实体及外部障碍物尚未纳入碰撞网格。旧路径坐标在 TCP 模式下解释为尖端目标，需要重新预检。未传 TCP 参数的下列旧命令仍以夹爪末端原点为参考。

TCP 接近起点时以最大 1 cm 的间距细分路径，避免长工具导致单个 IK 目标关节跳变过大。Z=0 是 URDF 模型底座平面，需要确认与实际纸面的位置关系。结束上抬 5 cm 后虚拟尖端位于 Z=5 cm，不能把上抬后的姿态当成 Z=0 接触姿态来调笔长。工具方向自由，调长度会沿当时的工具方向改变 XYZ，不一定只改变高度。

TCP 绘图段独立检查纸面高度：`--paper-z-cm` 默认取路径高度，`--z-below-tolerance-mm` 默认 5 mm。记录反馈后、到位判断和继续发送补偿前，若 TCP 低于允许高度就停止整个执行，沿用异常处理请求保持当前位置并保存日志，不自动抬笔或继续下一点。此保护只作用于 draw，不作用于 home/approach/lift；约 0.25 秒外加通信/检查耗时的反馈间隔无法保证停止前没有越界。`--spacing 1` 单位为 cm，对 replay 也重新采样直线；与 `--insert-points` 不能同时使用。XYZ 到位容差仍为 10 mm。

绘图补偿每轮只改变一个关节的发送目标。根据当前实测姿态、未到位的旧发送目标，采样各轴到达顺序的端点组合及同步插值姿态，拒绝预测 TCP 低于配置保护阈值的候选，预测下限与实测停止下限统一为 `paper-z-cm - z-below-tolerance-mm / 10` cm。候选优先改善 XYZ，距纸面不足 2 mm 时优先增加 Z 余量。肩关节 ID2 和肘关节 ID3 的步长依次尝试 1°、0.5°、0.25°，其余轴依次尝试 0.5°、0.25°、0.125°。没有合适候选就停止轨迹，记录原因。接近、回零和上抬仍使用原有补偿。单轴目标调整不代表其余关节立刻停止，有限模型预测也不保证真机不下扎，实测 Z 停止保护仍保留。选择的关节和预测高度会打印，并存入 execution.json 的 samples/commands。

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
# 三个大目标之间仍按肩和肘每次最多 1°、其余轴每次最多 0.5° 发送；不要求中途为完美直线
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --points 3 --joint-tolerance 2 --xyz-tolerance-mm 10 --execute

# 五个运动关节回零后，重走已保存的三点路径；夹爪保持当前位置
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --replay 垃圾文件夹/plans/line-20261014-165413-100878/planning.json --return-home --execute

# 原 10 cm 线的起终点之间插入 5 点（共 7 点），画完后末端上抬 3 cm
# 前五轴先回零，夹爪保持；若路径中途失败，不继续执行后续目标
/Users/wangqibo/cs3.1/multimedia/robot/.venv/bin/python plan.py --replay 垃圾文件夹/plans/line-20261014-170434-683856/planning.json --return-home --insert-points 5 --lift-cm 3 --execute

# 每次运行单独记录：垃圾文件夹/plans/line-时间/ 下的 planning.json、execution.json、actual_feedback.jsonl 和 actual_feedback.csv
# 补偿超限、反馈不变、超时及模型碰撞检查仍会停止
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
