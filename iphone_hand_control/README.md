# iPhone 前置深度相机 → SO100 手部位置控制

独立原型，适用于具备 TrueDepth 的 iPhone。仅控制机械臂前五个运动关节，不写夹爪。iPhone 识别手掌位置，经局域网发给电脑；电脑计算末端目标和逆运动学。无需上架 App Store。

## 目录

- `ios/HandControl/HandControlApp.swift`：SwiftUI 界面、Vision 手部识别、同步彩色/深度采集及网络发送。
- `ios/project.yml`：XcodeGen 项目定义。
- `desktop/server.py`：默认只预览；显式 `--execute` 才连接 Phosphobot 并发送控制。
- `desktop/robot_model.py`、`desktop/models/`：从本仓库现有实现复制的独立 SO100 模型、反馈读取和碰撞检查，运行不导入旁边的 `robot/`。模型许可证随资源保留。

## 1. 电脑先运行预览

在 Mac 终端执行，不需要连接机械臂，也不需要第三方 Python 库：

```bash
cd /Users/wangqibo/cs3.1/multimedia/robot2/iphone_hand_control
python3 desktop/server.py
```

终端显示端口和随机 Token。手机与电脑连接同一可信局域网，在手机 App 输入 `http://电脑局域网IP:8765` 和该 Token。每次重启服务，Token 会变化；也可以使用 `--token 自己设置的长字符串`。仅在可信局域网使用，原型传输为 HTTP，不应映射到公网。只使用一个手机控制端。

## 2. 将 App 装到 iPhone

需要完整 Xcode、与 iPhone 系统版本兼容的 iOS SDK，以及 Apple ID 签名。免费个人开发团队可尝试真机安装，具体签名有效期和设备限制以 Xcode 提示为准；无需 App Store 发布。

安装 Xcode 后打开一次，完成组件安装，并在 Xcode → Settings → Locations 选择命令行工具。安装 XcodeGen 后生成项目：

```bash
brew install xcodegen
cd /Users/wangqibo/cs3.1/multimedia/robot2/iphone_hand_control/ios
xcodegen generate
open HandControl.xcodeproj
```

在 Signing & Capabilities 选择自己的 Team，必要时修改 Bundle Identifier。用线连接 iPhone，在 Xcode 选择该手机并运行；按手机提示开启开发者模式、允许相机和局域网访问。

固定手机，按 App 支持方向横屏，手掌朝前置相机，先尝试约 30–50 cm 的距离。该范围是起测建议，实际深度是否有效以现场输出为准。点“开始跟随”，屏幕显示电脑返回的目标坐标和 `preview` 模式。当前界面显示数值，没有相机预览图。

## 3. 控制规则

第一帧有效手掌位置是手部原点，机械臂当时的位置是末端原点。默认比例 0.4，即手移动 10 cm，目标移动 4 cm：

| 手相对手机移动 | 模型末端目标 |
| --- | --- |
| 远离相机 | X 增大 |
| 向原始图像右侧 | Y 减小 |
| 向上 | Z 增大 |

左右取未镜像相机坐标，真机前先通过预览核对方向。手腕及四个掌指关节均需足够置信度；取掌心邻域有效深度中值，使用相机内参反投影为米。未使用捏合或夹爪手势。

暂停、丢手或断流超过 0.3 秒后清除原点；再次获得有效手部数据时以当前位置重新定零。退出前台会关闭跟随。工作区越界或控制异常会锁定停止，需要重启电脑端。

## 4. 真机接入（预览通过后）

安装独立 Python 环境：

```bash
cd /Users/wangqibo/cs3.1/multimedia/robot2/iphone_hand_control
python3 -m venv .venv
.venv/bin/pip install -r desktop/requirements.txt
```

在另一个终端运行本机已安装的 Phosphobot：

```bash
phosphobot run --no-cameras --no-realsense
```

确认服务中的实际设备是已标定 SO100，并移除绘图长杆等额外工具。此程序模型参考点为 Fixed_Jaw 原点，不继承旧绘图程序的笔长。确认机械臂周围无障碍物、起始位置处于空中，再显式启动：

```bash
.venv/bin/python desktop/server.py --execute --scale 0.4
```

默认 Phosphobot 地址 `http://127.0.0.1:80`，可用 `--robot-url` 修改。发送 `/joints/write` 时仅传前五轴；需要现场确认安装的 Phosphobot 版本支持前缀关节写入。尚未完成该接口的真机兼容验证。

程序限制相对目标范围为 8 cm、目标模型高度至少 5 cm，每轮目标位移至多 2 mm、单关节变化至多 0.5°，名义周期 0.1 秒；模型检查包含限位、自碰撞和底座平面间隙。IK/网络耗时会降低实际更新率，尚无实测延迟。前一目标与反馈偏差超过 8°即停止，不做绘图程序的持续误差补偿。

暂停/异常时尝试读取实际角度并发送保持位置；网络或设备故障时无法保证此请求成功。停止不是断电急停。模型没有周围障碍物，也没有额外工具；有限姿态采样不能证明所有连续运动都无碰撞。

## 验证状态

已完成：Python 协议与状态测试（非法深度、轴映射、断流重新定零、越界锁定停止）；独立模型加载；HTTP 接收链路本机测试。

```bash
python3 -m unittest discover -s desktop -v
```

未完成：iOS SDK 编译、iPhone 17 实测深度与坐标对齐、真机前五轴写入兼容性、实际机械臂跟随及端到端延迟。本机当前选中 CommandLineTools，Swift 检查还遇到 `SwiftBridging` 模块重复定义，因此不能把源码交付视为手机端已运行。

实现参考 Apple 文档：
- https://developer.apple.com/documentation/avfoundation/avcapturedepthdataoutput
- https://developer.apple.com/documentation/vision/vndetecthumanhandposerequest
