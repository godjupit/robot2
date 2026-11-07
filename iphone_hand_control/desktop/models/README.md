# SO100 模型资源

`so-100.urdf` 描述运动链及关节限位；`so-100/meshes/` 包含此 URDF 引用的全部 7 个 STL 网格，供模型碰撞检查使用。

网格原样复制自本项目原有的 `phosphobot-main/phosphobot/resources/urdf/so-100/meshes/`。未修改几何、尺度或 URDF 中的模型偏移。加载器将 `package://` 后的路径相对于本目录解析，因此无需保留上游源码目录。

`mesh-manifest.json` 记录各网格的 SHA-256；`PHOSPHOBOT-LICENSE` 保留来源仓库的许可证。上游项目：https://github.com/phospho-app/phosphobot 。

这些网格仅描述机器人本体，不包含外加笔杆和外部环境。
