# map_editer_copy

`map_editer_copy` 是一个基于 ROS2 与 C++ 的地图编辑 / 路径规划相关项目，主要包含地形地图、路径运动规划、自动启动配置以及调试工具等模块。  
项目整体采用 `ament_cmake` 构建方式，适合在 Linux 环境下进行开发、编译与调试。

## 项目简介

本项目围绕地图编辑与机器人运动规划展开，包含以下核心模块：

- **altitude_map**：高程地图相关功能模块
- **path_motion_planning**：路径规划与运动控制核心模块
- **auto_start**：启动与运行配置模块，包含 launch、rviz 和 config 资源
- **debug_tools**：调试辅助工具
- **rm_interface**：接口相关模块

从当前仓库结构来看，该项目更偏向于机器人地图处理、轨迹规划与系统集成场景。

---

## 功能特性

- 基于 ROS2 架构开发
- 使用 C++ 实现核心逻辑
- 支持地图相关处理与路径规划
- 提供多个 ROS2 节点可执行程序
- 支持 RViz 可视化配置
- 包含自动启动与运行配置资源
- 便于后续扩展更多地图编辑功能

---

## 目录结构

```text
map_editer_copy/
├── src/
│   ├── altitude_map/          # 高程地图模块
│   ├── auto_start/            # 启动、配置、RViz 资源
│   ├── debug_tools/           # 调试工具
│   ├── path_motion_planning/  # 路径规划与运动控制
│   └── rm_interface/          # 接口相关模块
├── .vscode/                   # VS Code 配置
├── .gitignore
└── README.md
```
