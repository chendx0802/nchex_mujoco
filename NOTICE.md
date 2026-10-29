# 来源与许可说明

`control/` 中标记 `SPDX-License-Identifier: GPL-3.0-or-later` 的控制代码，
由 ArduPilot NCHex 定制实现及公共控制器提炼而来，按 GPL-3.0-or-later 处理。
完整 GPLv3 文本保存在 [COPYING](COPYING)。算法来源文件见
[control/README.md](control/README.md)。本仓库是仿真适配与控制算法提炼版本，
不代表 ArduPilot 官方发布或完整飞控固件。

模型与网格来自原始 `nchex` ROS/Gazebo 工程。该工程的 `package.xml`
声明 `BSD-3-Clause AND MIT`。现有模块未携带逐个网格的作者及完整上游许可文本，
因此本文件仅记录已知来源，不将 GPL 控制代码的许可自动扩大到所有模型资源。
模型资源的完整来源、版权声明和逐文件许可信息仍有待补齐。

发布整理新增的启动辅助脚本及 `examples/` 示例按 GPL-3.0-or-later 提供。
