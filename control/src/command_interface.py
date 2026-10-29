# SPDX-License-Identifier: GPL-3.0-or-later
"""运行中的 NCHex 目标点终端指令接口。"""

from __future__ import annotations

import math
import threading
import numpy as np


HELP_TEXT = """可用指令：
  forward [m] / 前进 [m]        沿机头方向前进，默认 0.2 m
  back [m] / 后退 [m]           沿机尾方向后退
  left [m] / 左移 [m]           沿机体左侧移动
  right [m] / 右移 [m]          沿机体右侧移动
  up [m] / 上升 [m]             增加高度
  down [m] / 下降 [m]           降低高度
  move dx dy dz / 移动 dx dy dz  按世界坐标增量移动
  target x y z [yaw_deg]         设置世界坐标绝对目标
  yaw deg / 偏航 deg             设置绝对偏航角
  turn deg / 转向 deg            增加相对偏航角
  status / 状态                  查看当前目标
  help / 帮助                    显示帮助
  quit / 退出                    关闭仿真
"""


class TargetCommandState:
    """带锁的目标位置/偏航角以及文本命令解析器。"""

    def __init__(self, position_world: np.ndarray, yaw_rad: float):
        self._position = np.asarray(position_world, dtype=float).copy()
        self._yaw_rad = float(yaw_rad)
        self._lock = threading.Lock()

    def snapshot(self) -> tuple[np.ndarray, float]:
        with self._lock:
            return self._position.copy(), self._yaw_rad

    def status_text(self) -> str:
        position, yaw_rad = self.snapshot()
        return (
            f"目标位置 x={position[0]:.3f}, y={position[1]:.3f}, "
            f"z={position[2]:.3f} m，偏航={math.degrees(yaw_rad):.1f}°"
        )

    @staticmethod
    def _distance(arguments: list[str]) -> float:
        if len(arguments) > 1:
            raise ValueError("该指令最多只能有一个距离参数")
        distance = 0.2 if not arguments else float(arguments[0])
        if distance < 0.0:
            raise ValueError("距离必须大于或等于 0")
        return distance

    def execute(self, text: str) -> tuple[str, bool]:
        """执行一行命令，返回（提示文本，是否退出）。"""

        fields = text.strip().split()
        if not fields:
            return "", False
        command = fields[0].lower()
        arguments = fields[1:]

        if command in {"help", "帮助", "?"}:
            return HELP_TEXT.rstrip(), False
        if command in {"quit", "exit", "退出"}:
            return "正在关闭仿真……", True
        if command in {"status", "状态"}:
            return self.status_text(), False

        with self._lock:
            if command in {"forward", "前进", "back", "backward", "后退"}:
                distance = self._distance(arguments)
                if command in {"back", "backward", "后退"}:
                    distance = -distance
                self._position[0] += distance * math.cos(self._yaw_rad)
                self._position[1] += distance * math.sin(self._yaw_rad)
            elif command in {"left", "左移", "right", "右移"}:
                distance = self._distance(arguments)
                if command in {"right", "右移"}:
                    distance = -distance
                self._position[0] += -distance * math.sin(self._yaw_rad)
                self._position[1] += distance * math.cos(self._yaw_rad)
            elif command in {"up", "上升", "down", "下降"}:
                distance = self._distance(arguments)
                self._position[2] += distance if command in {"up", "上升"} else -distance
                self._position[2] = max(0.05, self._position[2])
            elif command in {"move", "移动"}:
                if len(arguments) != 3:
                    raise ValueError("用法: move dx dy dz")
                self._position += np.asarray([float(value) for value in arguments])
                self._position[2] = max(0.05, self._position[2])
            elif command in {"target", "目标"}:
                if len(arguments) not in {3, 4}:
                    raise ValueError("用法: target x y z [yaw_deg]")
                self._position[:] = [float(value) for value in arguments[:3]]
                self._position[2] = max(0.05, self._position[2])
                if len(arguments) == 4:
                    self._yaw_rad = math.radians(float(arguments[3]))
            elif command in {"yaw", "偏航"}:
                if len(arguments) != 1:
                    raise ValueError("用法: yaw deg")
                self._yaw_rad = math.radians(float(arguments[0]))
            elif command in {"turn", "转向"}:
                if len(arguments) != 1:
                    raise ValueError("用法: turn deg")
                self._yaw_rad += math.radians(float(arguments[0]))
            else:
                raise ValueError(f"未知指令: {fields[0]}；输入 help 查看帮助")

        return self.status_text(), False
