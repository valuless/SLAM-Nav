#!/usr/bin/env python3
"""
键盘速度控制 — 发布 Twist 到 dobot_atom_bridge 节点

W/S: 前进/后退
A/D: 左移/右移
Q/E: 逆时针/顺时针旋转
空格: 急停
R/F: 增加/减少速度档位
Ctrl+C: 退出
"""

import sys
import termios
import tty
import select
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


BANNER = """
========================================
  键盘速度控制 (dobot_atom_bridge)
========================================
  W/S     前进/后退
  A/D     左移/右移
  Q/E     逆时针/顺时针旋转
  空格     急停 (速度归零)
  R/F     增加/减少速度档位
  Ctrl+C  退出
========================================
"""


class _Terminal:
    """终端 raw 模式管理，初始化时进入 raw 模式，退出时恢复"""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get_key(self, timeout: float = 0.05) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if ready else ""


class KeyboardVelPublisher(Node):
    def __init__(self, topic: str = "/atom_bridge_node/cmd_vel"):
        super().__init__("keyboard_vel_pub")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.speed_level = 0  # 速度档位
        self.speeds = [0.05, 0.1, 0.2, 0.4, 0.8]  # m/s 或 rad/s

        # 定时发布（即使没按键也持续发布，保持速度指令有效）
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.lock = threading.Lock()

    def timer_callback(self):
        msg = Twist()
        with self.lock:
            msg.linear.x = self.vx
            msg.linear.y = self.vy
            msg.angular.z = self.vz
        self.pub.publish(msg)

    def set_velocity(self, vx: float, vy: float, vz: float):
        with self.lock:
            self.vx = vx
            self.vy = vy
            self.vz = vz

    def stop(self):
        self.set_velocity(0.0, 0.0, 0.0)

    def speed_up(self):
        self.speed_level = min(self.speed_level + 1, len(self.speeds) - 1)

    def speed_down(self):
        self.speed_level = max(self.speed_level - 1, 0)

    @property
    def speed(self) -> float:
        return self.speeds[self.speed_level]


def main():
    rclpy.init()
    topic = sys.argv[1] if len(sys.argv) > 1 else "/cmd_vel"
    node = KeyboardVelPublisher(topic)

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    term = _Terminal()

    print(BANNER)
    print(f"  Publishing to: {topic}")
    print(f"  Speed: {node.speed:.2f}  (档位 {node.speed_level})")
    print()

    try:
        while rclpy.ok():
            key = term.get_key(timeout=0.05)
            if not key:
                continue

            s = node.speed

            if key == "w":
                node.set_velocity(s, 0.0, 0.0)
                print(f"\r  前进  vx={s:.2f}  ", end="", flush=True)
            elif key == "s":
                node.set_velocity(-s, 0.0, 0.0)
                print(f"\r  后退  vx={-s:.2f}  ", end="", flush=True)
            elif key == "a":
                node.set_velocity(0.0, s, 0.0)
                print(f"\r  左移  vy={s:.2f}  ", end="", flush=True)
            elif key == "d":
                node.set_velocity(0.0, -s, 0.0)
                print(f"\r  右移  vy={-s:.2f}  ", end="", flush=True)
            elif key == "q":
                node.set_velocity(0.0, 0.0, s)
                print(f"\r  逆时针旋转  vz={s:.2f}  ", end="", flush=True)
            elif key == "e":
                node.set_velocity(0.0, 0.0, -s)
                print(f"\r  顺时针旋转  vz={-s:.2f}  ", end="", flush=True)
            elif key == " ":
                node.stop()
                print("\r  急停!               ", end="", flush=True)
            elif key.lower() == "r":
                node.speed_up()
                print(f"\r  速度档位: {node.speed_level} ({node.speed:.2f} m/s)  ", end="", flush=True)
            elif key.lower() == "f":
                node.speed_down()
                print(f"\r  速度档位: {node.speed_level} ({node.speed:.2f} m/s)  ", end="", flush=True)
            # Ctrl+C 会被 termios 传为 \x03
            elif key == "\x03":
                break

    except KeyboardInterrupt:
        pass
    finally:
        term.restore()
        node.stop()
        msg = Twist()
        node.pub.publish(msg)
        time.sleep(0.1)
        print("\n\n  已退出。")
        rclcpp.shutdown()


if __name__ == "__main__":
    main()
