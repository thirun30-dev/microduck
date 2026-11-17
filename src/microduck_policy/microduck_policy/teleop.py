"""Keyboard teleop for the Gazebo Microduck: walk with /cmd_vel, steer the head with /microduck/head.

    ros2 run microduck_policy teleop

The velstand policy only steps above ~0.2 m/s; below that it stands, so the first press
of w/x jumps straight to 0.3 m/s.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

HELP = """
Microduck teleop  (Ctrl-C to quit)
  walk     w / x : forward / back (0.3 m/s first, then +-0.1, max 0.5)
           a / d : turn left / right      q / e : sidestep left / right
           space or s : stop
  head     i / k : look up / down         j / l : look left / right
           u / o : tilt left / right      h : centre head
"""
MAX_VX, STEP_VX, WALK_VX = 0.5, 0.1, 0.3
MAX_WZ, STEP_WZ = 1.5, 0.25
MAX_VY = 0.2
HEAD_STEP = 0.1
HEAD_LIM = [(-0.6, 0.6), (-0.6, 0.6), (-1.2, 1.2), (-0.4, 0.4)]  # neck_pitch head_pitch head_yaw head_roll


class Teleop(Node):
    def __init__(self):
        super().__init__("microduck_teleop")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.head_pub = self.create_publisher(Float64MultiArray, "/microduck/head", 10)
        self.vx = self.vy = self.wz = 0.0
        self.head = [0.0, 0.0, 0.0, 0.0]

    def key(self, k):
        clamp = lambda v, lo, hi: max(lo, min(hi, v))
        if k == "w":
            self.vx = WALK_VX if self.vx < WALK_VX - 1e-6 and self.vx >= 0 else self.vx + STEP_VX
        elif k == "x":
            self.vx = -WALK_VX if self.vx > -WALK_VX + 1e-6 and self.vx <= 0 else self.vx - STEP_VX
        elif k == "a":
            self.wz += STEP_WZ
        elif k == "d":
            self.wz -= STEP_WZ
        elif k == "q":
            self.vy = MAX_VY if self.vy <= 0 else 0.0
        elif k == "e":
            self.vy = -MAX_VY if self.vy >= 0 else 0.0
        elif k in (" ", "s"):
            self.vx = self.vy = self.wz = 0.0
        elif k in "ikjluo":
            i, d = {"i": (1, -1), "k": (1, 1), "j": (2, 1), "l": (2, -1), "u": (3, -1), "o": (3, 1)}[k]
            self.head[i] = clamp(self.head[i] + d * HEAD_STEP, *HEAD_LIM[i])
        elif k == "h":
            self.head = [0.0, 0.0, 0.0, 0.0]
        else:
            return
        self.vx = clamp(round(self.vx, 3), -MAX_VX, MAX_VX)
        self.wz = clamp(round(self.wz, 3), -MAX_WZ, MAX_WZ)
        sys.stdout.write(f"\r  vx={self.vx:+.2f}  vy={self.vy:+.2f}  wz={self.wz:+.2f}  "
                         f"head={[round(h, 1) for h in self.head]}        ")
        sys.stdout.flush()

    def publish(self):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = self.vx, self.vy, self.wz
        self.cmd_pub.publish(t)
        self.head_pub.publish(Float64MultiArray(data=self.head))


def main():
    # Keep SIGINT for ourselves so the final zero command still goes out on Ctrl-C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Teleop()
    if not sys.stdin.isatty():
        node.get_logger().error("teleop needs an interactive terminal")
        return
    print(HELP)
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                node.key(sys.stdin.read(1).lower())
            node.publish()  # 10 Hz keep-alive
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        node.vx = node.vy = node.wz = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
        node.destroy_node()
        rclpy.try_shutdown()
