"""Run a Microduck ONNX policy from the Hugging Face Hub against the Gazebo duck.

Mirrors robotd's control step (pollen-robotics/microduck, duck-control/src/obs.rs and
robotd/src/control.rs):

    obs[61] = gyro(3) | projected gravity(3) | q - home (14) | qdot (14) | last action (14)
              | command(13) = vx vy vyaw | neck_pitch head_pitch head_yaw head_roll
                              | body x y z roll pitch yaw (x, y, yaw always 0)
    target  = home + action_scale * action, then an EMA low-pass (legs 0.7, head 0.5)

Topics: /cmd_vel (geometry_msgs/Twist) drives it; /microduck/head
(std_msgs/Float64MultiArray, 4 values) steers the head.
"""
import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, Float64, Float64MultiArray

from .hub import fetch

JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
          "neck_pitch", "head_pitch", "head_yaw", "head_roll",
          "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]
HOME = np.array([0.0, -0.0873, -0.4579, -0.0049, 0.4530,
                 0.3491, 0.3491, 0.0, 0.0,
                 0.0, 0.0873, 0.4579, 0.0049, -0.4530])
HEAD = slice(5, 9)
LEGS = np.r_[0:5, 9:14]


def quat_rotate_inverse(q, v):
    """Rotate world vector v into the body frame of quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    qv = np.array([x, y, z])
    return v * (2 * w * w - 1) - np.cross(qv, v) * 2 * w + qv * 2 * np.dot(qv, v)


class PolicyNode(Node):
    def __init__(self):
        super().__init__("microduck_policy")
        p = self.declare_parameter
        repo = p("hf_repo", "pollen-robotics/microduck-policies").value
        rev = p("hf_revision", "v10").value
        fname = p("policy_file", "velstand.onnx").value
        self.scale = p("action_scale", 0.9).value
        self.legs_alpha = p("legs_lowpass", 0.7).value
        self.head_alpha = p("head_lowpass", 0.5).value
        self.settle_s = p("settle_s", 1.5).value
        self.max_v = p("max_vx", 0.5).value
        self.debug = p("debug", False).value
        self.log_every = p("log_every", 250).value

        self.get_logger().info(f"fetching {repo}@{rev}/{fname} from the Hugging Face Hub …")
        path = fetch(fname, repo, rev)
        self.sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        inp = self.sess.get_inputs()
        self.inputs = {i.name: i for i in inp}
        self.in_name = inp[0].name
        self.recurrent = len(inp) > 1
        self.state = {i.name: np.zeros([d if isinstance(d, int) else 1 for d in i.shape], np.float32)
                      for i in inp[1:]}
        meta = self.sess.get_modelmeta().custom_metadata_map
        self.get_logger().info(f"loaded {path}\n  inputs: {[(i.name, i.shape) for i in inp]}"
                               f"\n  outputs: {[(o.name, o.shape) for o in self.sess.get_outputs()]}"
                               f"\n  metadata: {dict(list(meta.items())[:8])}")

        self.q = self.qd = None
        self.gyro = np.zeros(3)
        self.quat = None
        self.cmd = np.zeros(3)
        self.head_cmd = np.zeros(4)
        self.last_action = np.zeros(14, np.float32)
        self.prev_target = HOME.copy()
        self.start = None

        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Imu, "/microduck/imu", self.on_imu, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Float64MultiArray, "/microduck/head", self.on_head, 10)
        self.pubs = [self.create_publisher(Float64, f"/microduck/cmd/{j}", 10) for j in JOINTS]
        self.detach_pub = self.create_publisher(Empty, "/microduck/detach", 10)
        self.released_at = None
        self.create_timer(0.02, self.step)  # 50 Hz, sim time
        self.ticks = 0

    def on_joints(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        self.q = np.array([msg.position[idx[j]] for j in JOINTS])
        self.qd = np.array([msg.velocity[idx[j]] if msg.velocity else 0.0 for j in JOINTS])

    def on_imu(self, msg):
        self.gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        o = msg.orientation
        self.quat = np.array([o.x, o.y, o.z, o.w])

    def on_cmd(self, msg):
        self.cmd = np.clip([msg.linear.x, msg.linear.y, msg.angular.z],
                           [-self.max_v, -0.2, -1.5], [self.max_v, 0.2, 1.5])

    def on_head(self, msg):
        if len(msg.data) == 4:
            self.head_cmd = np.array(msg.data)

    def send(self, target):
        for pub, t in zip(self.pubs, target):
            pub.publish(Float64(data=float(t)))

    def step(self):
        if self.q is None or self.quat is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start = now
            self.get_logger().info("sensors live; holding home pose to settle")
        if self.released_at is None:
            # Hanging from the hanger: drive to home, then drop onto the feet.
            self.send(HOME)
            err = np.abs(self.q - HOME).max()
            if now - self.start > self.settle_s and (err < 0.03 or now - self.start > 8.0):
                self.detach_pub.publish(Empty())
                self.released_at = now
                self.get_logger().info(f"joints at home (max err {err:.3f} rad): released onto the floor")
            return
        if now - self.released_at < 0.3:  # let the feet land
            self.detach_pub.publish(Empty())
            self.send(HOME)
            return

        gravity = quat_rotate_inverse(self.quat, np.array([0.0, 0.0, -1.0]))
        command = np.zeros(13)
        command[0:3] = self.cmd
        command[3:7] = self.head_cmd
        obs = np.concatenate([self.gyro, gravity, self.q - HOME, self.qd,
                              self.last_action, command]).astype(np.float32)[None]

        feeds = {self.in_name: obs, **self.state}
        outs = self.sess.run(None, feeds)
        action = np.asarray(outs[0], np.float32).reshape(-1)[:14]
        if self.recurrent:  # carry LSTM state: outputs after the action map onto inputs
            for name, val in zip(list(self.state), outs[1:]):
                self.state[name] = val
        action = np.nan_to_num(action)
        self.last_action = action

        target = HOME + self.scale * action
        target[LEGS] = self.legs_alpha * target[LEGS] + (1 - self.legs_alpha) * self.prev_target[LEGS]
        target[HEAD] = self.head_alpha * target[HEAD] + (1 - self.head_alpha) * self.prev_target[HEAD]
        self.prev_target = target
        self.send(target)

        self.ticks += 1
        if self.debug and self.ticks <= 60 and self.ticks % 3 == 0:
            self.get_logger().info(
                f"tick {self.ticks:3d} grav={gravity.round(2)} gyro={self.gyro.round(2)} "
                f"dq={np.round(self.q - HOME, 2)[[2, 3, 4, 11, 12, 13]]} a={np.round(action[[2, 3, 4]], 2)}")
        if self.ticks % self.log_every == 0:
            tilt = np.degrees(np.arccos(np.clip(-gravity[2], -1, 1)))
            self.get_logger().info(f"t={now - self.start:6.1f}s  cmd={self.cmd.round(2)}  tilt={tilt:4.1f}°  gyro={np.abs(self.gyro).max():.2f}")


def main():
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
