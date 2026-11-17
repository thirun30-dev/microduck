#!/usr/bin/env python3
"""Convert the Microduck MJCF (pollen-robotics/microduck_rl, robot_walk.xml) to a URDF
for RViz and Gazebo Harmonic.

    mjcf_to_urdf.py <robot_walk.xml> <meshes_dir> <out.urdf>

* Every MuJoCo body becomes a link, every hinge a revolute joint (axis z of the child frame).
* Visual mesh geoms keep their pose and material colour.
* The two sole meshes become box collisions (fitted to the STL bounding box), which is
  far more stable in DART than mesh-vs-plane contact for a 25 cm robot.
* Servo gains, damping, armature / joint friction / limits come from the `chosen_actuator` class (the XL330 fit
  the policies were trained against). Gazebo plugins (MicroduckServo servos with rotor
  inertia, joint state, IMU, odometry) are appended so the same file is spawned straight into Gazebo.
"""
import math
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

PKG = "microduck_description"
BODY_BOXES = ("trunk_base", "jaw_soft")

# Home pose (duck-control/src/model.rs DEFAULT_POSITION, mouth removed).
HOME = {
    "left_hip_yaw": 0.0, "left_hip_roll": -0.0873, "left_hip_pitch": -0.4579,
    "left_knee": -0.0049, "left_ankle": 0.4530,
    "neck_pitch": 0.3491, "head_pitch": 0.3491, "head_yaw": 0.0, "head_roll": 0.0,
    "right_hip_yaw": 0.0, "right_hip_roll": 0.0873, "right_hip_pitch": 0.4579,
    "right_knee": 0.0049, "right_ankle": -0.4530,
}

# XL330 position servo as fitted in microduck_rl (class "chosen_actuator").
KP, FORCE, DAMPING, FRICTION, ARMATURE = 0.55, 0.96, 0.053, 0.0048, 0.0018


def floats(s, default):
    return [float(x) for x in s.split()] if s else list(default)


def quat_to_mat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_rpy(R):
    # URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)
    sy = -R[2, 0]
    if abs(sy) >= 1.0 - 1e-9:
        pitch = math.copysign(math.pi / 2, sy)
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    else:
        pitch = math.asin(sy)
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def origin(parent, xyz, R):
    o = ET.SubElement(parent, "origin")
    o.set("xyz", " ".join(f"{v:.6g}" for v in xyz))
    o.set("rpy", " ".join(f"{v:.6g}" for v in mat_to_rpy(R)))
    return o


def stl_bbox(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        pts = [list(map(float, l.split()[1:4])) for l in data.decode().splitlines()
               if l.strip().startswith("vertex")]
        pts = np.array(pts)
    else:
        n = struct.unpack("<I", data[80:84])[0]
        arr = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype(
            [("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
        pts = arr["v"].reshape(-1, 3)
    return pts.min(0), pts.max(0)


def main(mjcf, mesh_dir, out):
    root = ET.parse(mjcf).getroot()
    materials = {m.get("name"): m.get("rgba") for m in root.iter("material")}

    robot = ET.Element("robot", name="microduck")
    for name, rgba in materials.items():
        m = ET.SubElement(robot, "material", name=name)
        ET.SubElement(m, "color", rgba=rgba)

    joints = []

    def add_link(body):
        link = ET.SubElement(robot, "link", name=body.get("name"))
        inert = body.find("inertial")
        if inert is not None:
            i = ET.SubElement(link, "inertial")
            origin(i, floats(inert.get("pos"), [0, 0, 0]), np.eye(3))
            ET.SubElement(i, "mass", value=inert.get("mass"))
            ixx, iyy, izz, ixy, ixz, iyz = floats(inert.get("fullinertia"), [0] * 6)
            ET.SubElement(i, "inertia", ixx=f"{ixx:.6g}", iyy=f"{iyy:.6g}", izz=f"{izz:.6g}",
                          ixy=f"{ixy:.6g}", ixz=f"{ixz:.6g}", iyz=f"{iyz:.6g}")
        for g in body.findall("geom"):
            if g.get("type") != "mesh":
                continue
            pos = np.array(floats(g.get("pos"), [0, 0, 0]))
            R = quat_to_mat(floats(g.get("quat"), [1, 0, 0, 0]))
            mesh = g.get("mesh")
            if g.get("class") == "collision":
                lo, hi = stl_bbox(f"{mesh_dir}/{mesh}.stl")
                centre = (lo + hi) / 2
                c = ET.SubElement(link, "collision", name=g.get("name", mesh))
                origin(c, pos + R @ centre, R)
                geo = ET.SubElement(c, "geometry")
                ET.SubElement(geo, "box", size=" ".join(f"{v:.6g}" for v in (hi - lo)))
                continue
            v = ET.SubElement(link, "visual")
            origin(v, pos, R)
            geo = ET.SubElement(v, "geometry")
            ET.SubElement(geo, "mesh", filename=f"package://{PKG}/meshes/{mesh}.stl")
            if g.get("material") in materials:
                ET.SubElement(v, "material", name=g.get("material"))
        if body.get("name") in BODY_BOXES:
            # The walk MJCF only collides at the soles; give the trunk and head a box (the
            # AABB of their visual meshes, shrunk 10 %) so a fall lands instead of sinking.
            pts = []
            for g in body.findall("geom"):
                if g.get("type") == "mesh" and g.get("class") != "collision":
                    lo, hi = stl_bbox(f"{mesh_dir}/{g.get('mesh')}.stl")
                    R = quat_to_mat(floats(g.get("quat"), [1, 0, 0, 0]))
                    pos = np.array(floats(g.get("pos"), [0, 0, 0]))
                    corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                                        for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
                    pts.append(corners @ R.T + pos)
            pts = np.vstack(pts)
            lo, hi = pts.min(0), pts.max(0)
            c = ET.SubElement(link, "collision", name=f"{body.get('name')}_box")
            origin(c, (lo + hi) / 2, np.eye(3))
            geo = ET.SubElement(c, "geometry")
            ET.SubElement(geo, "box", size=" ".join(f"{v:.6g}" for v in 0.9 * (hi - lo)))
        for child in body.findall("body"):
            j = child.find("joint")
            pos = floats(child.get("pos"), [0, 0, 0])
            R = quat_to_mat(floats(child.get("quat"), [1, 0, 0, 0]))
            if j is None:
                jn = ET.SubElement(robot, "joint", name=f"{child.get('name')}_fixed", type="fixed")
            else:
                jn = ET.SubElement(robot, "joint", name=j.get("name"), type="revolute")
                lo, hi = floats(j.get("range"), [-math.pi, math.pi])
                ET.SubElement(jn, "axis", xyz=j.get("axis", "0 0 1"))
                ET.SubElement(jn, "limit", lower=f"{lo:.6g}", upper=f"{hi:.6g}",
                              effort=str(FORCE), velocity="8.0")
                ET.SubElement(jn, "dynamics", damping="0", friction=str(FRICTION))
                joints.append(j.get("name"))
            origin(jn, pos, R)
            ET.SubElement(jn, "parent", link=body.get("name"))
            ET.SubElement(jn, "child", link=child.get("name"))
            add_link(child)

    # SDF forbids a link and a joint sharing a name (MJCF has body+joint "neck_pitch").
    joint_names = {j.get("name") for j in root.iter("joint") if j.get("name")}
    for b in root.iter("body"):
        if b.get("name") in joint_names:
            b.set("name", b.get("name") + "_link")

    trunk = root.find("worldbody/body")
    add_link(trunk)

    # ---- Gazebo extensions -------------------------------------------------------------
    for foot in ("ankle_left", "ankle_right"):
        gz = ET.SubElement(robot, "gazebo", reference=foot)
        ET.SubElement(gz, "mu1").text = "1.0"
        ET.SubElement(gz, "mu2").text = "1.0"
        ET.SubElement(gz, "kp").text = "1e5"
        ET.SubElement(gz, "kd").text = "10"

    gz = ET.SubElement(robot, "gazebo", reference="trunk_base")
    s = ET.SubElement(gz, "sensor", name="imu", type="imu")
    ET.SubElement(s, "always_on").text = "1"
    ET.SubElement(s, "update_rate").text = "200"
    ET.SubElement(s, "topic").text = "/microduck/imu"
    ET.SubElement(s, "gz_frame_id").text = "trunk_base"
    # site "imu" in the MJCF
    ET.SubElement(s, "pose").text = "-0.021 0 -0.0147 0 0 0"

    gz = ET.SubElement(robot, "gazebo")
    p = ET.SubElement(gz, "plugin", filename="gz-sim-joint-state-publisher-system",
                      name="gz::sim::systems::JointStatePublisher")
    ET.SubElement(p, "topic").text = "/microduck/joint_states"
    p = ET.SubElement(gz, "plugin", filename="gz-sim-odometry-publisher-system",
                      name="gz::sim::systems::OdometryPublisher")
    ET.SubElement(p, "odom_frame").text = "odom"
    ET.SubElement(p, "robot_base_frame").text = "trunk_base"
    ET.SubElement(p, "dimensions").text = "3"
    ET.SubElement(p, "odom_publish_frequency").text = "50"
    ET.SubElement(p, "odom_topic").text = "/microduck/odom"
    ET.SubElement(p, "tf_topic").text = "/microduck/tf"
    # Gazebo spawns joints at 0 rad, not the home pose. Hold the trunk to a static hanger
    # (worlds/flat.sdf) until the servos reach home, then the policy node publishes detach.
    p = ET.SubElement(gz, "plugin", filename="gz-sim-detachable-joint-system",
                      name="gz::sim::systems::DetachableJoint")
    ET.SubElement(p, "parent_link").text = "trunk_base"
    ET.SubElement(p, "child_model").text = "duck_hanger"
    ET.SubElement(p, "child_link").text = "link"
    ET.SubElement(p, "detach_topic").text = "/microduck/detach"
    ET.SubElement(p, "attach_topic").text = "/microduck/attach"
    # XL330 servos with rotor inertia (microduck_gazebo/src/MicroduckServo.cc). Gazebo has no
    # joint armature; without it the policies fall while walking.
    p = ET.SubElement(gz, "plugin", filename="MicroduckServo",
                      name="microduck_gazebo::MicroduckServo")
    ET.SubElement(p, "topic_prefix").text = "/microduck/cmd/"
    for tag, val in (("kp", KP), ("max_torque", FORCE), ("damping", DAMPING),
                     ("armature", ARMATURE), ("coupling_stiffness", 10.0),
                     ("coupling_damping", 0.002)):
        ET.SubElement(p, tag).text = str(val)
    for name in joints:
        ET.SubElement(p, "joint", name=name, initial=str(HOME[name]))

    ET.indent(robot)
    header = ("<?xml version=\"1.0\"?>\n<!-- Generated by mjcf_to_urdf.py from pollen-robotics/"
              "microduck_rl robot_walk.xml (Apache-2.0). Meshes: CC BY-NC-SA 4.0, "
              "Pollen Robotics. -->\n")
    with open(out, "w") as f:
        f.write(header + ET.tostring(robot, encoding="unicode") + "\n")
    print(f"wrote {out}: {len(joints)} actuated joints: {' '.join(joints)}")


if __name__ == "__main__":
    main(*sys.argv[1:4])
