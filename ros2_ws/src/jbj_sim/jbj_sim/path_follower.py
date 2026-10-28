"""Pure-pursuit path follower for the rear-axle reference point. Runs on every GNSS fix (10 Hz), like an autosteer.

Publishes the steering angle (bicycle-model equivalent) + speed request, the planter up/down command and status.
"""
import json
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from . import spec
from .common import PathTracker, load_plan, wrap, yaw_from_quat


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")
        self.declare_parameter("tractor", "")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("lookahead_gain_s", 1.4)
        self.declare_parameter("lookahead_min_m", 2.5)
        self.declare_parameter("lookahead_max_m", 6.0)
        _, _, g = spec.tractor(self.get_parameter("tractor").value or None)
        self.L = g["L"]
        self.dmax = 0.95 * g["delta_eq_max"]
        self.path, _, _, self.meta = load_plan(self.get_parameter("run_dir").value)
        self.trk = PathTracker(self.path)
        self.done = False
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, "/control/cmd", 10)
        self.pub_impl = self.create_publisher(Bool, "/implement/down", 10)
        self.pub_status = self.create_publisher(String, "/control/status", 10)

    def on_pose(self, m):
        x, y, yaw = m.pose.position.x, m.pose.position.y, yaw_from_quat(m.pose.orientation)
        p = self.path
        i = self.trk.update(x, y)
        end_d = math.hypot(p[-1, 0] - x, p[-1, 1] - y)
        if i >= len(p) - 3 and end_d < 1.5:
            self.done = True
        v_ref = 0.0 if self.done else float(p[i, 5])
        g = self.get_parameter
        ld = float(np.clip(g("lookahead_gain_s").value * max(v_ref, 0.6), g("lookahead_min_m").value, g("lookahead_max_m").value))
        j = self.trk.point_ahead(ld)
        tx, ty = p[j, 0], p[j, 1]
        dist = max(math.hypot(tx - x, ty - y), 0.5)
        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
        delta = float(np.clip(math.atan2(2.0 * self.L * math.sin(alpha), dist), -self.dmax, self.dmax))
        cmd = AckermannDriveStamped()
        cmd.header.stamp = m.header.stamp
        cmd.drive.steering_angle = delta
        cmd.drive.speed = v_ref
        self.pub.publish(cmd)
        work = bool(p[i, 4] > 0.5) and not self.done
        self.pub_impl.publish(Bool(data=work))
        kind = self.meta["kinds"][int(p[i, 6])]
        self.pub_status.publish(String(data=json.dumps({
            "idx": i, "n": len(p), "xte": self.trk.xte(x, y), "work": work, "kind": kind, "done": self.done,
            "progress": float(self.trk.s[i] / self.trk.s[-1])})))


def main():
    rclpy.init()
    rclpy.spin(PathFollower())


if __name__ == "__main__":
    main()
