"""Dual-antenna RTK GNSS receiver model.

Takes the two Gazebo NavSat antennas (perfect positions), adds realistic receiver errors and publishes the
rear-axle pose in the local field frame (x = east, y = north), plus the RTK status.

Error model (per fix quality, see specs/kit.json -> gnss):
  * common-mode position error: 1st-order Gauss-Markov (slowly wandering), sigma from the receiver datasheet
  * small white noise on each antenna; the relative noise between the antennas sets the heading accuracy
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from . import spec
from .common import lla_to_enu

TAU = {"FIXED": 60.0, "FLOAT": 20.0, "NONE": 60.0}       # s, correlation time of the position error (ASSUMPTION)
SIGMA_KEY = {"FIXED": "rtk_fixed_sigma_m", "FLOAT": "rtk_float_sigma_m", "NONE": "single_sigma_m"}


class GnssReceiver(Node):
    def __init__(self):
        super().__init__("gnss_receiver")
        self.declare_parameter("tractor", "")
        self.declare_parameter("seed", 7)
        tid, _, g = spec.tractor(self.get_parameter("tractor").value or None)
        kit, field = spec.load("kit"), spec.load("field")
        self.gn = kit["gnss"]
        self.origin = (field["origin_lat"], field["origin_lon"], field["origin_alt_m"])
        self.ant_x = 0.30                                   # antennas sit 0.30 m ahead of the rear axle (see gen_models.py)
        self.rng = np.random.default_rng(self.get_parameter("seed").value)
        self.err = np.zeros(2)
        self.quality = "FIXED"
        self.last_t = None
        self.latest = {}
        self.create_subscription(NavSatFix, "/gnss/left", lambda m: self.on_fix("L", m), 20)
        self.create_subscription(NavSatFix, "/gnss/right", lambda m: self.on_fix("R", m), 20)
        self.create_subscription(String, "/sim/gnss_quality", lambda m: setattr(self, "quality", m.data), 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/localization/pose", 10)
        self.pub_status = self.create_publisher(String, "/gnss/rtk_status", 10)
        self.get_logger().info(f"dual-antenna RTK model for {tid}, {self.gn['rate_hz']} Hz")

    def on_fix(self, side, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.latest[side] = (t, m)
        if "L" in self.latest and "R" in self.latest and abs(self.latest["L"][0] - self.latest["R"][0]) < 0.02:
            self.process(t, self.latest.pop("L")[1], self.latest.pop("R")[1])

    def process(self, t, ml, mr):
        q = self.quality
        dt = 0.1 if self.last_t is None else max(1e-3, t - self.last_t)
        self.last_t = t
        sigma, tau = self.gn[SIGMA_KEY[q]], TAU[q]
        if q != getattr(self, "last_q", q):          # new fix type: the receiver's solution jumps to the new accuracy
            self.err = self.rng.normal(0.0, sigma, 2)
        self.last_q = q
        phi = math.exp(-dt / tau)
        self.err = phi * self.err + self.rng.normal(0.0, sigma * math.sqrt(1 - phi * phi), 2)
        if q == "FIXED":   # per-antenna noise that gives the published dual-antenna heading accuracy
            s_rel = self.gn["antenna_baseline_m"] * math.radians(self.gn["heading_sigma_deg"]) / math.sqrt(2)
        else:
            s_rel = 0.02 if q == "FLOAT" else 0.3
        pl = np.array(lla_to_enu(ml.latitude, ml.longitude, ml.altitude, *self.origin)[:2]) + self.err + self.rng.normal(0, s_rel, 2)
        pr = np.array(lla_to_enu(mr.latitude, mr.longitude, mr.altitude, *self.origin)[:2]) + self.err + self.rng.normal(0, s_rel, 2)
        d = pl - pr                                          # right -> left antenna = body +Y
        yaw = math.atan2(d[1], d[0]) - math.pi / 2
        mid = (pl + pr) / 2
        x, y = mid[0] - self.ant_x * math.cos(yaw), mid[1] - self.ant_x * math.sin(yaw)
        msg = PoseStamped()
        msg.header.stamp = ml.header.stamp
        msg.header.frame_id = "field"
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub_pose.publish(msg)
        self.pub_status.publish(String(data=q))


def main():
    rclpy.init()
    rclpy.spin(GnssReceiver())


if __name__ == "__main__":
    main()
