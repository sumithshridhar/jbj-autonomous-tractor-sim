"""Safety supervisor: sits between the path follower and the vehicle interface and can always stop the tractor.

Stops (practical minimum, see specs/kit.json -> safety):
  ESTOP          handheld remote e-stop pressed                         latched, needs release + resume
  REMOTE_LOST    no heartbeat from the handheld remote for > timeout     latched, needs signal back + resume
  RTK            RTK fix lost for > rtk_loss_stop_s                      auto-resume after 3 s of RTK fix
  OBSTACLE       lidar points in the path corridor inside stop distance  auto-resume after the corridor is clear
  OFFTRACK       cross-track error too big (e.g. steering fault)         latched, needs resume
  GEOFENCE       tractor+planter footprint about to leave the field      latched, needs resume
Obstacles inside the slow zone limit the speed (SLOW).
"""
import json
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shapely.geometry import Polygon
from std_msgs.msg import Bool, Float64MultiArray, String

from . import spec
from .common import PathTracker, load_plan, yaw_from_quat
from .planner import footprint

LATCHED = ("ESTOP", "REMOTE_LOST", "OFFTRACK", "GEOFENCE")


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("safety_supervisor")
        self.declare_parameter("tractor", "")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("obstacle_auto_resume_s", 5.0)
        self.declare_parameter("offtrack_work_m", 0.6)
        self.declare_parameter("offtrack_turn_m", 1.5)
        _, _, self.g = spec.tractor(self.get_parameter("tractor").value or None)
        self.kit = spec.load("kit")
        self.sf = self.kit["safety"]
        self.path, field, _, self.meta = load_plan(self.get_parameter("run_dir").value)
        self.fence = Polygon(field).buffer(self.sf["geofence_margin_m"])
        self.trk = PathTracker(self.path)
        self.corr_half = self.kit["implement"]["working_width_m"] / 2 + 0.35
        self.lidar = self.g["lidar_xyz"]
        self.pose = None
        self.v = 0.0
        self.cmd = (0.0, 0.0)
        self.status = {}
        self.rtk = "NONE"
        self.rtk_bad_since = None
        self.rtk_good_since = None
        self.hb_t = None
        self.estop_pressed = False
        self.resume_req = False
        self.latched = None
        self.obstacle_gap = math.inf
        self.obstacle_stop = False
        self.obstacle_clear_since = None
        self.state = "WAIT"
        self.reason = "waiting for GNSS + remote"
        self.create_subscription(AckermannDriveStamped, "/control/cmd", self.on_cmd, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_subscription(String, "/gnss/rtk_status", self.on_rtk, 10)
        self.create_subscription(String, "/control/status", lambda m: setattr(self, "status", json.loads(m.data)), 10)
        self.create_subscription(Bool, "/remote/estop", self.on_remote, 10)
        self.create_subscription(Bool, "/operator/resume", lambda m: setattr(self, "resume_req", bool(m.data)), 10)
        self.create_subscription(Float64MultiArray, "/vehicle/state", lambda m: setattr(self, "v", m.data[1]), 10)
        self.create_subscription(PointCloud2, "/lidar/points", self.on_lidar, qos_profile_sensor_data)
        self.pub = self.create_publisher(AckermannDriveStamped, "/vehicle/cmd", 10)
        self.pub_state = self.create_publisher(String, "/safety/state", 10)
        self.create_timer(0.05, self.step)

    # ------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, m):
        self.cmd = (m.drive.steering_angle, m.drive.speed)

    def on_pose(self, m):
        self.pose = (m.pose.position.x, m.pose.position.y, yaw_from_quat(m.pose.orientation))
        self.trk.update(self.pose[0], self.pose[1])

    def on_rtk(self, m):
        t = self.now()
        if m.data != "FIXED":
            self.rtk_good_since = None
            if self.rtk_bad_since is None:
                self.rtk_bad_since = t
        else:
            self.rtk_bad_since = None
            if self.rtk_good_since is None:
                self.rtk_good_since = t
        self.rtk = m.data

    def on_remote(self, m):
        self.hb_t = self.now()
        self.estop_pressed = bool(m.data)

    def stop_distance(self, v):
        return v * v / (2 * self.sf["brake_decel_mps2"]) + v * self.sf["reaction_s"] + self.sf["stop_margin_m"]

    def on_lidar(self, msg):
        if self.pose is None:
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        lx, ly, lz = self.lidar
        xb, yb, zb = lx + pts[:, 0], ly - pts[:, 1], lz - pts[:, 2]       # sensor is mounted upside down (roll = pi)
        # self-mask: the lidar also sees the tractor (bumper e-stop sticks out 6 cm), so ignore the first 15 cm
        keep = (zb > self.sf["obstacle_min_height_m"]) & (zb < 3.0) & (xb > self.g["x_front"] + 0.15)
        xb, yb = xb[keep], yb[keep]
        gap = math.inf
        if len(xb):
            x, y, yaw = self.pose
            c, s = math.cos(yaw), math.sin(yaw)
            fx, fy = x + c * xb - s * yb, y + s * xb + c * yb
            i0 = self.trk.idx
            look = self.sf["slow_zone_m"] + self.g["x_front"] + 6.0
            i1 = self.trk.point_ahead(look) + 1
            seg = self.path[i0:i1:2]
            sarr = self.trk.s[i0:i1:2] - self.trk.s[i0]
            if len(seg):
                d2 = (fx[:, None] - seg[None, :, 0]) ** 2 + (fy[:, None] - seg[None, :, 1]) ** 2
                j = np.argmin(d2, axis=1)
                lat = np.sqrt(d2[np.arange(len(j)), j])
                along = sarr[j] - self.g["x_front"]
                hit = (lat < self.corr_half) & (along > -0.5)
                if np.count_nonzero(hit) >= 3:
                    gap = float(np.percentile(along[hit], 5))
        self.obstacle_gap = gap

    # ------------------------------------------------------------ decision
    def step(self):
        t = self.now()
        if self.pose is None or self.hb_t is None:
            self.publish(0.0, "WAIT", self.reason)
            return
        # The simulator's clock can jump when Gazebo loads a model. Don't blame the remote for a gap in our own clock.
        if t - getattr(self, "last_step_t", t) > 0.25:
            self.hb_t = max(self.hb_t, t - 0.05)
        self.last_step_t = t
        reason = None
        # latched stops
        if self.estop_pressed:
            self.latched = ("ESTOP", "remote e-stop pressed")
        elif t - self.hb_t > self.sf["remote_timeout_s"]:
            self.latched = ("REMOTE_LOST", f"no remote signal for {t - self.hb_t:.1f} s")
        xte = abs(self.status.get("xte", 0.0))
        lim = self.get_parameter("offtrack_work_m").value if self.status.get("work") else self.get_parameter("offtrack_turn_m").value
        if self.latched is None and xte > lim:
            self.latched = ("OFFTRACK", f"cross-track error {xte:.2f} m > {lim} m")
        x, y, yaw = self.pose
        ahead = max(self.v, 0.3) * 1.0
        if self.latched is None:
            for dx in (0.0, ahead):
                fp = footprint(x + dx * math.cos(yaw), y + dx * math.sin(yaw), yaw, self.g, self.kit)
                if not self.fence.contains(fp):
                    self.latched = ("GEOFENCE", "tractor/planter would leave the field")
                    break
        if self.latched is not None:
            kind, why = self.latched
            cleared = (kind == "ESTOP" and not self.estop_pressed) or \
                      (kind == "REMOTE_LOST" and t - self.hb_t <= self.sf["remote_timeout_s"]) or kind in ("OFFTRACK", "GEOFENCE")
            if cleared and self.resume_req and kind in ("ESTOP", "REMOTE_LOST"):
                self.latched, self.resume_req = None, False
            else:
                self.publish(0.0, "STOP_" + kind, why)
                return
        # RTK
        if self.rtk_bad_since is not None and t - self.rtk_bad_since > self.sf["rtk_loss_stop_s"]:
            self.publish(0.0, "STOP_RTK", f"RTK {self.rtk} for {t - self.rtk_bad_since:.1f} s")
            return
        if self.rtk_good_since is None or t - self.rtk_good_since < 3.0:
            if self.state == "STOP_RTK":
                self.publish(0.0, "STOP_RTK", "waiting for a stable RTK fix")
                return
        # obstacles
        v_plan = self.cmd[1]
        d_stop = self.stop_distance(max(self.v, v_plan))
        if self.obstacle_gap < d_stop:
            self.obstacle_stop = True
            self.obstacle_clear_since = None
        elif self.obstacle_stop:
            if self.obstacle_gap > d_stop + 2.0:
                self.obstacle_clear_since = self.obstacle_clear_since or t
                if t - self.obstacle_clear_since >= self.get_parameter("obstacle_auto_resume_s").value:
                    self.obstacle_stop = False
            else:
                self.obstacle_clear_since = None
        if self.obstacle_stop:
            self.publish(0.0, "STOP_OBSTACLE", f"object {self.obstacle_gap:.1f} m ahead (stop distance {d_stop:.1f} m)")
            return
        if self.status.get("done"):
            self.publish(0.0, "DONE", "plan finished")
            return
        if self.obstacle_gap < self.sf["slow_zone_m"]:
            self.publish(min(v_plan, self.sf["slow_speed_kmh"] / 3.6), "SLOW", f"object {self.obstacle_gap:.1f} m ahead")
            return
        self.publish(v_plan, "RUN", "")

    def publish(self, v, state, reason):
        self.state, self.reason = state, reason
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.steering_angle = self.cmd[0]
        out.drive.speed = float(v)
        self.pub.publish(out)
        self.pub_state.publish(String(data=json.dumps({"state": state, "reason": reason,
                                                       "gap": None if math.isinf(self.obstacle_gap) else round(self.obstacle_gap, 2)})))


def main():
    rclpy.init()
    try:
        rclpy.spin(SafetySupervisor())
    except (KeyboardInterrupt, ExternalShutdownException):   # normal end of a run
        pass


if __name__ == "__main__":
    main()
