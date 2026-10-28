"""Scenario runner: plays the operator, the handheld remote, animals/people and the GNSS conditions.
Logs every run at 10 Hz (ground truth from Gazebo), scores it and writes results to run_dir.

Scenarios: full_field, cow, person, remote_estop, remote_lost, rtk_loss, steering_fault
"""
import csv
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from . import spec
from .common import PathTracker, footprint_corners, load_plan, yaw_from_quat

SCENARIOS = {
    "full_field": {"timeout": 7200.0},
    "cow": {"timeout": 200.0, "t_event": 30.0},
    "person": {"timeout": 200.0, "t_event": 30.0},
    "remote_estop": {"timeout": 120.0, "t_event": 40.0},
    "remote_lost": {"timeout": 120.0, "t_event": 40.0},
    "rtk_loss": {"timeout": 140.0, "t_event": 40.0},
    "steering_fault": {"timeout": 120.0, "t_event": 40.0},
}
OBST_SIZE = {"cow": (2.1, 0.62), "person": (0.3, 0.6)}


class ScenarioRunner(Node):
    def __init__(self):
        super().__init__("scenario_runner")
        for k, v in (("scenario", "full_field"), ("tractor", ""), ("run_dir", ""), ("seed", 7)):
            self.declare_parameter(k, v)
        self.name = self.get_parameter("scenario").value
        self.cfg = SCENARIOS[self.name]
        self.tid, self.t_spec, self.g = spec.tractor(self.get_parameter("tractor").value or None)
        self.kit = spec.load("kit")
        self.run_dir = Path(self.get_parameter("run_dir").value)
        self.path, field, _, self.meta = load_plan(self.run_dir)
        self.field = Polygon(field)
        self.trk = PathTracker(self.path)
        self.share = Path(get_package_share_directory("jbj_sim"))
        self.gt = None
        self.est = None
        self.vstate = (0.0, 0.0)
        self.safety = {"state": "WAIT", "reason": "", "gap": None}
        self.status = {}
        self.rtk = ""
        self.work = False
        self.t0 = None
        self.rows = []
        self.events = []
        self.ev = {}
        self.estop = False
        self.hb_on = True
        self.quality = "FIXED"
        self.obst = None            # (name, x, y, yaw)
        self.finished = False
        self.create_subscription(Odometry, "/model/tractor/ground_truth", self.on_gt, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_est, 10)
        self.create_subscription(Float64MultiArray, "/vehicle/state", lambda m: setattr(self, "vstate", tuple(m.data)), 10)
        self.create_subscription(String, "/safety/state", lambda m: setattr(self, "safety", json.loads(m.data)), 10)
        self.create_subscription(String, "/control/status", lambda m: setattr(self, "status", json.loads(m.data)), 10)
        self.create_subscription(String, "/gnss/rtk_status", lambda m: setattr(self, "rtk", m.data), 10)
        self.create_subscription(Bool, "/implement/down", lambda m: setattr(self, "work", m.data), 10)
        self.pub_estop = self.create_publisher(Bool, "/remote/estop", 10)
        self.pub_q = self.create_publisher(String, "/sim/gnss_quality", 10)
        self.pub_resume = self.create_publisher(Bool, "/operator/resume", 10)
        self.pub_fault = self.create_publisher(Float64, "/sim/steer_fault", 10)
        self.pub_stuck = self.create_publisher(Float64, "/sim/steer_stuck", 10)
        self.cli_spawn = self.create_client(SpawnEntity, "/world/field/create")
        self.cli_pose = self.create_client(SetEntityPose, "/world/field/set_pose")
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f"scenario '{self.name}' on {self.tid}")

    # ------------------------------------------------------------ helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_from_quat(p.orientation), m.twist.twist.linear.x)

    def on_est(self, m):
        self.est = (m.pose.position.x, m.pose.position.y, yaw_from_quat(m.pose.orientation))

    def mark(self, key, t, **info):
        if key not in self.ev:
            self.ev[key] = t
            self.events.append(dict(event=key, t=round(t, 2), **info))
            self.get_logger().info(f"[{t:7.1f} s] {key} {info if info else ''}")

    def path_point_ahead(self, dist):
        i = self.trk.point_ahead(dist)
        return self.path[i, 0], self.path[i, 1], self.path[i, 2]

    def spawn(self, name, x, y, yaw):
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf_filename = str(self.share / "models" / name / "model.sdf")
        req.entity_factory.pose.position.x, req.entity_factory.pose.position.y = float(x), float(y)
        req.entity_factory.pose.orientation.z, req.entity_factory.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.cli_spawn.call_async(req)
        self.obst = (name, x, y, yaw)

    def move(self, x, y):
        name, _, _, yaw = self.obst
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        req.pose.position.x, req.pose.position.y = float(x), float(y)
        req.pose.orientation.z, req.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.cli_pose.call_async(req)
        self.obst = (name, x, y, yaw)

    def obstacle_gap_true(self):
        """Distance from the tractor's front bumper to the obstacle's outline (ground truth)."""
        if self.obst is None or self.gt is None:
            return None
        name, ox, oy, oyaw = self.obst
        L, W = OBST_SIZE[name]
        box = Polygon(footprint_corners(ox, oy, oyaw, L / 2, -L / 2, W / 2))
        x, y, yaw, _ = self.gt
        hw = (self.g["track"] + self.g["rear"]["w"]) / 2
        bumper = LineString([(x + math.cos(yaw) * self.g["x_front"] - math.sin(yaw) * s, y + math.sin(yaw) * self.g["x_front"] + math.cos(yaw) * s)
                             for s in (-hw, hw)])
        return bumper.distance(box)

    # ------------------------------------------------------------ main loop
    def tick(self):
        if self.finished:
            return
        t_abs = self.now()
        self.pub_q.publish(String(data=self.quality))
        if self.hb_on:
            self.pub_estop.publish(Bool(data=self.estop))
        if self.gt is None:
            return
        self.trk.update(self.gt[0], self.gt[1])
        if self.t0 is None:
            if self.safety.get("state") in ("RUN", "SLOW"):
                self.t0 = t_abs
                self.mark("start", 0.0)
            return
        t = t_abs - self.t0
        self.script(t)
        self.log(t)
        if self.status.get("done") and self.gt[3] < 0.05:
            self.mark("plan_done", t)
            self.finish(t)
        elif t > self.cfg["timeout"] or self.ev.get("end") is not None:
            self.finish(t)

    def script(self, t):
        n, te = self.name, self.cfg.get("t_event", 1e9)
        st = self.safety.get("state", "")
        v = self.gt[3]
        if n in ("cow", "person") and t >= te and self.obst is None:
            ahead = 25.0 if n == "cow" else 22.0
            x, y, yaw = self.path_point_ahead(self.g["x_front"] + ahead)
            if n == "cow":    # cow standing across the path
                self.spawn("cow", x, y, yaw + math.pi / 2)
            else:             # person starts 7 m to the left of the path and walks across
                self.walk = (x - 7 * math.sin(yaw), y + 7 * math.cos(yaw), math.sin(yaw), -math.cos(yaw))
                self.spawn("person", self.walk[0], self.walk[1], yaw - math.pi / 2)
            self.mark("obstacle_appears", t, obstacle=n, ahead_of_bumper_m=ahead)
        if n == "cow" and "obstacle_appears" in self.ev:
            if st == "STOP_OBSTACLE":
                self.mark("stopped_for_obstacle", t, gap_m=round(self.obstacle_gap_true(), 2))
            if "stopped_for_obstacle" in self.ev and t - self.ev["stopped_for_obstacle"] > 15 and t - self.ev["stopped_for_obstacle"] < 22:
                name, x, y, yaw = self.obst       # cow walks away sideways at ~1 m/s
                self.move(x + 0.1 * math.cos(yaw), y + 0.1 * math.sin(yaw))
                self.mark("cow_walks_away", t)
            if "cow_walks_away" in self.ev and st == "RUN" and v > 0.3:
                self.mark("resumed", t)
            if "resumed" in self.ev and t - self.ev["resumed"] > 20:
                self.ev["end"] = t
        if n == "person" and "obstacle_appears" in self.ev:
            k = t - self.ev["obstacle_appears"]
            if k < 10.0:                       # walks 14 m across at 1.4 m/s
                x0, y0, ux, uy = self.walk
                self.move(x0 + ux * 1.4 * k, y0 + uy * 1.4 * k)
            if st == "SLOW":
                self.mark("slowed_for_obstacle", t, gap_m=round(self.obstacle_gap_true(), 2))
            if st == "STOP_OBSTACLE":
                self.mark("stopped_for_obstacle", t, gap_m=round(self.obstacle_gap_true(), 2))
            if k > 10.0 and st == "RUN" and v > 0.3:
                self.mark("resumed", t)
            if "resumed" in self.ev and t - self.ev["resumed"] > 20:
                self.ev["end"] = t
        if n == "remote_estop":
            if te <= t < te + 10:
                self.estop = True
                self.mark("estop_pressed", t, speed_mps=round(v, 2))
            elif t >= te + 10:
                self.estop = False
                self.mark("estop_released", t)
            if "estop_pressed" in self.ev and v < 0.05:
                self.mark("standstill", t)
            if t >= te + 13 and "resume_sent" not in self.ev:
                self.pub_resume.publish(Bool(data=True))
                self.mark("resume_sent", t)
            if t >= te + 40:
                self.ev["end"] = t
        if n == "remote_lost":
            self.hb_on = not (te <= t < te + 6)
            if not self.hb_on:
                self.mark("remote_signal_lost", t, speed_mps=round(v, 2))
            if "remote_signal_lost" in self.ev and st == "STOP_REMOTE_LOST":
                self.mark("stop_detected", t)
            if "remote_signal_lost" in self.ev and v < 0.05:
                self.mark("standstill", t)
            if t >= te + 9 and "resume_sent" not in self.ev:
                self.pub_resume.publish(Bool(data=True))
                self.mark("resume_sent", t)
            if t >= te + 40:
                self.ev["end"] = t
        if n == "rtk_loss":
            self.quality = "FLOAT" if te <= t < te + 15 else "FIXED"
            if self.quality == "FLOAT":
                self.mark("rtk_lost", t, speed_mps=round(v, 2))
            if "rtk_lost" in self.ev and st == "STOP_RTK":
                self.mark("stop_detected", t)
            if "rtk_lost" in self.ev and v < 0.05:
                self.mark("standstill", t)
            if t >= te + 15:
                self.mark("rtk_back", t)
            if "rtk_back" in self.ev and st == "RUN" and v > 0.3:
                self.mark("resumed", t)
            if t >= te + 60:
                self.ev["end"] = t
        if n == "steering_fault":
            if t >= te:                         # steering motor jams at 8 deg left
                self.pub_stuck.publish(Float64(data=math.radians(8.0)))
                self.mark("steering_fault", t, stuck_at_deg=8.0)
            if "steering_fault" in self.ev and st.startswith("STOP_"):
                self.mark("stop_detected", t, state=st)
            if "stop_detected" in self.ev and v < 0.05:
                self.mark("standstill", t)
            if "standstill" in self.ev and t - self.ev["standstill"] > 5:
                self.ev["end"] = t

    def log(self, t):
        gx, gy, gyaw, gv = self.gt
        ex, ey, eyaw = self.est if self.est else (math.nan, math.nan, math.nan)
        gap = self.obstacle_gap_true()
        self.rows.append([round(t, 2), gx, gy, gyaw, gv, ex, ey, eyaw, self.vstate[0], self.vstate[1],
                          self.safety.get("state"), self.safety.get("reason"), self.safety.get("gap"), self.rtk,
                          int(self.work), self.trk.idx, self.status.get("kind", ""), self.trk.xte(gx, gy),
                          "" if gap is None else round(gap, 3),
                          "" if self.obst is None else round(self.obst[1], 3), "" if self.obst is None else round(self.obst[2], 3)])

    # ------------------------------------------------------------ results
    def finish(self, t):
        self.finished = True
        cols = ["t", "gt_x", "gt_y", "gt_yaw", "gt_v", "est_x", "est_y", "est_yaw", "steer", "v_cmd_actuator",
                "state", "reason", "lidar_gap", "rtk", "planter_down", "path_idx", "segment", "xte_true",
                "obstacle_gap_true", "obstacle_x", "obstacle_y"]
        with open(self.run_dir / "log.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            w.writerows(self.rows)
        m = self.metrics(t)
        (self.run_dir / "metrics.json").write_text(json.dumps(m, indent=1))
        try:
            from .report import plot_run
            plot_run(self.run_dir, self.path, self.field, self.rows, m)
        except Exception as e:   # plots are nice-to-have
            self.get_logger().warn(f"plot failed: {e}")
        self.get_logger().info("RESULT " + json.dumps(m))
        raise SystemExit(0)

    def metrics(self, t_end):
        a = np.array([[r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[14], r[17]] for r in self.rows], dtype=float)
        tcol, gx, gy, gyaw, gv, ex, ey, eyaw, work, xte = a.T
        states = [r[10] for r in self.rows]
        moving_work = (work > 0.5) & (gv > 0.3) & np.array([s == "RUN" for s in states])
        dist = float(np.sum(np.hypot(np.diff(gx), np.diff(gy))))
        m = {"scenario": self.name, "tractor": self.t_spec["name"], "tractor_id": self.tid,
             "implement": self.kit["implement"]["name"], "sim_time_s": round(float(t_end), 1),
             "distance_m": round(dist, 1), "events": self.events}
        if np.any(moving_work):
            e = np.abs(xte[moving_work])
            m["xte_planting_cm"] = {"mean": round(100 * float(e.mean()), 2), "rms": round(100 * float(np.sqrt((e ** 2).mean())), 2),
                                    "p95": round(100 * float(np.percentile(e, 95)), 2), "max": round(100 * float(e.max()), 2)}
        ok = np.isfinite(ex)
        if np.any(ok):
            pe = np.hypot(ex[ok] - gx[ok], ey[ok] - gy[ok])
            he = np.degrees(np.abs(np.arctan2(np.sin(eyaw[ok] - gyaw[ok]), np.cos(eyaw[ok] - gyaw[ok]))))
            m["gnss_error"] = {"pos_rms_cm": round(100 * float(np.sqrt((pe ** 2).mean())), 2), "heading_rms_deg": round(float(np.sqrt((he ** 2).mean())), 3),
                               "note": "includes the 10 Hz update lag of a moving tractor"}
        stops = {}
        prev = None
        for s in states:
            if s != prev and s and s.startswith("STOP_"):
                stops[s] = stops.get(s, 0) + 1
            prev = s
        m["stops"] = stops
        gaps = [r[18] for r in self.rows if r[18] != ""]
        if gaps:
            m["min_gap_to_obstacle_m"] = round(float(min(gaps)), 2)
            m["contact"] = bool(min(gaps) <= 0.0)
        if self.name in ("remote_estop", "remote_lost", "rtk_loss", "steering_fault"):
            k0 = {"remote_estop": "estop_pressed", "remote_lost": "remote_signal_lost", "rtk_loss": "rtk_lost",
                  "steering_fault": "steering_fault"}[self.name]
            if k0 in self.ev and "standstill" in self.ev:
                t0, t1 = self.ev[k0], self.ev["standstill"]
                sel = (tcol >= t0) & (tcol <= t1)
                m["time_to_standstill_s"] = round(t1 - t0, 2)
                m["distance_to_standstill_m"] = round(float(np.sum(np.hypot(np.diff(gx[sel]), np.diff(gy[sel])))), 2)
        if self.name == "steering_fault":
            m["max_xte_m"] = round(float(np.max(np.abs(xte))), 2)
            fps = [Polygon(footprint_corners(r[1], r[2], r[3], self.g["x_front"], -2.85, 2.3)) for r in self.rows[::5]]
            m["max_outside_field_m"] = round(max(0.0, *[max(Point(c).distance(self.field) for c in fp.exterior.coords) for fp in fps]), 2)
        if self.name == "full_field":
            m.update(self.coverage(gx, gy, gyaw, work, gv))
        return m

    def coverage(self, gx, gy, gyaw, work, gv):
        w = self.kit["implement"]["working_width_m"]
        back = 0.95 + 0.85
        wx, wy = gx - back * np.cos(gyaw), gy - back * np.sin(gyaw)
        runs, cur = [], []
        for i in range(len(wx)):
            if work[i] > 0.5 and gv[i] > 0.05:
                cur.append((wx[i], wy[i]))
            elif len(cur) > 1:
                runs.append(cur)
                cur = []
            else:
                cur = []
        if len(cur) > 1:
            runs.append(cur)
        strips = [LineString(r).buffer(w / 2, cap_style=2, join_style=2) for r in runs]
        if not strips:
            return {"coverage_pct": 0.0}
        u = unary_union(strips)
        covered = u.intersection(self.field).area
        total = sum(s.area for s in strips)
        return {"field_ha": round(self.field.area / 1e4, 3), "coverage_pct": round(100 * covered / self.field.area, 2),
                "overlap_pct": round(100 * (total - u.area) / max(u.area, 1e-9), 2),
                "planted_outside_field_m2": round(u.difference(self.field).area, 1),
                "planting_runs": len(runs)}


def main():
    rclpy.init()
    node = ScenarioRunner()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
