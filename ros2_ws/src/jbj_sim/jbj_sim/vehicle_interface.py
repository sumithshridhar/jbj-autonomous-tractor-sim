"""Vehicle interface + actuator model (what the I/O board and the Sveaverken steering motor do on the real tractor).

Steering: rate-limited (front-wheel deg/s) with a first-order lag. Speed: acceleration/brake limited.
Output: Twist for Gazebo's Ackermann drive (v, w = v tan(delta) / L). A steering fault can be injected for tests.
"""
import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from . import spec

ACCEL_UP = 0.5    # m/s^2, ASSUMPTION: gentle pull-away in a field gear


class VehicleInterface(Node):
    def __init__(self):
        super().__init__("vehicle_interface")
        self.declare_parameter("tractor", "")
        _, _, g = spec.tractor(self.get_parameter("tractor").value or None)
        kit = spec.load("kit")
        self.L = g["L"]
        self.dmax = 0.95 * g["delta_eq_max"]
        self.rate = math.radians(kit["steering"]["front_wheel_rate_deg_s"])
        self.tau = kit["steering"]["steer_response_s"]
        self.brake = kit["safety"]["brake_decel_mps2"]
        self.delta = self.delta_rl = self.v = 0.0
        self.target = (0.0, 0.0)
        self.fault = 0.0
        self.stuck = None
        self.create_subscription(AckermannDriveStamped, "/vehicle/cmd", self.on_cmd, 10)
        self.create_subscription(Float64, "/sim/steer_fault", lambda m: setattr(self, "fault", m.data), 10)
        self.create_subscription(Float64, "/sim/steer_stuck", lambda m: setattr(self, "stuck", m.data), 10)
        self.pub = self.create_publisher(Twist, "/model/tractor/cmd_vel", 10)
        self.pub_state = self.create_publisher(Float64MultiArray, "/vehicle/state", 10)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)

    def on_cmd(self, m):
        self.target = (m.drive.steering_angle, m.drive.speed)

    def step(self):
        d_t, v_t = self.target
        d_t = max(-self.dmax, min(self.dmax, d_t + self.fault))
        if self.stuck is not None:          # jammed steering motor: wheels go to this angle whatever is commanded
            d_t = self.stuck
        step = self.rate * self.dt
        self.delta_rl += max(-step, min(step, d_t - self.delta_rl))          # steering motor speed limit
        self.delta += (self.delta_rl - self.delta) * min(1.0, self.dt / self.tau)   # motor + hydraulics lag
        dv = v_t - self.v
        lim = (ACCEL_UP if dv > 0 else self.brake) * self.dt
        self.v += max(-lim, min(lim, dv))
        tw = Twist()
        tw.linear.x = self.v
        tw.angular.z = self.v * math.tan(self.delta) / self.L
        self.pub.publish(tw)
        self.pub_state.publish(Float64MultiArray(data=[self.delta, self.v]))


def main():
    rclpy.init()
    rclpy.spin(VehicleInterface())


if __name__ == "__main__":
    main()
