"""Full sim: Gazebo (field + MF tractor with kit) + ROS 2 bridge + autonomy nodes + scenario runner.

  ros2 launch jbj_sim sim.launch.py scenario:=full_field tractor:=mf_5m_145 gui:=false
"""
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from jbj_sim import planner, spec
from jbj_sim.common import save_plan


def setup(context):
    share = Path(get_package_share_directory("jbj_sim"))
    scenario = LaunchConfiguration("scenario").perform(context)
    tid, t, g = spec.tractor(LaunchConfiguration("tractor").perform(context) or None)
    gui = LaunchConfiguration("gui").perform(context) == "true"
    run_dir = Path(os.path.expanduser(LaunchConfiguration("runs").perform(context))) / f"{scenario}_{tid}"
    kit, field = spec.load("kit"), spec.load("field")
    plan = planner.plan_field(field, kit, g)
    chk = planner.check_plan(plan, g, kit, kit["safety"]["geofence_margin_m"])
    save_plan(run_dir, plan, {"tractor": tid, "scenario": scenario, "check": chk})
    x0, y0, yaw0 = plan["path"][0][:3]
    world = str(share / "worlds" / "field.sdf")
    gz_cmd = ["gz", "sim", "-r", "-v", "2", world] + ([] if gui else ["-s", "--headless-rendering"])
    common = {"use_sim_time": True, "tractor": tid, "run_dir": str(run_dir)}
    bridge = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
              "/gnss/left@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
              "/gnss/right@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
              "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
              "/model/tractor/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
              "/model/tractor/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry",
              "/world/field/create@ros_gz_interfaces/srv/SpawnEntity",
              "/world/field/set_pose@ros_gz_interfaces/srv/SetEntityPose"]
    runner = Node(package="jbj_sim", executable="scenario_runner", output="screen",
                  parameters=[dict(common, scenario=scenario)])
    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share / "models")),
        ExecuteProcess(cmd=gz_cmd, output="log"),
        Node(package="ros_gz_sim", executable="create", output="log",
             arguments=["-world", "field", "-file", str(share / "models" / tid / "model.sdf"), "-name", "tractor",
                        "-x", f"{x0:.3f}", "-y", f"{y0:.3f}", "-z", "0.05", "-Y", f"{yaw0:.4f}"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=bridge, output="log",
             parameters=[{"use_sim_time": True}]),
        Node(package="jbj_sim", executable="gnss_receiver", parameters=[common], output="log"),
        Node(package="jbj_sim", executable="path_follower", parameters=[common], output="log"),
        Node(package="jbj_sim", executable="safety_supervisor", parameters=[common], output="log"),
        Node(package="jbj_sim", executable="vehicle_interface", parameters=[common], output="log"),
        runner,
        RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown(reason="scenario done"))])),
    ]
    print(f"[jbj] {scenario} on {tid}: plan {chk['length_m']:.0f} m, start ({x0:.1f}, {y0:.1f}) yaw {math.degrees(yaw0):.0f} deg, "
          f"plan footprint max outside edge {chk['max_outside_m']:.2f} m -> {run_dir}")
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="full_field"),
        DeclareLaunchArgument("tractor", default_value=""),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("runs", default_value="~/jbj_runs"),
        OpaqueFunction(function=setup),
    ])
