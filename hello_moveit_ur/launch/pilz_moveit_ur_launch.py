import os
import sys

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("hello_moveit_ur"), "config", "ur10e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            "ur10e",
            "default_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            "ur10e",
            "physical_parameters.yaml",
        ]
    )
    visual_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            "ur10e",
            "visual_parameters.yaml",
        ]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            " ",
            "robot_ip:=10.216.71.90",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur10e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    return robot_description


def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic


def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    coordinates_x_arg = DeclareLaunchArgument(
        "coordinates_x",
        default_value="0.0",
        description="X-coordinate value for the robot",
    )
    coordinates_y_arg = DeclareLaunchArgument(
        "coordinates_y",
        default_value="0.0",
        description="Y-coordinate value for the robot",
    )
    coordinates_z_arg = DeclareLaunchArgument(
        "coordinates_z",
        default_value="0.0",
        description="Z-coordinate value for the robot",
    )
    vel_scale_arg = DeclareLaunchArgument(
        "vel_scale",
        default_value="0.1",
        description="Velocity scale for the robot",
    )
    acc_scale_arg = DeclareLaunchArgument(
        "acc_scale",
        default_value="0.1",
        description="Acceleration scale for the robot",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation time"
    )
    coordinates_x = LaunchConfiguration("coordinates_x")
    coordinates_y = LaunchConfiguration("coordinates_y")
    coordinates_z = LaunchConfiguration("coordinates_z")
    vel_scale = LaunchConfiguration("vel_scale")
    acc_scale = LaunchConfiguration("acc_scale")
    use_sim_time = LaunchConfiguration("use_sim_time")
    coordinates = {"x": coordinates_x, "y": coordinates_y, "z": coordinates_z}

    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    demo_node = Node(
        package="hello_moveit_ur",
        executable="pilz_moveit_ur",
        name="pilz_moveit_ur",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"use_sim_time": use_sim_time},
            {"coordinates": coordinates},
            {"vel_scale": vel_scale},
            {"acc_scale": acc_scale},
        ],
    )

    # Return the launch description with declared arguments and the node
    return launch.LaunchDescription(
        [
            coordinates_x_arg,
            coordinates_y_arg,
            coordinates_z_arg,
            vel_scale_arg,
            acc_scale_arg,
            use_sim_time_arg,
            demo_node,
        ]
    )
