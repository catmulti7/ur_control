from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

# Initialize Arguments
ur_type = LaunchConfiguration("ur_type")
use_fake_hardware = LaunchConfiguration("use_fake_hardware")
safety_limits = LaunchConfiguration("safety_limits")
safety_pos_margin = LaunchConfiguration("safety_pos_margin")
safety_k_position = LaunchConfiguration("safety_k_position")
# General arguments
description_package = LaunchConfiguration("description_package")
description_file = LaunchConfiguration("description_file")
moveit_config_package = LaunchConfiguration("moveit_config_package")
moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
moveit_config_file = LaunchConfiguration("moveit_config_file")
warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
prefix = LaunchConfiguration("prefix")
use_sim_time = LaunchConfiguration("use_sim_time")
launch_rviz = LaunchConfiguration("launch_rviz")
launch_servo = LaunchConfiguration("launch_servo")

joint_limit_params = PathJoinSubstitution(
    [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
)
kinematics_params = PathJoinSubstitution(
    [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
)
physical_params = PathJoinSubstitution(
    [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
)
visual_params = PathJoinSubstitution(
    [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
)

robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " ",
        "robot_ip:=xxx.yyy.zzz.www",
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
        safety_limits,
        " ",
        "safety_pos_margin:=",
        safety_pos_margin,
        " ",
        "safety_k_position:=",
        safety_k_position,
        " ",
        "name:=",
        "ur",
        " ",
        "ur_type:=",
        ur_type,
        " ",
        "script_filename:=ros_control.urscript",
        " ",
        "input_recipe_filename:=rtde_input_recipe.txt",
        " ",
        "output_recipe_filename:=rtde_output_recipe.txt",
        " ",
        "prefix:=",
        prefix,
        " ",
    ]
)
robot_description = {"robot_description": robot_description_content}

# MoveIt Configuration
robot_description_semantic_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
        ),
        " ",
        "name:=",
        # Also ur_type parameter could be used but then the planning group names in yaml
        # configs has to be updated!
        "ur",
        " ",
        "prefix:=",
        prefix,
        " ",
    ]
)
robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

robot_description_kinematics = PathJoinSubstitution(
    [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
)


def generate_launch_description():

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="robot_control",
        package="robot_control",
        executable="robot_control",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_demo])