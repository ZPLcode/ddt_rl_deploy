#!/usr/bin/env python
import os
import xacro
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    ns = LaunchConfiguration("ns").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    if not config_file:
        raise RuntimeError(
            "config_file is required: pass config_file:=<path>/deploy.yaml "
            "(scripts/run_sim.sh sets it automatically)")

    # Get world file path
    world_file = os.path.join(
        FindPackageShare("gazebo_bridge").find("gazebo_bridge"),
        "worlds",
        "empty_world.world",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={
            "world": world_file,
            "pause": "false",
            "verbose": "false",
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            f"{ns}/robot_description",
            "-entity",
            f"{ns}",
            "-robot_namespace",
            f"{ns}",
            "-x",
            "0.",
            "-y",
            "0.",
            "-z",
            "0.47",
        ],
        output="screen",
    )

    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot.xacro",
    )

    robot_description = xacro.process_file(
        robot_xacro_path,
        mappings={"hw_env": "gazebo", "controller_config": config_file},
    ).toxml()

    # Replace package:// URIs for the description packages present in this repo
    description_packages = ["d1_description"]
    for desc_pkg in description_packages:
        robot_description = robot_description.replace(
            "package://" + desc_pkg,
            "file://" + get_package_share_directory(desc_pkg),
        )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
            {"publish_frequency": 15.0},
            {"frame_prefix": ns + "/"},
        ],
        namespace=ns,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ns + "/controller_manager",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            ns + "/controller_manager",
        ],
    )
    # passthrough controller: consumes command/joint_command from rl_inference.py
    joint_command_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_command_controller",
            "--controller-manager",
            ns + "/controller_manager",
        ],
    )
    nodes = [
        robot_state_pub_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        joint_command_controller_spawner,
    ]

    return nodes


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "robot",
            default_value="d1",
            description="robot name -> <robot>_description package",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "config_file",
            default_value="",   # config is a plain folder, not a package; run_sim.sh passes the path
            description="Shared robot deployment YAML",
        )
    )
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "ns",
            default_value="",
            description="Namespace of launch",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
