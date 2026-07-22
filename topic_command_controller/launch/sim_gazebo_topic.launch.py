#!/usr/bin/env python
# Gazebo variant of sim_mujoco_topic.launch.py: vendor gazebo_bridge sim +
# topic_command_controller, so rl_inference.py drives the robot over
# command/joint_command exactly as in the mujoco sim.
import os
import xacro
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import (IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    ns = LaunchConfiguration("ns").perform(context)
    gui = LaunchConfiguration("gui").perform(context)

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
            "gui": gui,
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", f"{ns}/robot_description",
            "-entity", f"{ns}",
            "-robot_namespace", f"{ns}",
            "-x", "0.", "-y", "0.", "-z", "0.65",
        ],
        output="screen",
    )

    # Same embodiment the mujoco topic launch uses (cargo_out — matches
    # scene_cargo_out.xml and the deployed policies).
    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot_cargo_out.xacro",
    )
    robot_description = xacro.process_file(
        robot_xacro_path, mappings={"hw_env": "gazebo"}
    ).toxml()

    description_packages = ["d1_description", "d1h_description",
                            "tita_description", "titatit_description"]
    for desc_pkg in description_packages:
        robot_description = robot_description.replace(
            "package://" + desc_pkg,
            "file://" + get_package_share_directory(desc_pkg),
        )

    # Point the gazebo_ros2_control <parameters> at the topic bridge config
    # (controller_manager + broadcasters + topic_command_controller) instead
    # of the rl_controller FSM config the vendor launch uses.
    robot_description = robot_description.replace(
        get_package_share_directory("gazebo_bridge") + "/config/controllers.yaml",
        get_package_share_directory("topic_command_controller")
        + "/config/" + robot_name + ".yaml",
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
        arguments=["joint_state_broadcaster",
                   "--controller-manager", ns + "/controller_manager"],
    )
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", ns + "/controller_manager"],
    )
    topic_command_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["topic_command_controller",
                   "--controller-manager", ns + "/controller_manager"],
    )

    return [
        robot_state_pub_node,
        gazebo,
        spawn_entity,
        RegisterEventHandler(
            # controller_manager exists only after gazebo_ros2_control loads
            # inside gzserver, which happens when spawn_entity finishes.
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[TimerAction(period=3.0,
                                     actions=[joint_state_broadcaster_spawner])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[imu_sensor_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            # strictly sequential: parallel spawners race the CM services
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster_spawner,
                on_exit=[topic_command_controller_spawner],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        launch.actions.DeclareLaunchArgument(
            "robot", default_value="d1", description="Robot description package prefix"
        ),
        launch.actions.DeclareLaunchArgument(
            "ns", default_value="", description="Namespace of launch"
        ),
        launch.actions.DeclareLaunchArgument(
            "gui", default_value="true", description="Start gzclient (GUI)"
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
