#!/usr/bin/env python
import os
import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from launch.actions import OpaqueFunction
import xacro


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    ns = LaunchConfiguration("ns").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)

    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot.xacro",
    )

    robot_description = xacro.process_file(
        robot_xacro_path, mappings={"hw_env": "webots"}
    ).toxml()
    spawn_robot = URDFSpawner(
        name=robot_name,
        robot_description=robot_description,
        # relative_path_prefix=os.path.join(robot_name + "_description", 'resource'),
        translation="0 0 0.47",
        rotation="0 0 0 0",
    )

    terrain = LaunchConfiguration("terrain").perform(context)
    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [FindPackageShare("webots_bridge"), "worlds", terrain + ".wbt"]
        ),
        ros2_supervisor=True,
    )

    tita_driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            config_file,
        ],
        respawn=True,
        namespace=ns,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
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

    def get_ros2_nodes(*args):
        return [
            spawn_robot,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessIO(
                    target_action=spawn_robot,
                    on_stdout=lambda event: get_webots_driver_node(
                        event,
                        [
                            robot_state_pub_node,
                            tita_driver,
                            joint_state_broadcaster_spawner,
                            launch.actions.RegisterEventHandler(
                                event_handler=launch.event_handlers.OnProcessExit(
                                    target_action=joint_state_broadcaster_spawner,
                                    on_exit=[imu_sensor_broadcaster_spawner],
                                )
                            ),
                            launch.actions.RegisterEventHandler(
                                event_handler=launch.event_handlers.OnProcessExit(
                                    target_action=imu_sensor_broadcaster_spawner,
                                    on_exit=[joint_command_controller_spawner],
                                )
                            ),
                        ],
                    ),
                )
            ),
        ]

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return [
        webots,
        webots._supervisor,
        webots_event_handler,
    ] + get_ros2_nodes()


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
            default_value=os.path.join(
                get_package_share_directory("d1_deploy"),
                "config",
                "deploy.yaml",
            ),
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
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "terrain",
            default_value="empty_world",
            description="Webots world file (worlds/<terrain>.wbt)",
            choices=["empty_world", "stairs", "uneven", "terrain"],
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
