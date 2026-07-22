# Mujoco sim with the topic_command_controller bridge instead of rl_controller:
# gives the no-ROS SDK the same command/joint_command seam as the real robot.
import os

import launch
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    ns = LaunchConfiguration("ns").perform(context)
    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot_cargo_out.xacro",
    )
    robot_description = xacro.process_file(
        robot_xacro_path, mappings={"hw_env": "webots"}
    ).toxml()

    robot_description = robot_description.replace(
        "<plugin>tita_webots_ros2_control::WebotsBridge</plugin>",
        "<plugin>mujoco_ros2_control/MujocoSystem</plugin>",
    )

    robot_controllers = os.path.join(
        get_package_share_directory("topic_command_controller"),
        "config",
        robot_name + ".yaml",
    )

    mujoco_simulate_app = Node(
        package="mujoco_sim_ros2",
        executable="mujoco_sim",
        parameters=[
            {"model_package": robot_name + "_description"},
            {"model_file": "mujoco/scene_cargo_out.xml"},
            {"physics_plugins": ["mujoco_ros2_control::MujocoRos2ControlPlugin"]},
            robot_controllers,
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", ns + "/controller_manager"],
    )
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", ns + "/controller_manager"],
    )
    topic_command_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["topic_command_controller", "--controller-manager", ns + "/controller_manager"],
    )

    return [
        mujoco_simulate_app,
        robot_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_simulate_app,
                # controller_manager only exists after mujoco receives
                # robot_description over a topic; spawning immediately races it
                on_start=[TimerAction(period=5.0,
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
            # strictly sequential: two spawners hitting the CM in parallel race
            # its load/configure services and one of them dies
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
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
