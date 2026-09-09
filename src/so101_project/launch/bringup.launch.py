#!/usr/bin/env python3
"""One entry point for all three SO-101 backends.

    ros2 launch so101_project bringup.launch.py hardware_type:=mock
    ros2 launch so101_project bringup.launch.py hardware_type:=mujoco
    ros2 launch so101_project bringup.launch.py hardware_type:=real usb_port:=/dev/ttyACM0

What differs per backend:
    mock    plain controller_manager/ros2_control_node,   use_sim_time = false
    mujoco  mujoco_ros2_control/ros2_control_node,         use_sim_time = true  (/clock from MuJoCo)
    real    plain controller_manager/ros2_control_node,   use_sim_time = false

Everything above the hardware interface - controller names, action namespaces, MoveIt
groups - is identical in all three, which is the whole point of this project: the goal
you plan in mock is executed unchanged in MuJoCo and on the real arm.

The node layout follows mujoco_ros2_control_demos/launch/01_basic_robot.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from moveit_configs_utils import MoveItConfigsBuilder

PROJECT_PKG = "so101_project"
MOVEIT_PKG = "so101_project_moveit_config"
VALID_BACKENDS = ("mock", "mujoco", "real")


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    hardware_type = arg("hardware_type")
    if hardware_type not in VALID_BACKENDS:
        raise RuntimeError(
            f"hardware_type must be one of {VALID_BACKENDS}, got '{hardware_type}'"
        )

    project_share = get_package_share_directory(PROJECT_PKG)
    xacro_file = os.path.join(project_share, "urdf", "so101.urdf.xacro")
    controllers_file = os.path.join(project_share, "config", "controllers.yaml")

    mujoco_model = arg("mujoco_model")
    if hardware_type == "mujoco":
        if not mujoco_model:
            mujoco_model = os.path.join(project_share, "mjcf", "scene.xml")
        if not os.path.isfile(mujoco_model):
            raise RuntimeError(
                f"MJCF scene not found: {mujoco_model}\n"
                "Generate it first: scripts/03_make_mjcf.sh, then hand-edit actuators "
                "and the initial keyframe (see docs)."
            )

    use_sim_time = hardware_type == "mujoco"

    # ------------------------------------------------------------------ MoveIt config
    # SRDF, self-collision matrix, joint limits, OMPL and pick_ik settings are reused
    # from the upstream package; only moveit_controllers.yaml was replaced.
    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name=MOVEIT_PKG)
        .robot_description(
            file_path=xacro_file,
            mappings={
                "hardware_type": hardware_type,
                "mujoco_model": mujoco_model,
                "mujoco_headless": arg("mujoco_headless"),
                "mujoco_keyframe": arg("mujoco_keyframe"),
                "usb_port": arg("usb_port"),
                "joint_config_file": arg("joint_config_file"),
                "variant": "follower",
            },
        )
        .robot_description_semantic(file_path="config/so101_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    common_time = {"use_sim_time": use_sim_time}
    nodes = []

    # -------------------------------------------------------- robot_state_publisher
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[moveit_config.robot_description, common_time],
        )
    )

    # --------------------------------------------------------- ros2_control node
    # MuJoCo needs the slightly modified node shipped by mujoco_ros2_control; the
    # plain controller_manager node cannot step the simulation or publish /clock.
    control_node_pkg = (
        "mujoco_ros2_control" if hardware_type == "mujoco" else "controller_manager"
    )
    control_node = Node(
        package=control_node_pkg,
        executable="ros2_control_node",
        emulate_tty=True,
        output="both",
        parameters=[ParameterFile(controllers_file), common_time],
        # Humble passes the description over ~/robot_description; Jazzy+ reads /robot_description.
        remappings=(
            [("~/robot_description", "/robot_description")]
            if os.environ.get("ROS_DISTRO") == "humble"
            else []
        ),
        on_exit=Shutdown(),
    )
    nodes.append(control_node)

    # ------------------------------------------------------------------- spawners
    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--param-file",
                controllers_file,
                "--controller-manager-timeout",
                "60",
            ],
            output="both",
        )

    jsb = spawner("joint_state_broadcaster")
    nodes.append(jsb)
    # Load the trajectory controllers only after joint states are being published, so a
    # failure order is unambiguous when something goes wrong.
    nodes.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=jsb,
                on_exit=[spawner("arm_controller"), spawner("gripper_controller")],
            )
        )
    )

    # --------------------------------------------------------------------- MoveIt
    if arg("launch_moveit").lower() in ("true", "1"):
        nodes.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    common_time,
                    # let clients (and group_goal_client.py) read the SRDF from a topic
                    {"publish_robot_description_semantic": True},
                ],
            )
        )

    if arg("use_rviz").lower() in ("true", "1"):
        rviz_config = os.path.join(
            get_package_share_directory(MOVEIT_PKG), "config", "moveit.rviz"
        )
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=["-d", rviz_config] if os.path.isfile(rviz_config) else [],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    common_time,
                ],
            )
        )

    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "hardware_type",
            default_value="mock",
            choices=list(VALID_BACKENDS),
            description="mock: no physics | mujoco: physics sim | real: Feetech servos",
        ),
        DeclareLaunchArgument(
            "mujoco_model",
            default_value="",
            description="Absolute path to the MJCF scene. Empty = <share>/mjcf/scene.xml",
        ),
        DeclareLaunchArgument(
            "mujoco_headless",
            default_value="false",
            description="Run MuJoCo without its viewer window",
        ),
        DeclareLaunchArgument(
            "mujoco_keyframe",
            default_value="rest",
            description='MJCF keyframe to start from (scene.xml defines "rest"); "" to skip',
        ),
        DeclareLaunchArgument(
            "usb_port",
            default_value="/dev/ttyACM0",
            description="Serial port of the follower arm (hardware_type:=real only)",
        ),
        DeclareLaunchArgument(
            "joint_config_file",
            default_value="",
            description="Optional upstream Feetech joint override file (real only)",
        ),
        DeclareLaunchArgument(
            "launch_moveit", default_value="true", description="Start move_group"
        ),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz2"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
