from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.actions import TimerAction
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch
)

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "skai_arm",
            package_name="skai_moveit_config"
        )
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory(
            "skai_moveit_config"
        ),
        "config",
        "ros2_controllers.yaml"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description
        ]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path
        ],
        output="screen"
    )

    # LOAD JOINT STATE BROADCASTER

    joint_state_loader = TimerAction(

        period=5.0,

        actions=[

            ExecuteProcess(

                cmd=[

                    "ros2",

                    "control",

                    "load_controller",

                    "--set-state",

                    "active",

                    "joint_state_broadcaster"

                ],

                output="screen"

            )

        ]

    )



    # LOAD ARM CONTROLLER

    arm_controller_loader = TimerAction(

        period=8.0,

        actions=[

            ExecuteProcess(

                cmd=[

                    "ros2",

                    "control",

                    "load_controller",

                    "--set-state",

                    "active",

                    "arm_controller"

                ],

                output="screen"

            )

        ]

    )

    moveit_controller_node = Node(
        package="skai_control",
        executable="moveit_controller",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ]
    )

    return LaunchDescription([

        generate_move_group_launch(
            moveit_config
        ),

        robot_state_publisher,

        ros2_control_node,

        # CONTROLLERS

        joint_state_loader,

        arm_controller_loader,

        moveit_controller_node

    ])
