from launch import LaunchDescription

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



    # ROS2 CONTROLLERS YAML

    ros2_controllers_path = os.path.join(

        get_package_share_directory(
            "skai_moveit_config"
        ),

        "config",

        "ros2_controllers.yaml"

    )



    # ROBOT STATE PUBLISHER

    robot_state_publisher = Node(

        package="robot_state_publisher",

        executable="robot_state_publisher",

        output="screen",

        parameters=[

            moveit_config.robot_description

        ]

    )



    # ROS2 CONTROL NODE

    ros2_control_node = Node(

        package="controller_manager",

        executable="ros2_control_node",

        parameters=[

            moveit_config.robot_description,

            ros2_controllers_path

        ],

        output="screen"

    )



    # JOINT STATE BROADCASTER

    joint_state_broadcaster_spawner = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[

            "joint_state_broadcaster",

            "--controller-manager",

            "/controller_manager"

        ],

        output="screen"

    )



    # ARM CONTROLLER

    arm_controller_spawner = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[

            "arm_controller",

            "--controller-manager",

            "/controller_manager"

        ],

        output="screen"

    )



    # MOVEIT CUSTOM NODE

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



        # MOVE GROUP

        generate_move_group_launch(
            moveit_config
        ),



        # ROBOT DESCRIPTION PUBLISHER

        robot_state_publisher,



        # ROS2 CONTROL

        ros2_control_node,



        # CONTROLLERS

        joint_state_broadcaster_spawner,

        arm_controller_spawner,



        # CUSTOM MOVEIT NODE

        moveit_controller_node

    ])