#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped

from moveit_msgs.action import MoveGroup

from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    BoundingVolume
)

from shape_msgs.msg import SolidPrimitive

from skai_interfaces.msg import TargetPose

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState


class MoveItController(Node):

    def __init__(self):

        super().__init__(
            "moveit_controller"
        )

        self.subscription = \
            self.create_subscription(
                TargetPose,
                "/target_pose",
                self.target_callback,
                10
            )

        self.move_group_client = \
            ActionClient(
                self,
                MoveGroup,
                "/move_action"
            )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/planned_trajectory",
            10
        )

        self.current_joint_state = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.get_logger().info(
            "MoveIt controller ready"
        )

    def joint_state_callback(self, msg):

        self.current_joint_state = msg

    def target_callback(self, msg):

        self.get_logger().info(
            f"Received target:"
        )

        self.get_logger().info(
            f"X={msg.x} "
            f"Y={msg.y} "
            f"Z={msg.z}"
        )

        self.send_goal(msg)

        def send_goal(self, msg):

            if self.current_joint_state is None:

                self.get_logger().error(
                    "No joint states received yet"
                )

                return

            goal_msg = MoveGroup.Goal()

            goal_request = \
                goal_msg.request

            goal_request.group_name = "arm"

            goal_request.num_planning_attempts = 10

            goal_request.allowed_planning_time = 5.0

            # IMPORTANT
            # Plan only, do not execute
            goal_msg.planning_options.plan_only = True

            #
            # USE ACTUAL ENCODER FEEDBACK AS START STATE
            #

            goal_request.start_state.joint_state = \
                self.current_joint_state

            constraints = Constraints()

            position_constraint = \
                PositionConstraint()

            position_constraint.header.frame_id = "BASE"

            position_constraint.link_name = "J_6"

            primitive = SolidPrimitive()

            primitive.type = \
                SolidPrimitive.SPHERE

            primitive.dimensions = [0.01]

            bounding_volume = BoundingVolume()

            bounding_volume.primitives.append(
                primitive
            )

            target_pose = PoseStamped()

            target_pose.header.frame_id = "BASE"

            target_pose.pose.position.x = msg.x
            target_pose.pose.position.y = msg.y
            target_pose.pose.position.z = msg.z

            target_pose.pose.orientation.w = 1.0

            bounding_volume.primitive_poses.append(
                target_pose.pose
            )

            position_constraint.constraint_region = \
                bounding_volume

            position_constraint.weight = 1.0

            constraints.position_constraints.append(
                position_constraint
            )

            goal_request.goal_constraints.append(
                constraints
            )

            if not self.move_group_client.wait_for_server(timeout_sec=5.0):

                self.get_logger().error(
                    "MoveGroup action server not available"
                )

                return

            self.get_logger().info(
                "Sending planning request..."
            )

            future = \
                self.move_group_client.send_goal_async(
                    goal_msg
                )

            future.add_done_callback(
                self.goal_response_callback
            )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                "Goal rejected"
            )

            return

        self.get_logger().info(
            "Goal accepted"
        )

        result_future = \
            goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result().result

        error_code = result.error_code.val

        if error_code != 1:

            self.get_logger().error(
                f"Planning failed — MoveIt error code: {error_code}"
            )

            return

        traj = (
            result.planned_trajectory
            .joint_trajectory
        )

        self.traj_pub.publish(traj)

        self.get_logger().info(
            f"Published trajectory with "
            f"{len(traj.points)} points"
        )

    def main_loop(self):
        pass


def main():

    rclpy.init()

    node = MoveItController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()