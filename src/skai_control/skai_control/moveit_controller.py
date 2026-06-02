#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped

from moveit_msgs.action import MoveGroup

from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume
)

from shape_msgs.msg import SolidPrimitive

from skai_interfaces.msg import TargetPose



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



        self.get_logger().info(
            "MoveIt controller ready"
        )



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



        goal_msg = MoveGroup.Goal()



        goal_request = \
            goal_msg.request



        goal_request.group_name = "arm"



        goal_request.num_planning_attempts = 10



        goal_request.allowed_planning_time = 5.0



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



        orientation_constraint = \
            OrientationConstraint()



        orientation_constraint.header.frame_id = \
            "BASE"



        orientation_constraint.link_name = "J_6"



        orientation_constraint.orientation.w = 1.0



        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1



        orientation_constraint.weight = 1.0



        constraints.orientation_constraints.append(
            orientation_constraint
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
            "Sending goal..."
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



        self.get_logger().info(
            "Motion completed"
        )



def main():



    rclpy.init()



    node = MoveItController()



    rclpy.spin(node)



    rclpy.shutdown()



if __name__ == "__main__":

    main()
