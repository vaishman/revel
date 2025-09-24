#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import (
    MoveGroupCommander,
    roscpp_initialize,
    roscpp_shutdown,
)


class MoveItArmDemo(Node):
    def __init__(self):
        super().__init__("moveit_two_point_demo")

        # Initialize MoveIt
        roscpp_initialize(sys.argv)

        # Arm MoveGroup
        self.arm = MoveGroupCommander("panda_arm")

        self.get_logger().info("MoveItArmDemo initialized")

    def go_to_pose(self, pose: PoseStamped, label: str):
        """Plan and execute a motion to the given pose"""
        self.arm.set_pose_target(pose)

        self.get_logger().info(f"Planning motion to {label}...")
        success = self.arm.go(wait=True)

        # Stop and clear targets
        self.arm.stop()
        self.arm.clear_pose_targets()

        if success:
            self.get_logger().info(f"Reached {label} successfully ✅")
        else:
            self.get_logger().warn(f"Planning to {label} failed ❌")


def main(args=None):
    rclpy.init(args=args)
    node = MoveItArmDemo()

    # -------- First target pose --------
    pose1 = PoseStamped()
    pose1.header.frame_id = "world"
    pose1.header.stamp = node.get_clock().now().to_msg()
    pose1.pose.position.x = 0.4
    pose1.pose.position.y = 0.0
    pose1.pose.position.z = 0.4
    pose1.pose.orientation.w = 1.0  # neutral orientation

    # -------- Second target pose --------
    pose2 = PoseStamped()
    pose2.header.frame_id = "world"
    pose2.header.stamp = node.get_clock().now().to_msg()
    pose2.pose.position.x = 0.5
    pose2.pose.position.y = -0.2
    pose2.pose.position.z = 0.3
    pose2.pose.orientation.w = 1.0

    # Move between two points
    node.go_to_pose(pose1, "Pose 1")
    node.go_to_pose(pose2, "Pose 2")
    node.go_to_pose(pose1, "Pose 1 (return)")

    rclpy.shutdown()
    roscpp_shutdown()


if __name__ == "__main__":
    main()

