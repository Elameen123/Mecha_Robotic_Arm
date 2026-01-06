#!/usr/bin/env python3
"""
Precision Pick and Place for Mecha Robot
Fixed for: /gripper_action_controller/gripper_cmd
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# IMPORTS FOR ARM
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# IMPORTS FOR GRIPPER (New!)
from control_msgs.action import GripperCommand

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # 1. SETUP ARM CLIENT (Trajectory)
        self.arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )

        # 2. SETUP GRIPPER CLIENT (GripperCommand)
        # UPDATED: Matches your 'ros2 action list' output
        self.gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/gripper_action_controller/gripper_cmd'
        )

        self.get_logger().info('Waiting for controllers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Connected to both controllers!')

        # 3. JOINT NAMES
        self.arm_joints = [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
            'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
        ]

        # 4. POSITIONS
        self.home_pos =      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Pre-Grasp: Rotate Base (0.49) & Lean Forward
        self.pre_grasp_pos = [0.49, 0.5, 0.5, 0.0, 1.57, 0.0]
        # Grasp: Dive down
        self.grasp_pos =     [0.49, 0.8, 0.8, 0.0, 1.57, 0.0]
        # Drop: Rotate Right (-1.57)
        self.drop_pos =      [-1.57, 0.5, 0.5, 0.0, 1.57, 0.0]

        # Start
        self.execute_sequence()

    def send_arm(self, positions, duration=4):
        """Moves the arm using Joint Trajectory"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=duration)
        goal.trajectory.points = [pt]
        
        self.get_logger().info(f'Moving Arm...')
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(duration + 0.5)

    def send_gripper(self, position):
        """
        Moves the gripper using GripperCommand.
        position: 0.0 (Open) to ~0.7 (Closed) depending on your URDF limits.
        """
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 5.0 # Force limit
        
        self.get_logger().info(f'Gripper Move: {position}')
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(2.0)

    def execute_sequence(self):
        self.get_logger().info('--- STARTING ---')

        # 1. Home & Open
        self.send_arm(self.home_pos)
        self.send_gripper(0.0) # Open

        # 2. Pre-Grasp
        self.send_arm(self.pre_grasp_pos)

        # 3. Grasp
        self.send_arm(self.grasp_pos)
        self.send_gripper(0.5) # Close (Try 0.5 or 0.7)

        # 4. Lift
        self.send_arm(self.pre_grasp_pos)

        # 5. Drop
        self.send_arm(self.drop_pos)
        self.send_gripper(0.0) # Open

        # 6. Home
        self.send_arm(self.home_pos)
        self.get_logger().info('--- DONE ---')

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
