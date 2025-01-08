#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from arm_msgs.action import ArmTask     
import numpy as np
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import time


class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.get_logger().info("Starting the Server")
        self.action_server = ActionServer(
            self, ArmTask, "task_server", self.goalCallback
        )
        self.agbot_arm = MoveItPy(node_name="moveit_py")
        self.agbot_arm_rails = self.agbot_arm.get_planning_component("rails")
        self.agbot_arm_arm = self.agbot_arm.get_planning_component("arm")
        self.agbot_arm_gripper = self.agbot_arm.get_planning_component("gripper")

    def goalCallback(self, goal_handle):
        self.get_logger().info(
            "Received goal request with task_number %d" % goal_handle.request.task_number
        )

        rail_state = RobotState(self.agbot_arm.get_robot_model())
        arm_state = RobotState(self.agbot_arm.get_robot_model())
        gripper_state = RobotState(self.agbot_arm.get_robot_model())
        
        rail_joint_goal = []
        arm_joint_goal = []
        gripper_joint_goal = []
        
        
        self.agbot_arm_rails.set_start_state_to_current_state()
        if self.agbot_arm_rails == np.array(0.0, 0.0):
            rail_joint_goal = np.array(0.572, 1.143)
            rail_state.set_joint_group_positions("rails", rail_joint_goal)
            self.agbot_arm_rails.set_goal_state(robot_state=rail_state)
            rail_plan_result = self.agbot_arm_rails.plan()
            self.agbot_arm.execute(rail_plan_result.trajectory, controllers = [])
            time.sleep(2)

        if goal_handle.request.task_number == 0:  #Home position
            arm_joint_goal = np.array(0.0, 0.0, 0.0, 0.0)
            gripper_joint_goal = np.array(0.0)
        elif goal_handle.request.task_number == 1: #Arm Ready
            arm_joint_goal = np.array(0.0, 1.502, -1.423, 0.0)
            gripper_joint_goal = np.array(-0.651)
        elif goal_handle.request.task_number == 2: #Arm Ready Low Left
            arm_joint_goal = np.array(-1.649, 2.696, -1.909, -0.772)
            gripper_joint_goal = np.array(-0.7)
        elif goal_handle.request.task_number == 3: #Arm Ready Low Right
            arm_joint_goal = np.array(1.128, 2.815, -2.10, -0.125)
            gripper_joint_goal = np.array(-0.2)
        elif goal_handle.request.task_number == 4: #Arm Ready High Right
            arm_joint_goal = np.array(1.503, 1.967, -2.378, 0.408)
            gripper_joint_goal = np.array(-0.018)
        elif goal_handle.request.task_number == 5: #Arm Ready High Left
            arm_joint_goal = np.array(1.128, 1.745, -2.078, 0.524)
            gripper_joint_goal = np.array(-0.9)
        else:
            self.get_logger().error("Invalid task number.")
            return
            
            
        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper",gripper_joint_goal)
        
        self.agbot_arm_arm.set_start_state_to_current_state()
        self.agbot_arm_gripper.set_start_state_to_current_state()
        
        self.agbot_arm_arm.set_goal_state(robot_state=arm_state)
        self.agbot_arm_gripper.set_goal_state(robot_state=gripper_state)
        
        arm_plan_result = self.agbot_arm_arm.plan()
        gripper_plan_result = self.agbot_arm_gripper.plan()
        
        if arm_plan_result:
            self.agbot_arm.execute(arm_plan_result.trajectory, controllers = [])
        else:
            self.get_logger().info("Arm Planner Failed.")
        if gripper_plan_result:
            self.agbot_arm.execute(gripper_plan_result.trajectory, controllers = [])
        else:
            self.get_logger().info("Gripper Planner Failed.")
            
        goal_handle.succeed()
        result = ArmTask.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)


if __name__ == "__main__":
    main()