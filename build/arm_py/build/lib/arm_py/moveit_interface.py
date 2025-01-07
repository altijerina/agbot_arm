import rclpy
from rclpy.logging import get_logger
import numpy as np
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

def move_robot():
    ag_arm = MoveItPy(node_name = "moveit_py")
    
    ag_arm_rail = ag_arm.get_planning_component("rails")
    ag_arm_arm = ag_arm.get_planning_component("arm")
    ag_arm_gripper = ag_arm.get_planning_component("gripper")
    
    rail_state = RobotState(ag_arm.get_robot_model())
    arm_state = RobotState(ag_arm.get_robot_model())
    gripper_state = RobotState(ag_arm.get_robot_model())
    
    arm_state.set_joint_group_positions("rails", np.array([0.572, 1.143]))
    arm_state.set_joint_group_positions("arm", np.array([1.503, -0.331, 0.858, -0.569]))
    gripper_state.set_joint_group_positions("gripper", np.array([-0.651, 0.651]))
    
    ag_arm_rail.set_start_state_to_current_state()
    ag_arm_arm.set_start_state_to_current_state()
    ag_arm_gripper.set_start_state_to_current_state()
    
    ag_arm_rail.set_goal_state(robot_state = rail_state)
    ag_arm_arm.set_goal_state(robot_state = arm_state)
    ag_arm_gripper.set_goal_state(robot_state = gripper_state)
    
    rail_plan_result = ag_arm_rail.plan()
    arm_plan_result = ag_arm_arm.plan()
    gripper_plan_result = ag_arm_gripper.plan()
    
    if rail_plan_result:
        ag_arm.execute(rail_plan_result.trajectory, controllers = [])
    else:
        get_logger("rclpy").error("Rail plan failed.")
        
    if arm_plan_result:
        ag_arm.execute(arm_plan_result.trajectory, controllers = [])
    else:
        get_logger("rclpy").error("Arm plan failed.")
        
    if gripper_plan_result:
        ag_arm.execute(gripper_plan_result.trajectory, controllers = [])
    else:
        get_logger("rclpy").error("Gripper plan failed.")        
        
def main():
    rclpy.init()
    move_robot()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()