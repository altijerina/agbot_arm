#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <vector>

#include <memory>


void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    //Create MoveGroupInterface for the move groups
    moveit::planning_interface::MoveGroupInterface rail_move_group(node, "rails");
    moveit::planning_interface::MoveGroupInterface arm_move_group(node, "arm");
    moveit::planning_interface::MoveGroupInterface gripper_move_group(node, "gripper");

    // Set the start state for the move groups
    rail_move_group.setStartStateToCurrentState();
    arm_move_group.setStartStateToCurrentState();
    gripper_move_group.setStartStateToCurrentState();

    //Define the target joint positions for the move groups
    std::vector<double> rail_joint_goal {0.572, 1.143};
    rail_move_group.setJointValueTarget(rail_joint_goal);

    std::vector<double> arm_joint_goal {1.128, 2.696, -1.909, -0.772};
    arm_move_group.setJointValueTarget(arm_joint_goal);

    std::vector<double> gripper_joint_goal {-0.7};
    gripper_move_group.setJointValueTarget(gripper_joint_goal);

    //Check if the move group goal is within joint limits.
    bool rail_within_bounds = (rail_joint_goal.size() == rail_move_group.getActiveJoints().size());
    if (!rail_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "RAIL Target joint position outside of limits.");
        return;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Rail Target joint position within joint limits.");
    }

    bool arm_within_bounds = (arm_joint_goal.size() == arm_move_group.getActiveJoints().size());
    if (!arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "ARM Target joint position outside of limits.");
        return;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Arm Target joint position within joint limits.");
    }
    
    bool gripper_within_bounds = (gripper_joint_goal.size() == gripper_move_group.getActiveJoints().size());
    if (!gripper_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "GRIPPER Target joint position outside of limits.");
        return;
    } 
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Gripper Target joint position within joint limits.");
    }   

    //Plan the move
    moveit::planning_interface::MoveGroupInterface::Plan rail_plan;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

    //Check if planning of the move is valid
    bool rail_plan_success = (rail_move_group.plan(rail_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (rail_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rail planner successfully planned rail motion!");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Rail planner failed!");
        return;
    }
    //Execute the Rail move and report success or failure
    bool rail_execution_success = (rail_move_group.execute(rail_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (rail_execution_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Planner Succeeded in moving the rails.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Planner Failed in moving the rails.");
    } 


    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (arm_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arm planner successfully planned arm motion!");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arm planner failed!");
        return;
    }
    //Execute the Arm move and report success or failure
    bool arm_execution_success = (arm_move_group.execute(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (arm_execution_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Planner Succeeded in moving the arm.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Planner Failed in moving the arm.");
    }       
    
    bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);     
    if (gripper_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper planner successfully planned gripper motion!");
    }   
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Gripper planner failed!");
        return;
    }
    //Execute the Gripper move and report success or failure
    bool gripper_execution_success = (gripper_move_group.execute(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (gripper_execution_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Planner Succeeded in moving the gripper.");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Planner Failed in moving the gripper.");
    } 
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface"); 
    move_robot(node);
    rclcpp::shutdown();
    return 0;
}