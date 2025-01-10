#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <arm_msgs/action/arm_task.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <thread>
using namespace std::chrono_literals;

using namespace std::placeholders;
using AgbotRobotTask = arm_msgs::action::ArmTask;
using agbotGoalHandle = rclcpp_action::ServerGoalHandle<AgbotRobotTask>;

namespace arm_remote
{
class AgbotTaskServer : public rclcpp::Node
{
public:
  explicit AgbotTaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("agbot_task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    agbot_action_server = rclcpp_action::create_server<AgbotRobotTask>(
        this, "agbot_task_server", std::bind(&AgbotTaskServer::agbotGoalCallback, this, _1, _2),
        std::bind(&AgbotTaskServer::agbotCancelCallback, this, _1),
        std::bind(&AgbotTaskServer::agbotAcceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<AgbotRobotTask>::SharedPtr agbot_action_server;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> agbot_rails_move_group;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> agbot_arm_move_group;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> agbot_gripper_move_group;
  std::vector<double> agbot_rail_joint_goal;
  std::vector<double> agbot_arm_joint_goal;
  std::vector<double> agbot_gripper_joint_goal;

  rclcpp_action::GoalResponse agbotGoalCallback(
      const rclcpp_action::GoalUUID& arm_uuid,
      std::shared_ptr<const AgbotRobotTask::Goal> agbot_goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with task_number: %d", agbot_goal->arm_task_number);
    (void)arm_uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse agbotCancelCallback(
    const std::shared_ptr<agbotGoalHandle> agbot_goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if (agbot_rails_move_group){
      agbot_rails_move_group->stop();
    }
    if (agbot_arm_move_group){
      agbot_arm_move_group->stop();
    }
    if (agbot_gripper_move_group){
      agbot_gripper_move_group->stop();
    }
    (void)agbot_goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void agbotAcceptedCallback(
      const std::shared_ptr<agbotGoalHandle> agbot_goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&AgbotTaskServer::execute, this, _1), agbot_goal_handle}.detach();
  }

  void execute(const std::shared_ptr<agbotGoalHandle> agbot_goal_handle)
  {
    int task_num_ = agbot_goal_handle->get_goal()->arm_task_number;

    RCLCPP_INFO(get_logger(), "Executing goal: %d", task_num_);
    if(!agbot_rails_move_group){
      agbot_rails_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "rails");
    }
    if(!agbot_arm_move_group){
      agbot_arm_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    }
    if(!agbot_gripper_move_group){
      agbot_gripper_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
    }

    if (agbot_goal_handle->get_goal()->arm_task_number == 0) //Home position
    {
      agbot_rail_joint_goal = {0.500, 1.130};
      agbot_arm_joint_goal = {0.00, 0.00, 0.00, 0.00};
      agbot_gripper_joint_goal = {-0.00};
    }
    else if(agbot_goal_handle->get_goal()->arm_task_number == 1) //Arm Ready
    {
      agbot_rail_joint_goal = {0.500, 1.130};
      agbot_arm_joint_goal = {0.0, 1.502, -1.423, 0.0};
      agbot_gripper_joint_goal = {-0.7} ;      
    }
    else if(agbot_goal_handle->get_goal()->arm_task_number == 2) //Arm Ready Low Left
    {
      agbot_rail_joint_goal = {0.500, 1.130};
      agbot_arm_joint_goal = {-1.349, 2.696, -1.909, -0.772};
      agbot_gripper_joint_goal = {-0.25} ;      
    }
    else if(agbot_goal_handle->get_goal()->arm_task_number == 3) //Arm Ready Low Right
    {
      agbot_rail_joint_goal = {0.500, 1.130};
      agbot_arm_joint_goal = {1.128, 2.815, -2.10, -0.125};
      agbot_gripper_joint_goal = {-0.7} ;      
    }
    else if(agbot_goal_handle->get_goal()->arm_task_number == 4) //Arm Ready High Right
    {
      agbot_rail_joint_goal = {0.500, 1.130};
      agbot_arm_joint_goal = {1.503, 1.967, -2.378, 0.408};
      agbot_gripper_joint_goal = {-1.50} ;      
    }
    else if(agbot_goal_handle->get_goal()->arm_task_number == 5) //Arm Ready High Left
    {
      agbot_rail_joint_goal = {0.500, 1.130};
      agbot_arm_joint_goal = {-1.128, 1.745, -2.078, 0.524};
      agbot_gripper_joint_goal = {-0.1} ;      
    }
    else if(agbot_goal_handle->get_goal()->arm_task_number == 6) //Home position
    {
      agbot_rail_joint_goal = {0.000, 0.000};
      agbot_arm_joint_goal = {0.00, 0.00, 0.00, 0.00};
      agbot_gripper_joint_goal = {-0.01};
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid task number");    
      return;
    }

    agbot_rails_move_group->setStartState(*agbot_rails_move_group->getCurrentState());
    agbot_arm_move_group->setStartState(*agbot_arm_move_group->getCurrentState());
    agbot_gripper_move_group->setStartState(*agbot_gripper_move_group->getCurrentState());

    bool rails_within_bounds = agbot_rails_move_group->setJointValueTarget(agbot_rail_joint_goal);
    bool arm_within_bounds = agbot_arm_move_group->setJointValueTarget(agbot_arm_joint_goal);
    bool gripper_within_bounds = agbot_gripper_move_group->setJointValueTarget(agbot_gripper_joint_goal);

    if (!rails_within_bounds)
    {
      RCLCPP_ERROR(get_logger(), "RAIL Target joint position outside of limits.");
      return;
    }
    
    if (!arm_within_bounds)
    {
      RCLCPP_ERROR(get_logger(), "ARM Target joint position outside of limits.");
      return;
    }

    if (!gripper_within_bounds)
    {
      RCLCPP_ERROR(get_logger(), "GRIPPER Target joint position outside of limits.");
      return;
    }     

    moveit::planning_interface::MoveGroupInterface::Plan agbot_rail_plan;
    moveit::planning_interface::MoveGroupInterface::Plan agbot_arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan agbot_gripper_plan;

    bool agbot_rail_plan_success = (agbot_rails_move_group->plan(agbot_rail_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool agbot_arm_plan_success = (agbot_arm_move_group->plan(agbot_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool agbot_gripper_plan_success = (agbot_gripper_move_group->plan(agbot_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (task_num_ == 6)
    {
        agbot_arm_move_group->move();
        std::this_thread::sleep_for(1s);
        agbot_gripper_move_group->move();
        agbot_rails_move_group->move();
    }
    else
    {
      if (agbot_rail_plan_success)
      {
          RCLCPP_INFO(get_logger(), "Rail Planner Succeeded in moving the rails.");
          agbot_rails_move_group->move();       
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Arm planner failed!");
        return;
      } 
      
      if (agbot_arm_plan_success)
      {
          RCLCPP_INFO(get_logger(), "Arm Planner Succeeded in moving the arm.");
          agbot_arm_move_group->move();       
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Arm planner failed!");
        return;
      }     

      if (agbot_gripper_plan_success)
      {
          RCLCPP_INFO(get_logger(), "Gripper Planner Succeeded in moving the gripper.");        
          agbot_gripper_move_group->move();       
      }    
      else
      {
        RCLCPP_ERROR(get_logger(), "Gripper planner failed!");
        return;
      }
    }    

    auto result = std::make_shared<AgbotRobotTask::Result>();
    result->arm_success = true;
    agbot_goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal executing Task Number: %d succeeded.", task_num_);

  }
};
}  // namespace agbot_arm_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arm_remote::AgbotTaskServer)