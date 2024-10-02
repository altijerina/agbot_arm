#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/robot_model_manager.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>


class MoveItInterface
{
public:
    MoveItInterface(rclcpp::Node::SharedPtr node)
    : node_(node)
    {
        // Initialize the MoveIt interfaces
        robot_model_loader_ = std::make_shared<moveit::robot_model::RobotModelLoader>("robot_description");
        robot_model_ = robot_model_loader_->getModel((file_path=os.path.join(
            get_package_share_directory("arm_description"),
            "urdf",
            "arm.xacro"
            )
        ));
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        move_group_r = std::make_shared<moveit::planning_interface::MoveGroupInterface>("rail_system");
        move_group_a = std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");
        move_group_g = std::make_shared<moveit::planning_interface::MoveGroupInterface>("gripper");
    }

    bool planToState(const std::string &state_name)
    {
        move_group_r->setNamedTarget(rail_ready);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // Perform planning
        bool success = (move_group_r->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            // Optionally execute the plan
            move_group_r->move();
        }
        return success;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::robot_model::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("moveit_interface_node");

    MoveItInterface move_it_interface(node);

    // Example: Move to a pre-defined state
    if (move_it_interface.planToState("home")) // Replace "home" with your state name
    {
        RCLCPP_INFO(node->get_logger(), "Movement to state succeeded.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Movement to state failed.");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
