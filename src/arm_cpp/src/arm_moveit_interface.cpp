#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class AgArmMover : public rclcpp::Node
{
public:
    AgArmMover() : Node("agbot_mover")
    {
        // Initialize Move Group Interfaces for different planning groups
        rail_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("rail_system");
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("gripper");

        // Move rail system
        moveRail();

        // Move arm to a specific pose
        moveArm();

        // Open the gripper
        controlGripper(true);  // true to open, false to close
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> rail_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

    void moveRail()
    {
        // Define a target for the rail system
        geometry_msgs::msg::Pose rail_target;
        rail_target.position.x = 1.0;  // Adjust as needed
        rail_target.position.y = 0.0;
        rail_target.position.z = 0.5;
        rail_group_->setPoseTarget(rail_target);

        // Plan and execute the movement
        moveit::planning_interface::MoveGroupInterface::Plan rail_plan;
        bool success = (rail_group_->plan(rail_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            rail_group_->move();
            RCLCPP_INFO(this->get_logger(), "Rail system moved successfully.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Rail system planning failed.");
        }
    }

    void moveArm()
    {
        // Define a target pose for the arm
        geometry_msgs::msg::Pose arm_target;
        arm_target.position.x = 0.4;  // Adjust as needed
        arm_target.position.y = 0.0;
        arm_target.position.z = 0.4;
        arm_target.orientation.w = 1.0;
        arm_group_->setPoseTarget(arm_target);

        // Plan and execute the arm movement
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        bool success = (arm_group_->plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            arm_group_->move();
            RCLCPP_INFO(this->get_logger(), "Arm moved successfully.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Arm planning failed.");
        }
    }

    void controlGripper(bool open)
    {
        // Control the gripper (assuming it can be controlled with joint positions)
        std::vector<double> gripper_joint_positions = open ? std::vector<double>{0.02} : std::vector<double>{0.0}; // Adjust values accordingly
        gripper_group_->setJointValueTarget(gripper_joint_positions);

        // Plan and execute the gripper movement
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool success = (gripper_group_->plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            gripper_group_->move();
            RCLCPP_INFO(this->get_logger(), open ? "Gripper opened." : "Gripper closed.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Gripper planning failed.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgBotMover>());
    rclcpp::shutdown();
    return 0;
}
