#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sdformat/sdf.hh>
#include <vector>
#include <string>
#include <iostream>

class RobotMover : public rclcpp::Node
{
public:
    RobotMover() : Node("robot_mover")
    {
        // Initialize MoveIt MoveGroup Interfaces for each group
        rail_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("rail_system");
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("gripper");

        // Define SDF file path
        sdf_file_path_ = "arm_moveit/config/agbot_arm.sdf";

        // Move to target positions for each group and state
        moveToTargetPositions("rail_system", "rail_ready");
        moveToTargetPositions("rail_system", "rail_home");
        moveToTargetPositions("arm", "arm_home");
        moveToTargetPositions("arm", "arm_ready");
        moveToTargetPositions("arm", "arm_ready_low_left");
        moveToTargetPositions("arm", "arm_ready_low_right");
        moveToTargetPositions("arm", "arm_ready_high_left");
        moveToTargetPositions("arm", "arm_ready_high_right");
        moveToTargetPositions("gripper", "gripper_ready");
        moveToTargetPositions("gripper", "gripper_grab");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> rail_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    std::string sdf_file_path_;

    geometry_msgs::msg::Pose computeEndEffectorPose(const std::vector<double>& joint_positions, const std::string& group_name)
    {
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);
        move_group->setJointValueTarget(joint_positions);
        move_group->move();  // Execute the movement
        return move_group->getCurrentPose("gripper").pose;  // Get the pose of the end effector
    }

    std::vector<double> extractJointPositionsFromSDF(const std::string& group_name, const std::string& state_name)
    {
        sdf::SDF sdf;
        sdf::init(sdf);
        sdf::readFile(sdf_file_path_, sdf);

        std::vector<double> joint_positions;

        sdf::ElementPtr model = sdf.root->GetElement("model");
        sdf::ElementPtr joint_states = model->GetElement("joint_states");

        while (joint_states) {
            std::string name = joint_states->Get<std::string>("name");
            std::string state = joint_states->Get<std::string>("state");

            if (name == group_name && state == state_name) {
                joint_positions = joint_states->Get<std::vector<double>>("positions");
                break;
            }

            joint_states = joint_states->GetNextElement("joint_states");
        }

        return joint_positions;
    }

    void moveToTargetPositions(const std::string& group_name, const std::string& state_name)
    {
        std::vector<double> joint_positions = extractJointPositionsFromSDF(group_name, state_name);

        // If no positions were found, return early
        if (joint_positions.empty()) {
            RCLCPP_WARN(this->get_logger(), "No joint positions found for %s/%s", group_name.c_str(), state_name.c_str());
            return;
        }

        geometry_msgs::msg::Pose target_pose = computeEndEffectorPose(joint_positions, group_name);

        // Log the target position in x, y, z
        RCLCPP_INFO(this->get_logger(), "Target position for %s/%s: x: %f, y: %f, z: %f",
                     group_name.c_str(), state_name.c_str(), target_pose.position.x, target_pose.position.y, target_pose.position.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mover = std::make_shared<RobotMover>();
    rclcpp::spin(mover);
    rclcpp::shutdown();
    return 0;
}
