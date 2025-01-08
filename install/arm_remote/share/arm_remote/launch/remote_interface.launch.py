import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value = "True"
    )
    
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value = "False"
    )
    
    is_sim = LaunchConfiguration("is_sim")
    use_python = LaunchConfiguration("use_python")

    moveit_config = (
        MoveItConfigsBuilder("agbot_arm", package_name="arm_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("arm_description"),
            "urdf",
            "arm.xacro"
            )
        )
        .robot_description_semantic(file_path="config/agbot_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()
    )
    
    task_server_node_py = Node(
        package = "arm_remote",
        executable = "task_server.py",
        condition=IfCondition(use_python),
        parameters = [
            moveit_config.to_dict(),
            {"use_sim_time" : is_sim}
        ]
    )
    
    task_server_node = Node(
        package = "arm_remote",
        executable = "task_server_node",
        condition=UnlessCondition(use_python),
        parameters = [{"use_sim_time" : is_sim}
        ]
    )
    
    alexa_interface_node = Node(
        package = "arm_remote",
        executable = "alexa_interface.py",
        parameters=[{"use_sim_time": is_sim}]
    )
    
    return LaunchDescription([
        use_python_arg,
        is_sim_arg,
        task_server_node_py,
        task_server_node,
        alexa_interface_node
    ])