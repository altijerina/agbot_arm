from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arm_description"),
            "launch",
            "arm_gazebo.launch.py"
        )        
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arm_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={"is_sim" : "True"}.items()
    )
    
    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arm_moveit"),
            "launch",
            "arm_moveit.launch.py"
        ),
        launch_arguments={"is_sim" : "True"}.items()
    )   
    
    remote_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arm_remote"),
            "launch",
            "arm_remote_interface.launch.py"
        )
    )  
    
    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        remote_interface
    ])