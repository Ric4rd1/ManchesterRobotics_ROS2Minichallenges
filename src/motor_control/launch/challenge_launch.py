# IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os
from ament_index_python.packages import get_package_share_directory

# IMPORTS REQUIRED FOR CALLING OTHER LAUNCH FILES (NESTING)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# IMPORTS REQUIRED TO PUSH A NAMESPACE (APPEND) A NAMESPACE TO A NESTED LAUNCH FILE
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParametersFromFile  # Import for YAML parameters

def generate_launch_description():
    package = "motor_control"
    launch_file = 'motor_launch.py'
    param_file = 'params.yaml' 

    group1_ns = 'group1'
    group2_ns = 'group2'
    group3_ns = 'group3'

    # Get directories
    package_directory = get_package_share_directory(package)
    launch_file_path = os.path.join(package_directory, "launch", launch_file)
    param_file_path = os.path.join(package_directory, "config", param_file)

    launch_source1 = PythonLaunchDescriptionSource(launch_file_path)
    launch_source2 = PythonLaunchDescriptionSource(launch_file_path)
    launch_source3 = PythonLaunchDescriptionSource(launch_file_path)

    talker_listener_launch_1 = IncludeLaunchDescription(launch_source1)
    talker_listener_launch_2 = IncludeLaunchDescription(launch_source2)
    talker_listener_launch_3 = IncludeLaunchDescription(launch_source3)

    # Setting parameters from YAML file
    param_action1 = SetParametersFromFile(param_file_path)
    param_action2 = SetParametersFromFile(param_file_path)
    param_action3 = SetParametersFromFile(param_file_path)

    motor_control_g1 = GroupAction(
        actions=[PushRosNamespace(group1_ns), param_action1, talker_listener_launch_1]
    )

    motor_control_g2 = GroupAction(
        actions=[PushRosNamespace(group2_ns), param_action2, talker_listener_launch_2]
    )

    motor_control_g3 = GroupAction(
        actions=[PushRosNamespace(group3_ns), param_action3, talker_listener_launch_3]
    )

    # Launch description
    l_d = LaunchDescription([motor_control_g1, motor_control_g2, motor_control_g3])

    return l_d
