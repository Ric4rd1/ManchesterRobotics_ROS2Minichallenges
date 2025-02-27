from launch import LaunchDescription
from launch_ros.actions import Node

# Packages to get the address of the yaml file
import os 
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():

    # Get the address of the yaml file 
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'params.yaml')

    motor_node = Node(name="motor_sys",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       parameters=[config]
                    )
    
    sp_node = Node(name="sp_gen",
                       package='motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       parameters=[config]
                       )
    
    ctrl_node = Node(name = "ctrl",
                     package='motor_control',
                     executable='controller',
                     emulate_tty=True,
                     output='screen',
                     parameters=[config]

                     )
    
    l_d = LaunchDescription([motor_node, sp_node, ctrl_node])

    return l_d