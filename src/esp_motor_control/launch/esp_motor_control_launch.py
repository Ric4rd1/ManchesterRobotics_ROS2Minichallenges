from launch import LaunchDescription
from launch_ros.actions import Node

# Packages to get the address of the yaml file
import os 
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():

    # Get the address of the yaml file 
    config = os.path.join(
        get_package_share_directory('esp_motor_control'),
        'config',
        'params.yaml')
    
    sp_node = Node(name="set_point",
                       package='esp_motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       namespace='RCG',
                       parameters=[config]
                       )
    
    micro_ros_agent_node = Node(name='micro_ros_agent',
                        package='micro_ros_agent',
                        executable='micro_ros_agent',
                        arguments=['serial', '--dev', '/dev/ttyUSB0'],
                        output='screen'
                        )
    
    rqt_reconfigure_node = Node(
                        package='rqt_reconfigure',
                        executable='rqt_reconfigure',
                        name='rqt_reconfigure',
                        output='screen'
                    )
    
    
    l_d = LaunchDescription([sp_node, micro_ros_agent_node,rqt_reconfigure_node])

    return l_d