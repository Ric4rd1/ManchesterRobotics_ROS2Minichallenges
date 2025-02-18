from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    signal_generator = Node(package='signal_processing',
                       executable='signal_generator',
                       output ='screen')
    
    process = Node(package='signal_processing',
                       executable='process',
                       output ='screen')
    
    l_d = LaunchDescription([signal_generator, process])
    return l_d