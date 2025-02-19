from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    signal_generator = Node(package='signal_processing',
                       executable='signal_generator',
                       output ='screen')
    
    process = Node(package='signal_processing',
                       executable='process',
                       output ='screen')
    
    node_plot = Node(name='rqt_plot',
                package='rqt_plot',
                executable='rqt_plot',
                arguments=['/signal/data',
                '/proc_signal/data'])
    # PlotJuggler
    '''
    node_plot = Node(
        package='plotjuggler',
        executable='plotjuggler',
        output='screen'
        )
    '''
    l_d = LaunchDescription([signal_generator, process, node_plot])
    return l_d