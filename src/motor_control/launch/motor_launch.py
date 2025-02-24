from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_node = Node(name="motor_sys",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       parameters=[{
                        'sample_time': 0.01,
                        'sys_gain_K': 2.16,
                        'sys_tau_T': 0.05,
                        'initial_conditions': 0.0,
                            }
                        ]
                    )
    
    sp_node = Node(name="sp_gen",
                       package='motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       )
    
    ctrl_node = Node(name = "ctrl",
                     package='motor_control',
                     executable='controller',
                     emulate_tty=True,
                     output='screen',
                     parameters=[{
                         'kp': 3.1,
                     }]

                     )
    
    l_d = LaunchDescription([motor_node, sp_node, ctrl_node])

    return l_d