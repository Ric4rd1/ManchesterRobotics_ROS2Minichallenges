import numpy as np
import time as pytime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        # Publishers
        self.sig_publisher = self.create_publisher(Float32, 'signal', 10) #buffer time might need adjusting
        self.time_publisher = self.create_publisher(Float32, 'time', 10)
        
        
        # Timer
        timer_period = 0.05 # 100Hz samle freq
        self.timer = self.create_timer(timer_period, self.calculate_sin)

        self.start_time = pytime.time()

    def calculate_sin(self):
        t = pytime.time() - self.start_time
        freq = 1
        sine_wave = np.sin(2*np.pi*freq*t)

        # Publish signal
        signal_msg = Float32()
        signal_msg.data = sine_wave
        self.sig_publisher.publish(signal_msg)

        '''
        # Publish time
        time_msg = Float32()
        time_msg.data = t
        self.time_publisher.publish(time_msg)
'''
        # Log
        self.get_logger().info(f'Time: {t:.2f}s, Signal: {sine_wave:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = SignalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
