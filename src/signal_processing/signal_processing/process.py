import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import numpy as np

class Process(Node):
    def __init__(self):
        super().__init__('process')
        # Publishser
        self.proc_signal = self.create_publisher(Float32, 'proc_signal', 10)
        # Subscriptions
        self.sig_subscription = self.create_subscription(
            Float32, 'signal', self.sig_callback, 10)
        self.time_subscription = self.create_subscription(
            Float32, 'time', self.time_callback, 10)

        self.last_sig = None
        self.last_time = None
        
    def time_callback(self, msg):
        self.last_time = msg.data

    def sig_callback(self, msg):
        self.last_sig = msg.data

        if self.last_time is not None:
            processed_signal = self.process_signal(self.last_time, self.last_sig)

            # Publish
            processed_signal_msg = Float32()
            processed_signal_msg.data = processed_signal
            self.proc_signal.publish(processed_signal_msg)

            self.get_logger().info(f'Time: {self.last_time:.2f}s, Signal: {processed_signal:.3f}')

    def process_signal(self, time, signal):
        # Cut amplitude by half
        proc_sig = signal / 2.0
        # Offset so signal remains positive for al time
        proc_sig += 0.5
        # Add a phase shift to the received signal
        # t1 = 1
        # proc_sig = proc_sig*np.cos(t1) + np.cos(time)*np.sin(t1)
        return proc_sig

def main(args=None):
    rclpy.init(args=args)
    node = Process()

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
