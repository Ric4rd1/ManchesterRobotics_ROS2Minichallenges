# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Parameters
        self.declare_parameter('timer_period', 0.001) # seconds
        self.declare_parameter('signal_type', 'square')
        self.signals = ['sine', 'square', 'step'] # valid signals
        # sine parameters
        self.declare_parameter('sine_amplitude', 2.0)
        self.declare_parameter('sine_omega', 1.0)
        # square parameters
        self.declare_parameter('square_amplitude', 2.0)
        self.declare_parameter('square_period', 5.0)
        self.declare_parameter('square_duty_cycle', 0.5)
        # step parameters
        self.declare_parameter('step_amplitude', 0.0)

        # Retrieve Parameters
        self.signal_type = self.get_parameter('signal_type').value
        # Sinewave signal params
        self.sine_amplitude = self.get_parameter('sine_amplitude').value
        self.sine_omega = self.get_parameter('sine_omega').value
        self.timer_period = self.get_parameter('timer_period').value
        # Square signal params
        self.square_amplitude = self.get_parameter('square_amplitude').value
        self.square_period = self.get_parameter('square_period').value
        self.square_duty_cycle = self.get_parameter('square_duty_cycle').value
        # Step signal params
        self.step_amplitude = self.get_parameter('step_amplitude').value

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)    ## CHECK FOR THE NAME OF THE TOPIC
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Initialization completed
        self.get_logger().info("SetPoint Node Started \U0001F680")


    def parameter_callback(self, params):
        for param in params:
            if param.name == "signal_type":
                if param.value in self.signals:
                    self.signal_type = param.value
                    self.get_logger().info(f"signal_type updated to {self.signal_type}")
                else:
                    self.get_logger().warn(f"Invalid Signal type! Must be a valid signal: {self.signals}")
                    return SetParametersResult(successful=False, reason="Invalid signal type")
                
            if param.name == "sine_amplitude":
                if param.value > 0:
                    self.sine_amplitude = param.value
                    self.get_logger().info(f"sine_amplitude updated to {self.sine_amplitude}")
                else:
                    self.get_logger().warn(f"Invalid sine amplitude! must be a positive number")
                    return SetParametersResult(successful=False, reason="negative number")
            
            if param.name == "sine_omega":
                if param.value > 0:
                    self.sine_omega = param.value
                    self.get_logger().info(f"sine_omega updated to {self.sine_omega}")
                else:
                    self.get_logger().warn(f"Invalid sine omega! must be a positive number")
                    return SetParametersResult(successful=False, reason="negative number")
            
            if param.name == "square_amplitude":
                if param.value > 0:
                    self.square_amplitude = param.value 
                    self.get_logger().info(f"square_amplitude updated to {self.square_amplitude}")
                else:
                    self.get_logger().warn(f"Invalid square amplitude! must be a positive number")
                    return SetParametersResult(successful=False, reason="negative number")
                
            if param.name == "square_period":
                if param.value > 0:
                    self.square_period = param.value
                    self.get_logger().info(f"square_period updated to {self.square_period}")
                else:
                    self.get_logger().warn(f"Invalid square period! must be a positive number")
                    return SetParametersResult(successful=False, reason="negative number")
                
            if param.name == "square_duty_cycle":
                if param.value > 0 and param.value <= 1:
                    self.square_duty_cycle = param.value
                    self.get_logger().info(f"square_duty_cycle updated to {self.square_duty_cycle}")
                else:
                    self.get_logger().warn(f"Invalid duty cycle! must be a number between 0 and 1")
                    return SetParametersResult(successful=False, reason="not a number between 0 and 1")
                
            if param.name == "step_amplitude":
                self.step_amplitude = param.value
                self.get_logger().info(f"step_amplitude updated to {self.step_amplitude}")
                
            

        return SetParametersResult(successful=True)

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):

        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9

        # Choose signal type
        if (self.signal_type == 'sine'):
            # Generate sine wave signal
            self.signal_msg.data = self.sine_amplitude * np.sin(self.sine_omega * elapsed_time)
            # Publish the signal
            self.signal_publisher.publish(self.signal_msg)
        elif (self.signal_type == 'square'):
            # Compute frequency from period
            f = 1 / self.square_period  # Hz
            # Generate square signal (using floor)
            self.signal_msg.data = self.square_amplitude * (2 * (np.floor(f * elapsed_time) - np.floor(f * elapsed_time + self.square_duty_cycle)) + 1)

            # Alternative for generating square singal (using sin, duty cycle 0.5 contant)
            #self.signal_msg.data = self.square_amplitude * np.sign(np.sin(self.sine_omega * elapsed_time))

            # Publish the signal
            self.signal_publisher.publish(self.signal_msg)
        elif (self.signal_type == 'step'):
            self.signal_msg.data = self.step_amplitude
            self.signal_publisher.publish(self.signal_msg)

    

#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
