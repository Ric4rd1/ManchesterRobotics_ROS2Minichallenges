import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Parameters
        self.declare_parameter('kp', 0.1)

        # Retrieve controll parameters
        self.param_kp = self.get_parameter('kp').value

        # Message output
        self.control_output_msg = Float32()

        # Variables
        self.setpoint = 0.0
        self.curr_speed = 0.0
        self.error = 0.0
        self.timer_period = 0.01 # seconds
        self.y_speed = 0.0

        # Publishers, Subscribers and Timers
        self.control_input_sub = self.create_subscription(Float32,'RCGmotor_speed_y',self.input_callback, 10)
        self.control_setpoint_sub = self.create_subscription(Float32,'RCGset_point', self.setpoint_callback, 10)
        self.control_signal_pub = self.create_publisher(Float32,'RCGmotor_input_u', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callbacks)

        # Node started
        self.get_logger().info('Control Node Started \u26A1')
    
    def timer_cb(self):
        # Error 
        self.error = self.setpoint - self.curr_speed
        # Proportional control
        self.y_speed = self.param_kp*self.error

        self.control_output_msg.data = self.y_speed
        self.control_signal_pub.publish(self.control_output_msg)


    def parameters_callbacks(self, params):
        for param in params:
            if param.name == 'kp':
                if (param.value >= 0):
                    self.param_kp = param.value
                    self.get_logger().info(f"Updated kp to {self.param_kp}")
                else:
                    self.get_logger().warn("Invalid kp, it cannot be a negative number.")
                    return SetParametersResult(successful=False, reason = "kp cannot be negative")
                
        return SetParametersResult(successful=True)

    # motor_speed_y input callback
    def input_callback(self, msg):
        self.curr_speed = msg.data

    # Setpoint callback 
    def setpoint_callback(self, msg):
        self.setpoint = msg.data
#Main
def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        controller.destroy_node()

#Execute Node
if __name__ == '__main__':
    main()
