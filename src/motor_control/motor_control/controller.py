import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import InitiateProcess

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Client for InitiateProcess
        self.system_running = False
        self.cli = self.create_client(InitiateProcess, 'EnableProcess') 

        while not self.cli.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info('Service not available, waiting again...') 
        self.send_request(True) 

        # Parameters
        self.declare_parameter('kp', 0.1)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.01)
        self.declare_parameter('sample_time', 0.05)

        # Retrieve control parameters
        self.param_kp = self.get_parameter('kp').value
        self.param_ki = self.get_parameter('ki').value
        self.param_kd = self.get_parameter('kd').value
        self.param_sample_time = self.get_parameter('sample_time').value

        # Message output
        self.control_output_msg = Float32()

        # Variables
        self.setpoint = 0.0
        self.curr_speed = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.y_speed = 0.0

        # Publishers, Subscribers and Timers
        self.control_input_sub = self.create_subscription(Float32, 'RCGmotor_speed_y', self.input_callback, 10)
        self.control_setpoint_sub = self.create_subscription(Float32, 'RCGset_point', self.setpoint_callback, 10)
        self.control_signal_pub = self.create_publisher(Float32, 'RCGmotor_input_u', 10)
        self.timer = self.create_timer(self.param_sample_time, self.timer_cb)

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callbacks)

        # Node started
        self.get_logger().info('Control Node Started ⚡')

    def send_request(self, enable: bool): 
        request = InitiateProcess.Request() 
        request.enable = enable 
        future = self.cli.call_async(request) 
        future.add_done_callback(self.response_callback) 

    def response_callback(self, future): 
        """Process the service response.""" 
        try: 
            response = future.result() 
            if response.success: 
                self.system_running = True 
                self.get_logger().info(f'Success: {response.message}') 
            else: 
                self.system_running = False 
                self.get_logger().warn(f'Failure: {response.message}') 
        except Exception as e: 
            self.system_running = False 
            self.get_logger().error(f'Service call failed: {e}') 

    def timer_cb(self):
        # Compute error
        self.error = self.setpoint - self.curr_speed

        # Compute integral term
        self.integral_error += self.error * self.param_sample_time

        # Compute derivative term
        derivative_error = (self.error - self.prev_error) / self.param_sample_time

        # Compute control output
        self.y_speed = (self.param_kp * self.error) + \
                       (self.param_ki * self.integral_error) + \
                       (self.param_kd * derivative_error)

        # Store previous error for next iteration
        self.prev_error = self.error

        # Publish control signal
        self.control_output_msg.data = self.y_speed
        self.control_signal_pub.publish(self.control_output_msg)

    def parameters_callbacks(self, params):
        for param in params:
            if param.name == 'kp' and param.value >= 0:
                self.param_kp = param.value
                self.get_logger().info(f"Updated kp to {self.param_kp}")
            elif param.name == 'ki' and param.value >= 0:
                self.param_ki = param.value
                self.get_logger().info(f"Updated ki to {self.param_ki}")
            elif param.name == 'kd' and param.value >= 0:
                self.param_kd = param.value
                self.get_logger().info(f"Updated kd to {self.param_kd}")
            elif param.value < 0:
                self.get_logger().warn(f"Invalid {param.name}, it cannot be negative.")
                return SetParametersResult(successful=False, reason=f"{param.name} cannot be negative")

        return SetParametersResult(successful=True)

    # motor_speed_y input callback
    def input_callback(self, msg):
        self.curr_speed = msg.data

    # Setpoint callback 
    def setpoint_callback(self, msg):
        self.setpoint = msg.data

# Main
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

# Execute Node
if __name__ == '__main__':
    main()
