import rclpy
from rclpy.node import Node
from rosflight_msgs.msg import Command
from rosplane_msgs.msg import ControllerCommands, State

from std_srvs.srv import Trigger

class CourseController(Node):

    # This contronller drives the course to a particular heading by commanding a particular roll anlge, phi_c

    def __init__(self) -> None:
        super().__init__('INSERT_YOUR_NAME_course_controller')

        # Service creation
        self.course_control_toggle_srv = self.create_service(Trigger, '/your_name/course_controller_toggle', self.activate_control_callback)
        self.is_active = False

        self.commanded_course = 22.0 # Degrees

        # Subscription to the current state
        self.state_sub = self.create_subscription(State, 'estimated_state', self.state_callback, 10)
        self.current_state = State()
        self.controller_command = ControllerCommands()

        # Subscription to the controller_command.
        self.controller_command_sub = self.create_subscription(ControllerCommands, 'controller_command', self.controller_command_callback, 10)

        self.control_pub = self.create_publisher(ControllerCommands, '/YOUR_NAME/controller_command', 10)

        self.kp = 1.0
        self.kd = 0.0
        self.ki = 0.001
        self.integrator = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0

        self.Ts = 0.01
        self.control_timer = self.create_timer(self.Ts, self.control)

    def activate_control_callback(self, request, response):

        # Since this is a Trigger message, the request is not used.
        
        self.get_logger().info('Toggling course control!')

        self.is_active = not self.is_active

        response.success = True

        if self.is_active:
            response.message = "The course control is active."
        else:
            response.message = "The course control is inactive."

        return response

    def state_callback(self, msg):
        self.current_state = msg
    
    def controller_command_callback(self, msg):
        self.controller_command = msg

    def control(self):

        course_angle = self.current_state.chi # Radians
        
        commanded_course_angle = self.commanded_course * 3.14159/180.0

        error = commanded_course_angle - course_angle

        self.integrator += (self.Ts / 2.0) * (error + self.prev_error);  

        commanded_roll_angle = self.kp * error + self.kd*self.current_state.r + self.ki*self.integrator
        
        self.prev_error = error

        msg = ControllerCommands()

        msg.phi_c = commanded_roll_angle
        
        # Pad with the rest of the values.
        msg.va_c = self.controller_command.va_c
        msg.h_c = self.controller_command.h_c
        msg.chi_c = self.controller_command.chi_c # Unused since we override phi_c
        msg.phi_ff = self.controller_command.phi_ff

        self.control_pub.publish(msg)

def main():
    rclpy.init()

    course_controller = CourseController()

    rclpy.spin(course_controller)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
