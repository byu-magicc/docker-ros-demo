import rclpy
from rclpy.node import Node
from rosflight_msgs.msg import Command
from rosplane_msgs.msg import ControllerCommands, State
from std_srvs.srv import Trigger

class CourseController(Node):
    
    # This controller drives the course to a particular heading by commanding a particular roll anlge, phi_c
    # NOTE: In order to run this file, you have to rename it to course_controller.py
    #       (this requires renaming the original course_controller.py to something else entirely).

    def __init__(self) -> None:
        super().__init__('YOUR_NAME_course_controller')

        # TODO: Create necessary publisher.

        # TODO: Create necessary subscribers.

        # TODO: Create Control gains.

        # TODO: Create a timer that the control will be calulated and published on.

        # TODO: Optional: Create service to toggle the control on and off, or to set the desired course.


    def activate_control_callback(self, request, response): # Callback for service.
        # This is an example service that would toggle control.
        # Feel free to modify it to set the course angle or something else!
        pass

    def state_callback(self, msg): # Callback for when state message is received
        # Save the state message for use in the control callback.
        pass

    def controller_command_callback(self, msg):
        # Save the controller command to fill the rest of the message later.
        pass

    def control(self): # Callback to run on each timer call.
        # Control to a desired arbitrary course (chi).
        # See https://docs.rosflight.org/git-main/developer-guide/rosplane/controller/controller-outline/#course-loop
        # under outer loop for more details.

        # OR just use the below controller
        
        course_angle = self.current_state.chi # Radians
        
        commanded_course_angle = self.commanded_course * 3.14159/180.0

        error = commanded_course_angle - course_angle

        self.integrator += (self.Ts / 2.0) * (error + self.prev_error);  

        commanded_roll_angle = self.kp * error + self.kd*self.current_state.r + self.ki*self.integrator
        
        self.prev_error = error

        pass


def main():
    rclpy.init()

    course_controller = CourseController()

    rclpy.spin(course_controller)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
