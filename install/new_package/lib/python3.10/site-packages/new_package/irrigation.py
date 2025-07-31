import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger
from action_tutorials_interfaces.action import Fibonacci  # Replace with your custom action if needed

class IrrigationActionServer(Node):

    def __init__(self):
        super().__init__('irrigation_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Replace with your custom action if needed
            'irrigation',
            self.execute_callback
        )
        self.servo_client = self.create_client(Trigger, 'rotate_servo')
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for servo motor service to become available...')
        self.get_logger().info('Irrigation Action Server has been started.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing irrigation action...')
        feedback_msg = Fibonacci.Feedback()  # Replace with your custom feedback message
        feedback_msg.partial_sequence = []  # Example feedback field

        # Step 1: Call servo motor service
        feedback_msg.partial_sequence.append('Calling servo motor service...')
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Calling servo motor service...')
        if not self.call_servo_motor_service():
            goal_handle.abort()
            self.get_logger().error('Failed to call servo motor service.')
            return Fibonacci.Result()  # Replace with your custom result message

        # Step 2: Open solenoid valve
        feedback_msg.partial_sequence.append('Opening solenoid valve...')
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Opening solenoid valve...')
        self.open_solenoid_valve()

        # Step 3: Measure water flow
        feedback_msg.partial_sequence.append('Measuring water flow...')
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Measuring water flow...')
        water_flow = self.measure_water_flow()

        # Final feedback
        feedback_msg.partial_sequence.append(f'Water flow: {water_flow} L/min')
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f'Water flow: {water_flow} L/min')

        # Complete the action
        goal_handle.succeed()
        result = Fibonacci.Result()  # Replace with your custom result message
        result.sequence = [water_flow]  # Example result field
        return result

    def call_servo_motor_service(self):
        request = Trigger.Request()
        future = self.servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Servo Motor Service Response: {response.message}')
                return True
            else:
                self.get_logger().error(f'Servo Motor Service Failed: {response.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'Failed to call Servo Motor Service: {e}')
            return False

    def open_solenoid_valve(self):
        # Simulate opening the solenoid valve
        self.get_logger().info('Solenoid valve opened.')

    def measure_water_flow(self):
        # Simulate measuring water flow
        water_flow = 10.5  # Example value in L/min
        return water_flow


def main(args=None):
    rclpy.init(args=args)
    node = IrrigationActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
# This code is a ROS2 action server that handles irrigation tasks. It interacts with a servo motor service and simulates solenoid valve operations and water flow measurement.