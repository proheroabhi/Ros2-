import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO

class ValveService(Node):
    def __init__(self):
        super().__init__('valve_service')
        self.srv = self.create_service(SetBool, 'open_valve', self.handle_open_valve)
        self.get_logger().info('Valve Service Node has been started.')

        self.valve_pin = 23  # Change to your GPIO pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.valve_pin, GPIO.OUT)
        GPIO.output(self.valve_pin, GPIO.LOW)  # Valve initially closed

    def handle_open_valve(self, request, response):
        if request.data:
            GPIO.output(self.valve_pin, GPIO.HIGH)  # Open valve
            response.success = True
            response.message = 'Solenoid valve opened.'
            self.get_logger().info('Solenoid valve opened.')
        else:
            GPIO.output(self.valve_pin, GPIO.LOW)  # Close valve
            response.success = True
            response.message = 'Solenoid valve closed.'
            self.get_logger().info('Solenoid valve closed.')
        return response

    def destroy_node(self):
        GPIO.output(self.valve_pin, GPIO.LOW)
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ValveService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()