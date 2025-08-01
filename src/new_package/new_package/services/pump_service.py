import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO

class PumpService(Node):
    def __init__(self):
        super().__init__('pump_service')
        self.srv = self.create_service(SetBool, 'open_pump', self.handle_pump)
        self.get_logger().info('Pump Service Node has been started.')

        self.pump_pin = 23  # Change to your GPIO pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pump_pin, GPIO.OUT)
        GPIO.output(self.pump_pin, GPIO.LOW)  # pump initially closed

    def handle_pump(self, request, response):
        if request.data:
            GPIO.output(self.pump_pin, GPIO.HIGH)  # Open pump
            response.success = True
            response.message = 'Water pump opened.'
            self.get_logger().info('Water pump opened.')
        else:
            GPIO.output(self.pump_pin, GPIO.LOW)  # Close pump
            response.success = True
            response.message = 'Water pump closed.'
            self.get_logger().info('Water pump closed.')
        return response

    def destroy_node(self):
        GPIO.output(self.pump_pin, GPIO.LOW)
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PumpService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()