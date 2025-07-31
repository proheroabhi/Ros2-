import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
#import RPi.GPIO as GPIO  # Replace with your GPIO library if different

class ServoMotorService(Node):
    def __init__(self):
        super().__init__('servo_motor_service')
        self.srv = self.create_service(Trigger, 'rotate_servo', self.handle_rotate_servo)
        self.get_logger().info('Servo Motor Service Node has been started.')

        # GPIO setup
        # self.servo_pin = 18  # Replace with your GPIO pin number
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.servo_pin, GPIO.OUT)
        # self.pwm = GPIO.PWM(self.servo_pin, 50)  # 50 Hz PWM frequency
        # self.pwm.start(0)  # Start with 0 duty cycle (stopped)

    def handle_rotate_servo(self, request, response):
        try:
            self.get_logger().info('Rotating servo motor at full speed...')
            # self.pwm.ChangeDutyCycle(10)  # Adjust duty cycle for full speed (e.g., 10 for clockwise)
            response.success = True
            response.message = 'Servo motor is rotating at full speed.'
        except Exception as e:
            self.get_logger().error(f'Failed to rotate servo motor: {e}')
            response.success = False
            response.message = f'Error: {e}'
        return response

    def destroy_node(self):
        # self.pwm.stop()
        # GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoMotorService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()