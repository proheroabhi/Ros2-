import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time


class WaterFlowPublisher(Node):
    def __init__(self):
        super().__init__('waterFlow_publisher')
        self.waterFlow_publisher = self.create_publisher(Float32, 'waterFlow', 10) # publish temperature and humidity data
        self.timer = self.create_timer(5.0, self.publish_sensor_data)  # Publish every 5 seconds
        self.get_logger().info('Water Flow Data Publisher Node has been started.')
        
        # GPIO pin where the water flow sensor is connected
        self.FLOW_SENSOR_PIN = 18  

        # Flow rate variables
        self.pulse_count = 0
        self.calibration_factor = 7.5  # Adjust based on your sensor specs

        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.FLOW_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.FLOW_SENSOR_PIN, GPIO.RISING, callback=self.pulse_counter)

        self.last_time = time.time()

    def pulse_counter(self, channel):
        """Interrupt function to count pulses"""
        self.pulse_count += 1

    def publish_sensor_data(self):
        # Simulated sensor data
        flow_rate = 55.5  # Example value for soil moisture

        # Calculate elapsed time
        current_time = time.time()
        elapsed = current_time - self.last_time
        self.last_time = current_time

        # Calculate flow rate (Liters per minute)
        flow_rate = (self.pulse_count / self.calibration_factor) * (60 / elapsed)
        self.pulse_count = 0

        # Publish temperature and humidity together
        flow_msg = Float32()
        flow_msg.data = flow_rate
        self.waterFlow_publisher.publish(flow_msg)
        self.get_logger().info(f'Published Water Flow Rate: {flow_msg.data:.2f} L/min')

def main(args=None):
    rclpy.init(args=args)
    node = WaterFlowPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()