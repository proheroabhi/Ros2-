import rclpy
from rclpy.node import Node
from new_package.msg import DHT11  # Import your DHT message
import Adafruit_DHT

class DHT_Publisher(Node):
    def __init__(self):
        super().__init__('DHT_data_publisher')
        self.tempHumi_publisher = self.create_publisher(DHT11.msg, 'DHT', 10) # publish temperature and humidity data
        self.timer = self.create_timer(5.0, self.publish_sensor_data)  # Publish every 5 seconds
        self.get_logger().info('DHT Data Publisher Node has been started.')
        
        # Setup sensors
        self.DHT_SENSOR = Adafruit_DHT.DHT11
        self.DHT_PIN = 4  # GPIO Pin for DHT11

    def publish_sensor_data(self):
        # Simulated sensor data
        temperature = 0.0    # Example value for temperature
        humidity = 0.0       # Example value for humidity
        humidity, temperature = Adafruit_DHT.read_retry(self.DHT_SENSOR, self.DHT_PIN)

        # Publish temperature and humidity together
        temp_humi_msg = TemperatureHumidity()
        temp_humi_msg.temperature = float(temperature) if temperature is not None else 0.0
        temp_humi_msg.humidity = float(humidity) if humidity is not None else 0.0
        self.tempHumi_publisher.publish(temp_humi_msg)
        self.get_logger().info(f'Published Temperature: {temp_humi_msg.temperature}, Humidity: {temp_humi_msg.humidity}')

def main(args=None):
    rclpy.init(args=args)
    node = DHT_Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()