import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SensorDataPublisher(Node):
    def __init__(self):
        super().__init__('sensor_data_publisher')
        self.soil_moisture_publisher = self.create_publisher(Float32, 'soil_moisture', 10)
        self.temperature_publisher = self.create_publisher(Float32, 'temperature', 10)
        self.humidity_publisher = self.create_publisher(Float32, 'humidity', 10)
        self.timer = self.create_timer(5.0, self.publish_sensor_data)  # Publish every second
        self.get_logger().info('Sensor Data Publisher Node has been started.')

    def publish_sensor_data(self):
        # Simulated sensor data
        soil_moisture = 45.6  # Example value for soil moisture
        temperature = 22.3    # Example value for temperature
        humidity = 55.2       # Example value for humidity

        # Publish soil moisture
        soil_moisture_msg = Float32()
        soil_moisture_msg.data = soil_moisture
        self.soil_moisture_publisher.publish(soil_moisture_msg)
        self.get_logger().info(f'Published Soil Moisture: {soil_moisture}')

        # Publish temperature
        temperature_msg = Float32()
        temperature_msg.data = temperature
        self.temperature_publisher.publish(temperature_msg)
        self.get_logger().info(f'Published Temperature: {temperature}')

        # Publish humidity
        humidity_msg = Float32()
        humidity_msg.data = humidity
        self.humidity_publisher.publish(humidity_msg)
        self.get_logger().info(f'Published Humidity: {humidity}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()