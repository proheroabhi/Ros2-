import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SoilDataPublisher(Node):
    def __init__(self):
        super().__init__('soil_data_publisher')
        self.soil_moisture_publisher = self.create_publisher(Float32, 'soil_data', 10) # publish soil moisture data
        self.timer = self.create_timer(5.0, self.publish_sensor_data)  # Publish every 5 seconds
        self.get_logger().info('Soil Data Publisher Node has been started.')
        

    def publish_sensor_data(self):
        # Simulated sensor data
        soil_moisture = 55.5  # Example value for soil moisture
        
        # Publish soil moisture
        soil_moisture_msg = Float32()
        soil_moisture_msg.data = soil_moisture
        self.soil_moisture_publisher.publish(soil_moisture_msg)
        self.get_logger().info(f'Published Soil Moisture: {soil_moisture}')


def main(args=None):
    rclpy.init(args=args)
    node = SoilDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()