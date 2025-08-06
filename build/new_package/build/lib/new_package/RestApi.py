import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import requests
import json

class SensorDataSubscriber(Node):
    def __init__(self):
        super().__init__('restApi')
        self.soil_moisture = None
        self.temperature = None
        self.humidity = None
        self.water_flow = None

        self.soil_moisture_subscription = self.create_subscription(
            Float32,
            'soil_moisture',
            self.soil_moisture_callback,
            10
        )
        self.temperature_subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )
        self.humidity_subscription = self.create_subscription(
            Float32,
            'humidity',
            self.humidity_callback,
            10
        )
        self.water_flow_subscription = self.create_subscription(
            Float32,
            'waterFlow',
            self.water_flow_callback,
            10
        )
        self.get_logger().info('Sensor Data Subscriber Node has been started.')

    def soil_moisture_callback(self, msg):
        self.soil_moisture = msg.data
        self.send_data()

    def temperature_callback(self, msg):
        self.temperature = msg.data
        self.send_data()

    def humidity_callback(self, msg):
        self.humidity = msg.data
        self.send_data()

    def water_flow_callback(self, msg):
        self.water_flow = msg.data
        self.send_data()

    def send_data(self):
        # Only send if all values are available
        if None in (self.soil_moisture, self.temperature, self.humidity, self.water_flow):
            return

        url = 'http://192.168.137.1/FarmCS/handlers/receive_sensor_data.php'
        data = {
            "soil_moisture": self.soil_moisture,
            "temperature": self.temperature,
            "humidity": self.humidity,
            "water_flow": self.water_flow,
            "light_intensity": 1200,  # Example static value
            "device_id": "FARM_SENSOR_001"
        }
        json_data = json.dumps(data)
        headers = {'Content-Type': 'application/json'}

        try:
            response = requests.post(url, data=json_data, headers=headers)
            self.get_logger().info(f"Status Code: {response.status_code}, Response: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()