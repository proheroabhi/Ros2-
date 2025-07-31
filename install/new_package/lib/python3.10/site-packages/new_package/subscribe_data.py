import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.task import Future
from std_srvs.srv import Trigger

import requests
import json

class SensorDataSubscriber(Node):
    temperature
    humidity

    def __init__(self):
        super().__init__('sensor_data_subscriber')
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
        self.servo_client = self.create_client(Trigger, 'rotate_servo')
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for servo motor service to become available...')
        self.get_logger().info('Sensor Data Subscriber Node has been started.')

    def soil_moisture_callback(self, msg):
        self.get_logger().info(f'Received Soil Moisture: {msg.data}')

    def temperature_callback(self, msg):
        temperature = msg.data
        self.get_logger().info(f'Received Temperature: {msg.data}')

    def humidity_callback(self, msg):
        humidity = msg.data
        self.get_logger().info(f'Received Humidity: {msg.data}')

        # URL of the web page to which you want to send the request
        url = 'http://192.168.137.1/FarmCS/handlers/receive_sensor_data.php'

        # Create a dictionary with the data you want to send
        data = {
        "soil_moisture": 55.2,
        "temperature": {temperature},
        "humidity": {humidity},
        "light_intensity": 1200,
        "device_id": "FARM_SENSOR_001"
        }

        # Convert the dictionary to a JSON string
        json_data = json.dumps(data)

        # Set headers to inform the server that you're sending JSON
        headers = {
            'Content-Type': 'application/json'
        }

        # Send a POST request with the JSON data
        response = requests.post(url, data=json_data, headers=headers)

        # Print the status code and response from the server
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.text}")

        if msg.data < 60.0:
            self.get_logger().info('Humidity is greater than 60. Calling servo motor service...')
            self.call_servo_motor_service()

    def call_servo_motor_service(self):
        request = Trigger.Request()
        future = self.servo_client.call_async(request)
        future.add_done_callback(self.handle_servo_response)

    def handle_servo_response(self, future: Future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Servo Motor Service Response: {response.message}')
            else:
                self.get_logger().error(f'Servo Motor Service Failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Failed to call Servo Motor Service: {e}')

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