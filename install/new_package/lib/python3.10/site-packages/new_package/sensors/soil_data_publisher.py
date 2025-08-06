import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

def open_serial():
    return serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

class SoilDataPublisher(Node):
    def __init__(self):
        super().__init__('soil_data_publisher')
        self.soil_moisture_publisher = self.create_publisher(Float32, 'soil_data', 10) # publish soil moisture data
        self.timer = self.create_timer(5.0, self.publish_sensor_data)  # Publish every 5 seconds
        self.get_logger().info('Soil Data Publisher Node has been started.')
        self.ser = open_serial()
        time.sleep(2)  # Wait for the serial connection to initialize
        
        # Simulated sensor data
        self.soil_moisture = 55.5  # Example value for soil moisture
        
    def calculate_soil_moisture(self): 
        try:
            data = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                self.soil_moisture = float(data)  # Convert the string data to float
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser.close()
            time.sleep(1)
            self.ser = open_serial()  # Reopen serial connection  

    def publish_sensor_data(self):

        self.calculate_soil_moisture()  # Call the method to read from the serial port
        
        # Publish soil moisture
        soil_moisture_msg = Float32()
        soil_moisture_msg.data = self.soil_moisture
        self.soil_moisture_publisher.publish(soil_moisture_msg)
        self.get_logger().info(f'Published Soil Moisture: {self.soil_moisture}')


def main(args=None):
    rclpy.init(args=args)
    node = SoilDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()