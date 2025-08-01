import serial
import time

def open_serial():
    return serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

try:
    ser = open_serial()
    time.sleep(2)  # Wait for the serial connection to initialize
    
    while True:
        try:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                print(f"Received data: {data}")
                # Here you can process the data as needed
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            ser.close()
            time.sleep(1)
            ser = open_serial()  # Reopen serial connection
except KeyboardInterrupt:
    print("Program interrupted by user")
finally:
    if ser:
        ser.close()
        print("Serial connection closed")