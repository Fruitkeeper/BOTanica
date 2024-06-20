from gattlib import GATTRequester, GATTResponse
from struct import unpack
import time
import serial

# Define the MAC address of the BLE sensor
address = "5C:85:7E:12:D0:2E"

# Function to find the correct serial port
def find_serial_port():
    import glob
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    if not ports:
        raise IOError("No serial ports found")
    return ports[0]

# Initialize serial communication with Arduino
try:
    serial_port = find_serial_port()
    arduino = serial.Serial(serial_port, 9600, timeout=1)  # Adjust the port name as needed
    print(f"Connected to Arduino on port {serial_port}")
except Exception as e:
    print(f"Error initializing serial connection: {e}")
    exit(1)

class SensorDataResponse(GATTResponse):
    def on_response(self, data):
        try:
            # Unpack the data received from handle 0x0035
            temperature, sunlight, moisture, fertility = unpack('<hxIBHxxxxxx', data)
            print("Light intensity:", sunlight, "lux")
            print("Temperature:", temperature / 10., "°C")
            print("Soil moisture:", moisture, "%")
            print("Soil fertility:", fertility, "uS/cm")

            # Send signal to Arduino if light intensity is below 140
            if sunlight < 140:
                arduino.write(b'F')  # Send 'F' to Arduino to move forward
                print("Light intensity below 140 lux, sending signal to Arduino...")
            else:
                arduino.write(b'S')  # Send 'S' to Arduino to stop
        except Exception as e:
            print("Error unpacking sensor data:", str(e))

def main():
    try:
        requester = GATTRequester(address, False)

        # Wait for connection
        print("Connecting to device...")
        requester.connect(True)
        print("Connected to device")

        # Read battery level and firmware version
        data = requester.read_by_handle(0x0038)[0]
        battery, version = unpack('<B6s', data)
        print("Battery level:", battery, "%")
        print("Firmware version:", version.decode('utf-8').strip())

        # Enable real-time data reading
        requester.write_by_handle(0x0033, bytes([0xa0, 0x1f]))
        print("Enabled real-time data reading")

        # Subscribe to sensor value notifications
        requester.write_by_handle(0x0036, bytes([0x01, 0x00]))
        print("Subscribed to sensor value notifications")

        # Create a response handler
        response = SensorDataResponse()

        # Perform 10 readings
        for _ in range(10):
            # Read sensor data asynchronously
            requester.read_by_handle_async(0x0035, response)
            time.sleep(5)  # Wait for 1 second before reading again

    except Exception as e:
        print("An error occurred:", str(e))

if __name__ == "__main__":
    main()