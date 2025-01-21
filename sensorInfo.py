from gattlib import GATTRequester, GATTResponse
from struct import unpack
import time
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)

# Define the MAC address of the BLE sensor
address = "5C:85:7E:12:D0:2E"

class SensorDataResponse(GATTResponse):
    def on_response(self, data):
        try:
            # Unpack the data received from handle 0x0035
            temperature, sunlight, moisture, fertility = unpack('<hxIBHxxxxxx', data)
            logging.info(f"Light intensity: {sunlight} lux")
            logging.info(f"Temperature: {temperature / 10.} °C")
            logging.info(f"Soil moisture: {moisture} %")
            logging.info(f"Soil fertility: {fertility} uS/cm")
        except Exception as e:
            logging.error(f"Error unpacking sensor data: {str(e)}")

def main():
    try:
        requester = GATTRequester(address, False)

        while True:
            try:
                # Wait for connection
                logging.info("Connecting to device...")
                requester.connect(True)
                logging.info("Connected to device")

                # Read battery level and firmware version
                try:
                    data = requester.read_by_handle(0x0038)[0]
                    battery, version = unpack('<B6s', data)
                    logging.info(f"Battery level: {battery} %")
                    logging.info(f"Firmware version: {version.decode('utf-8').strip()}")
                except Exception as e:
                    logging.warning(f"Error reading battery or firmware version: {e}")

                # Enable real-time data reading
                requester.write_by_handle(0x0033, bytes([0xa0, 0x1f]))
                logging.info("Enabled real-time data reading")

                # Subscribe to sensor value notifications
                requester.write_by_handle(0x0036, bytes([0x01, 0x00]))
                logging.info("Subscribed to sensor value notifications")

                # Continuously check sensor data
                response = SensorDataResponse()
                while True:
                    try:
                        # Read sensor data asynchronously
                        requester.read_by_handle_async(0x0035, response)
                        time.sleep(5)  # Wait for 5 seconds before reading again
                    except Exception as e:
                        logging.error(f"Error reading sensor data: {str(e)}")
                        break  # Break and reconnect if there's an issue

            except Exception as e:
                logging.error(f"Connection lost: {str(e)}. Retrying...")
                time.sleep(5)  # Wait before reconnecting

    except Exception as e:
        logging.error(f"Fatal error: {str(e)}")
        exit(1)

if __name__ == "__main__":
    main()

