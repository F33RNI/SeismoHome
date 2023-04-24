"""
 Copyright (C) 2023 Fern Lane, SeismoHome earthquake detector project
 Licensed under the GNU Affero General Public License, Version 3.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
       https://www.gnu.org/licenses/agpl-3.0.en.html
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR
 OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
"""

import ctypes
import logging
import multiprocessing
import threading
import time

import numpy as np
import serial

PACKET_SOH = 0x01
PACKET_EOT = 0x04
PACKET_ESC = 0x1B


class SerialHandler:
    def __init__(self, config: dict) -> None:
        """
        ALL METHODS OF THIS CLASS MUST BE CALLED FROM MAIN PROCESS BECAUSE PYSERIAL DOES NOT WORK WITH MULTIPROCESSING
        :param config:
        """
        self.config = config

        # Create queue for acceleration data
        self.accelerations_queue = multiprocessing.Queue()

        # Initialize multiprocessing variables for data RX
        self.battery_voltage_mv = multiprocessing.Value(ctypes.c_uint16, 0)
        self.power_state = multiprocessing.Value(ctypes.c_uint8, 0)
        self.battery_low = multiprocessing.Value(ctypes.c_bool, False)
        self.temperature = multiprocessing.Value(ctypes.c_int8, 0)
        self.button_flag = multiprocessing.Value(ctypes.c_bool, False)

        # Initialize multiprocessing variables for data TX
        self.alarm_state = multiprocessing.Value(ctypes.c_uint8, 0)
        self.clear_button_flag = multiprocessing.Value(ctypes.c_bool, False)
        self.calibration_state = multiprocessing.Value(ctypes.c_uint8, 0)

        # Loop running flags
        self.reader_thread_running = False
        self.sender_running = False

        # Thread for reading from serial port
        self.reader_thread = None

        # Serial port
        self.serial_port = None

    def start(self) -> None:
        """
        Initializes logging and starts reader as thread and start sender loop (blocking)
        :return:
        """
        # Start reader thread
        self.reader_thread_running = True
        self.reader_thread = threading.Thread(target=self.reader_loop)
        logging.info("Starting reader_loop() thread with name: " + str(self.reader_thread.name))
        self.reader_thread.start()
        time.sleep(1)

        # Go and stay into sender loop
        self.sender_running = True
        self.sender_loop()

    def stop_all(self) -> None:
        """
        Stops all threads and closes serial port
        :return:
        """
        # Stop loops
        self.sender_running = False
        self.reader_thread_running = False

        # Join reader loop
        if self.reader_thread is not None and self.reader_thread.is_alive():
            self.reader_thread.join()

    def sender_loop(self) -> None:
        """
        Constantly sends data to the serial port
        :return:
        """
        # TX buffer (5 bytes including checksum, big-ending)
        # alarm_state stream_enabled clear_button_flag calibration_state checksum
        #      0            1                2                 3            4
        tx_buffer = [0] * 5

        # Permanently enable stream
        tx_buffer[1] = 1

        # Infinite loop
        while self.sender_running:
            try:
                # Set alarm state
                tx_buffer[0] = self.alarm_state.value

                # Set clear_button_flag
                tx_buffer[2] = self.clear_button_flag.value

                # Set calibration state
                tx_buffer[3] = self.calibration_state.value

                # Calculate checksum
                tx_buffer[4] = 0
                for i in range(4):
                    tx_buffer[4] = (tx_buffer[4] ^ tx_buffer[i]) & 0xFF

                # Write SOH (start-of-header) byte as packet start
                self.serial_port.write(bytes([PACKET_SOH]))

                # Write data to serial port and replace SOH / EOT / ESC bytes with ESC + SOH / EOT / ESC (escaping them)
                for i in range(len(tx_buffer)):
                    if tx_buffer[i] == PACKET_SOH or tx_buffer[i] == PACKET_EOT or tx_buffer[i] == PACKET_ESC:
                        self.serial_port.write(bytes([PACKET_ESC]))
                    self.serial_port.write(bytes([tx_buffer[i]]))

                # Write EOT (end-of-transmission) byte as packet end
                self.serial_port.write(bytes([PACKET_EOT]))

                # Sleep for next cycle
                time.sleep(float(self.config["send_data_period"]))

            # Exit requested
            except KeyboardInterrupt:
                logging.warning("KeyboardInterrupt @ sender_loop()")
                break

            # Oh no, error!
            except Exception as e:
                logging.error("Error sending data to serial port!", exc_info=e)
                time.sleep(1)

        # Why are we here?
        logging.warning("sender_loop() finished!")

    def reader_loop(self) -> None:
        """
        Reads and parses data from serial port
        :return:
        """
        # Receive packet (12 bytes, big-ending)
        # acc_x acc_y acc_z battery_voltage_mv_int power_state battery_low temperature button_flag
        #  0 1   2 3   4 5          6 7                 8          9            10          11
        rx_packet = [0] * 12

        # Buffer size = 1 (SOH) + packet size * 2 + 2 (checksum and it's possible escaping byte) + 1 (EOT)
        rx_buffer = [0] * 28
        rx_buffer_cursor = 0

        # Infinite loop for reading data from serial port
        while self.reader_thread_running:
            try:
                # Open serial port
                if self.serial_port is None or not self.serial_port.isOpen:
                    is_opened = False
                    while not is_opened and self.reader_thread_running:
                        logging.info("Trying to open serial port from reader_loop()")
                        is_opened = self.open_port()
                        time.sleep(1)

                # Read one byte from serial port
                rx_buffer[rx_buffer_cursor] = ord(self.serial_port.read())

                #  Consider packet end if we have at least 2 bytes and current byte is EOT and previous byte was not ESC
                if rx_buffer_cursor > 0 \
                        and rx_buffer[rx_buffer_cursor] == PACKET_EOT and rx_buffer[rx_buffer_cursor - 1] != PACKET_ESC:

                    # Parse packet only if we have at least RX_PACKET_SIZE bytes + 1 EOT byte
                    if rx_buffer_cursor > len(rx_packet):

                        # Try to find packet start by searching for SOH without escaping byte
                        rx_buffer_start = 0
                        for i in range(rx_buffer_cursor):
                            # SOH at the first byte, so packet starts from 1 (because we don't need to parse SOH byte)
                            if i == 0 and rx_buffer[i] == PACKET_SOH:
                                rx_buffer_start = 1
                                break

                            # SOH without previous ESC, so packet starts from i + 1
                            # (because we don't need to parse SOH byte)
                            if i > 0 and rx_buffer[i - 1] != PACKET_ESC and rx_buffer[i] == PACKET_SOH:
                                rx_buffer_start = i + 1
                                break

                        # Check if we successfully found packet start
                        if rx_buffer_start > 0 and rx_buffer_start + len(rx_packet) <= len(rx_buffer):

                            # Try to extract packet (without escaping bytes) and calculate checksum at the same time
                            # Stop at rx_buffer_cursor - 2 (Last two bytes are checksum byte and EOT)
                            rx_packet_cursor = 0
                            check_byte = 0
                            i = rx_buffer_start
                            while i <= rx_buffer_cursor - 2:
                                # Stop if we have all packet bytes
                                if rx_packet_cursor >= len(rx_packet):
                                    break

                                # We found escape byte
                                if rx_buffer[i] == PACKET_ESC:
                                    # Stop if we have escape byte before checksum byte
                                    if i + 1 == rx_buffer_cursor - 1:
                                        break

                                    # Append next byte to packet
                                    rx_packet[rx_packet_cursor] = rx_buffer[i + 1]
                                    rx_packet_cursor += 1

                                    # Calculate checksum using only next byte
                                    check_byte = (check_byte ^ rx_buffer[i + 1]) & 0xFF

                                    # Skip one cycle
                                    i += 1

                                    # Skip normal calculation
                                    i += 1
                                    continue

                                # Append data byte
                                rx_packet[rx_packet_cursor] = rx_buffer[i]
                                rx_packet_cursor += 1

                                # Calculate checksum normally (no previous escape byte)
                                check_byte = (check_byte ^ rx_buffer[i]) & 0xFF

                                # Go to next loop cycle
                                i += 1

                            # Check if checksum match
                            if check_byte == rx_buffer[rx_buffer_cursor - 1]:
                                # Parse accelerations data
                                acc_x = int.from_bytes(bytearray(rx_packet[0: 2]), byteorder="big", signed=True)
                                acc_y = int.from_bytes(bytearray(rx_packet[2: 4]), byteorder="big", signed=True)
                                acc_z = int.from_bytes(bytearray(rx_packet[4: 6]), byteorder="big", signed=True)

                                # Convert to m/s^2
                                acc_x = self.acceleration_to_mss(acc_x)
                                acc_y = self.acceleration_to_mss(acc_y)
                                acc_z = self.acceleration_to_mss(acc_z)

                                # Parse battery voltage
                                self.battery_voltage_mv.value \
                                    = int.from_bytes(bytearray(rx_packet[6: 8]), byteorder="big", signed=False)

                                # Parse power state
                                self.power_state.value = rx_packet[8]

                                # Parse battery low flag
                                self.battery_low.value = rx_packet[9] > 0

                                # Parse temperature in Celsius degrees
                                self.temperature.value = (rx_packet[10] + 128) % 256 - 128

                                # Parse button pressed flag
                                self.button_flag.value = rx_packet[11] > 0

                                # Append accelerations data to the queue as numpy array
                                self.accelerations_queue.put(np.array([acc_x, acc_y, acc_z], dtype=np.float32))

                            else:
                                logging.warning("Wrong packet checksum!")

                    # Reset bytes counter
                    rx_buffer_cursor = 0

                else:
                    # Increment bytes counter
                    rx_buffer_cursor += 1

                    # Reset buffer position on overflow
                    if rx_buffer_cursor >= len(rx_buffer):
                        rx_buffer_cursor = 0

            # Exit requested
            except KeyboardInterrupt:
                logging.warning("KeyboardInterrupt @ reader_loop()")
                break

            # Oh no, error!
            except Exception as e:
                logging.error("Error reading data from serial port!", exc_info=e)
                time.sleep(1)

                # Try to close serial port
                if self.serial_port is not None:
                    try:
                        logging.info("Closing serial port")
                        self.serial_port.close()
                        self.serial_port = None
                    except Exception as e:
                        logging.warning("Error closing serial port!", exc_info=e)

        # Try to close serial port
        if self.serial_port is not None:
            try:
                logging.info("Closing serial port")
                self.serial_port.close()
                self.serial_port = None
            except Exception as e:
                logging.warning("Error closing serial port!", exc_info=e)

        # Why are we here?
        logging.warning("reader_loop() finished!")

    def open_port(self) -> bool:
        """
        Tries to open serial port
        :return:
        """
        try:
            logging.info("Trying to open port " + str(self.config["serial_port"])
                         + " @ " + str(self.config["serial_baud_rate"]))
            self.serial_port = None
            self.serial_port = serial.Serial(str(self.config["serial_port"]),
                                             int(self.config["serial_baud_rate"]),
                                             timeout=1)
            self.serial_port.close()
            self.serial_port.open()
            if self.serial_port.isOpen():
                logging.info("Serial port " + str(self.config["serial_port"])
                             + " opened successfully @ " + str(self.config["serial_baud_rate"]))
                return True
            else:
                logging.error("Error opening serial port " + str(self.config["serial_port"]))
                self.serial_port = None
        except Exception as e:
            logging.error("Error opening serial port!", exc_info=e)
            self.serial_port = None

        return False

    def acceleration_to_mss(self, acceleration) -> float:
        """
        Converts acceleration to m/s^2
        :param acceleration: raw acceleration value
        :return: acceleration in m/s^2
        """
        in_min = float(self.config["acceleration_data_range_min"])
        in_max = float(self.config["acceleration_data_range_max"])
        out_min = float(self.config["acceleration_mss_range_min"])
        out_max = float(self.config["acceleration_mss_range_max"])
        return (acceleration - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
