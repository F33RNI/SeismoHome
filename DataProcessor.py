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
import datetime
import json
import logging
import math
import multiprocessing
import os.path
import time
from typing import IO

import numpy as np

import Filter
import LoggingHandler
from SerialHandler import SerialHandler
from WebHandler import WebHandler

FILTERS_ORDER = 2

CALIBRATION_STATE_NO = 0
CALIBRATION_STATE_DELAY = 1
CALIBRATION_STATE_IN_PROCESS = 2
CALIBRATION_STATE_OK = 3

ALARM_STATE_OFF = 0
ALARM_STATE_LOW = 1
ALARM_STATE_HIGH = 2

POWER_STATE_IDLE = 0
POWER_STATE_CHARGING = 1
POWER_STATE_ON_BAT = 2


def compute_fft_mag(data: np.ndarray) -> np.ndarray:
    """
    Computes real fft in signal magnitude (rms)
    :param data: input data (float32)
    :return: fft
    """
    # Generate window
    window = np.hanning(len(data))

    # Multiply by a window
    if window is not None:
        data = data[0:len(data)] * window

    # Calculate real FFT
    real_fft = np.fft.rfft(data)

    # Scale the magnitude of FFT by window and factor of 2
    mag = np.abs(real_fft) * 2 / (len(data) / 2)

    # Return calculated fft
    return mag


def fft_to_jma(fft: np.ndarray) -> np.ndarray:
    """
    Converts fft from m/s^2 to JMA (Japan Meteorological Agency) intensity scale
    :param fft:
    :return:
    """
    # Convert each fft bin to approximated peak ground acceleration in gal
    fft_gal = np.multiply(np.multiply(np.multiply(fft, 2.), math.sqrt(2)), 100)

    # Prevent zero values
    min_value = np.finfo(np.float32).eps
    fft_gal[fft_gal < min_value] = min_value

    # Convert to JMA scale
    fft_jma = np.add(np.multiply(np.log10(fft_gal), 2), .94)

    # Prevent negative values
    fft_jma[fft_jma < 0.] = 0.

    # Return fft in JMA scale
    return fft_jma


def pga_to_jma(pga: float) -> float:
    """
    Converts peak ground acceleration to JMA (Japan Meteorological Agency) intensity scale
    :param pga: peak ground acceleration in m/s^2
    :return: pga 0 to 7+
    """
    # Prevent zero value
    if pga < np.finfo(np.float32).eps:
        pga = np.finfo(np.float32).eps

    # Convert acceleration to gal
    gal = pga * 100.

    # Convert to jma
    jma = 2. * math.log10(gal) + .94

    # Prevent negative value
    if jma < 0.:
        jma = 0.

    return jma


def jma_to_msk(jma: float) -> float:
    """
    Converts (Japan Meteorological Agency) intensity scale to MSK (Medvedev–Sponheuer–Karnik)
    :param jma: 0 - 7
    :return: 0 - 12
    """
    # Convert to msk
    return jma * 1.5 + 1.5


class DataProcessor:
    def __init__(self, config: dict, serial_handler: SerialHandler, web_handler: WebHandler) -> None:
        self.config = config
        self.serial_handler = serial_handler
        self.web_handler = web_handler
        
        self.processor_loop_running = multiprocessing.Value(ctypes.c_bool, False)

    def start_file(self) -> IO:
        """
        Generates new file and opens it for writing in wb mode
        :return:
        """
        # Generate timestamp for filename
        timestamp = datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")

        # Generate format description for filename
        file_format = str(self.config["sampling_rate"]) + "sps__3ch__float32__l_endian"

        # Combine into filename
        filename = timestamp + "__" + file_format + ".raw"

        # Finally, add create output directory and to filename
        if not os.path.exists(self.config["samples_directory"]):
            os.makedirs(self.config["samples_directory"])
        abs_filename = os.path.join(self.config["samples_directory"], filename)

        # Open new file
        logging.info("Starting new file: " + str(abs_filename))

        # Send filename to WebHandler class
        with self.web_handler.lock:
            self.web_handler.active_filename.value = filename.encode("utf-8")

        # Open file for writing and return it
        return open(abs_filename, "wb")

    def processor_loop(self, logging_queue: multiprocessing.Queue):
        # Set loop flag
        self.processor_loop_running.Value = True

        # Setup logging for current process
        LoggingHandler.worker_configurer(logging_queue)

        # File IO
        file = None

        # Get sampling rate from config
        sampling_rate = int(self.config["sampling_rate"])

        # Set chunk size = samplerate to make 1s chunks
        chunk_size = sampling_rate

        # Low-pass and high-pass filters for each axis
        low_pass_filters = []
        high_pass_filters = []
        for _ in range(3):
            low_pass_filters.append(Filter.Filter(Filter.FILTER_TYPE_LOWPASS, sampling_rate,
                                                  float(self.config["low_pass_filter_cutoff"]),
                                                  order=FILTERS_ORDER))
            high_pass_filters.append(Filter.Filter(Filter.FILTER_TYPE_HIGHPASS, sampling_rate,
                                                   float(self.config["high_pass_filter_cutoff"]),
                                                   order=FILTERS_ORDER))

        # Buffer for incoming data
        chunk = np.zeros((chunk_size, 3), dtype=np.float32)
        chunk_cursor = 0

        # Calculated magnitudes
        pgas = np.zeros(3, dtype=np.float32)
        jmas = np.zeros(3, dtype=np.float32)
        msks = np.zeros(3, dtype=np.float32)
        jma_current = 0.
        msk_current = 0.
        low_intensity_chunks_counter = 0
        high_intensity_chunks_counter = 0
        jma_file_max = 0.
        msk_file_max = 0.

        # Calibration variables
        calibration_state = CALIBRATION_STATE_NO
        pga_calibration_buffer = np.zeros((3, int(self.config["calibration_chunks"])), dtype=np.float32)
        pga_calibrations = np.zeros(3, dtype=np.float32)
        pga_calibration_buffer_position = 0
        calibration_delay_counter = 0

        # FFTs
        ffts_prev = np.zeros((3, chunk_size // 2 + 1), dtype=np.float32)
        ffts = np.zeros((3, chunk_size // 2 + 1), dtype=np.float32)

        # Linear gradient of two FFTs (for webpage)
        ffts_linspace = np.zeros((3, chunk_size // 2 + 1, chunk_size + 1), dtype=np.float32)

        # Counter for file size
        file_size_bytes = 0

        # Variable to store when alarm was enabled
        alarm_enabled_time = 0

        while self.processor_loop_running.Value:
            try:
                # Get new data
                accelerations = self.serial_handler.accelerations_queue.get(block=True)

                # Filter each axis independently
                for i in range(3):
                    # Apply low-pass filter
                    accelerations[i] = low_pass_filters[i].filter(accelerations[i])

                    # Apply high-pass filter
                    accelerations[i] = high_pass_filters[i].filter(accelerations[i])

                # Check json queue
                if not self.web_handler.json_packets_queue.full():
                    # Prepare some data for json packet
                    alarm_state_str = "Off"
                    if self.web_handler.trigger_alarm.value == ALARM_STATE_LOW:
                        alarm_state_str = "Test low"
                    elif self.web_handler.trigger_alarm.value == ALARM_STATE_HIGH:
                        alarm_state_str = "Test high"
                    elif self.serial_handler.alarm_state.value == ALARM_STATE_HIGH:
                        alarm_state_str = "High"
                    elif self.serial_handler.alarm_state.value == ALARM_STATE_LOW:
                        alarm_state_str = "Low"

                    battery_state_str = ""
                    if self.serial_handler.battery_low.value:
                        battery_state_str += "Low"
                    else:
                        battery_state_str += "Normal"
                    if self.serial_handler.power_state.value == POWER_STATE_ON_BAT:
                        battery_state_str += ", On battery"
                    elif self.serial_handler.power_state.value == POWER_STATE_CHARGING:
                        battery_state_str += ", Charging"
                    else:
                        battery_state_str += ", Connected"

                    # Create data packet for webpage
                    dict_packet = {
                        "timestamp": round(time.time() * 1000),
                        "intensity_jma": float(jma_current),
                        "intensity_msk": float(msk_current),
                        "intensity_jma_max": float(jma_file_max),
                        "intensity_msk_max": float(msk_file_max),
                        "battery_voltage_mv": self.serial_handler.battery_voltage_mv.value,
                        "battery_state": battery_state_str,
                        "temperature": self.serial_handler.temperature.value,
                        "alarm_state": alarm_state_str,
                        "calibration_state": calibration_state,
                        "accelerations": chunk[chunk_cursor].tolist(),
                        "ffts": ffts_linspace[:, :, chunk_cursor].tolist(),
                        "fft_range_from": 0,
                        "fft_range_to": sampling_rate // 2,
                    }

                    # Append packet to the queue
                    self.web_handler.json_packets_queue.put(json.dumps(dict_packet))

                # Write new filtered data to buffer
                chunk[chunk_cursor] = accelerations
                chunk_cursor += 1

                # Buffer is full
                if chunk_cursor >= len(chunk):
                    # Read button state (hardware or virtual)
                    button_flag = self.serial_handler.button_flag.value or self.web_handler.button_flag.value

                    # Clear hardware button
                    self.serial_handler.clear_button_flag.value = self.serial_handler.button_flag.value

                    # Clear virtual button
                    self.web_handler.button_flag.value = False

                    # Reset buffer position
                    chunk_cursor = 0

                    # Transpose chunk to (3, chunk_size), so len(chunk) will be = 3
                    chunk = chunk.transpose()

                    # Process each axis independently
                    for i in range(3):
                        # Store previous FFTs
                        ffts_prev[i, :] = ffts[i, :]

                        # Calculate FFT in JMA scale
                        ffts[i] = fft_to_jma(compute_fft_mag(chunk[i]))

                        # Calculate FFT gradient
                        for fft_bin_n in range(len(ffts[i])):
                            ffts_linspace[i][fft_bin_n] = np.linspace(ffts_prev[i][fft_bin_n],
                                                                      ffts[i][fft_bin_n],
                                                                      num=chunk_size + 1)

                        # Calculate RMS value of chunk
                        rms = np.sqrt(np.mean(np.square(chunk[i])))

                        # Approximately convert EMS to peak ground acceleration
                        pgas[i] = rms * 2. * math.sqrt(2)

                        # Check calibration
                        if calibration_state == CALIBRATION_STATE_OK:
                            # Apply calibration
                            pgas[i] -= pga_calibrations[i]
                            if pgas[i] < 0.:
                                pgas[i] = 0.

                            # Calculate intensities
                            jmas[i] = pga_to_jma(pgas[i])
                            msks[i] = jma_to_msk(jmas[i])

                        # Set zero intensities because we are not calibrated
                        else:
                            jmas[i] = 0
                            msks[i] = 0

                    # First start calibration
                    if calibration_state == CALIBRATION_STATE_NO:
                        calibration_delay_counter = int(self.config["calibration_initial_delay"])
                        calibration_state = CALIBRATION_STATE_DELAY
                        logging.info("Starting PGA calibration after " + str(calibration_delay_counter) + "s")

                    # Requested calibration from physical button or from web page
                    elif calibration_state == CALIBRATION_STATE_OK and button_flag:
                        calibration_delay_counter = int(self.config["calibration_button_delay"])
                        calibration_state = CALIBRATION_STATE_DELAY
                        logging.info("Starting PGA calibration after " + str(calibration_delay_counter) + "s")

                    # Waiting delay
                    if calibration_state == CALIBRATION_STATE_DELAY:
                        # Subtract calibration delay counter
                        if calibration_delay_counter >= 0:
                            calibration_delay_counter -= 1

                        # Delay done
                        else:
                            # Start calibration
                            calibration_state = CALIBRATION_STATE_IN_PROCESS
                            pga_calibration_buffer_position = 0
                            logging.info("Started PGA calibration for "
                                         + str(len(pga_calibration_buffer[0])) + "s")

                    if calibration_state == CALIBRATION_STATE_IN_PROCESS:
                        # Calibration in process
                        if pga_calibration_buffer_position < len(pga_calibration_buffer[0]):
                            for i in range(3):
                                pga_calibration_buffer[i][pga_calibration_buffer_position] = pgas[i]
                            pga_calibration_buffer_position += 1

                        # Calibration done
                        else:
                            calibration_state = CALIBRATION_STATE_OK
                            for i in range(3):
                                pga_calibrations[i] = np.average(pga_calibration_buffer[i])
                            logging.info("PGA calibration done! XYZ values: " + str(pga_calibrations))

                    # Set calibration led state
                    if calibration_state == CALIBRATION_STATE_NO \
                            or calibration_state == CALIBRATION_STATE_DELAY:
                        self.serial_handler.calibration_state.value = 0
                    elif calibration_state == CALIBRATION_STATE_IN_PROCESS:
                        self.serial_handler.calibration_state.value = 1
                    else:
                        self.serial_handler.calibration_state.value = 2

                    # Calculate intensities
                    jma_current = np.max(jmas)
                    msk_current = np.max(msks)

                    # Calculate maximum intensity in current file
                    if jma_current > jma_file_max:
                        jma_file_max = jma_current
                    if msk_current > msk_file_max:
                        msk_file_max = msk_current

                    # Starting / stopping alarm
                    if calibration_state == CALIBRATION_STATE_OK:
                        # Count intensities
                        if jma_current >= self.web_handler.alarm_enable_high_jma.value:
                            high_intensity_chunks_counter += 1
                            logging.warning("Current intensity (" + str(round(jma_current, 2))
                                            + ") is above HIGH threshold("
                                            + str(self.web_handler.alarm_enable_high_jma.value) + ")!")
                        elif high_intensity_chunks_counter > 0:
                            high_intensity_chunks_counter -= 1

                        if jma_current >= self.web_handler.alarm_enable_low_jma.value:
                            low_intensity_chunks_counter += 1
                            logging.warning("Current intensity (" + str(round(jma_current, 2))
                                            + ") is above LOW threshold("
                                            + str(self.web_handler.alarm_enable_low_jma.value) + ")!")
                        elif low_intensity_chunks_counter > 0:
                            low_intensity_chunks_counter -= 1

                        # Real data
                        if high_intensity_chunks_counter >= int(self.config["alarm_high_threshold_chunks"]):
                            logging.warning("Starting alarm in HIGH mode!")
                            self.serial_handler.alarm_state.value = ALARM_STATE_HIGH

                            # Disable test trigger is we have real data
                            self.web_handler.trigger_alarm.value = ALARM_STATE_OFF

                            # Store alarm start time
                            alarm_enabled_time = time.time()

                        elif low_intensity_chunks_counter >= int(self.config["alarm_low_threshold_chunks"]):
                            # Trigger alarm to LOW only if it is not in HIGH mode
                            if self.serial_handler.alarm_state.value != ALARM_STATE_HIGH:
                                logging.warning("Starting alarm in LOW mode!")
                                self.serial_handler.alarm_state.value = ALARM_STATE_LOW

                            # Disable test trigger is we have real data
                            self.web_handler.trigger_alarm.value = ALARM_STATE_OFF

                            # Store alarm start time
                            alarm_enabled_time = time.time()

                        # Test (if alarm is in lower mode than trigger, and we have trigger)
                        elif self.web_handler.trigger_alarm.value != ALARM_STATE_OFF and \
                                self.serial_handler.alarm_state.value < self.web_handler.trigger_alarm.value:
                            self.serial_handler.alarm_state.value = self.web_handler.trigger_alarm.value

                            # Store alarm start time
                            alarm_enabled_time = time.time()

                    # Stop alarm if button was pressed or time has passed or not calibrated
                    if self.serial_handler.alarm_state.value != ALARM_STATE_OFF:
                        if button_flag \
                                or time.time() - alarm_enabled_time > self.web_handler.alarm_active_time_s.value \
                                or calibration_state != CALIBRATION_STATE_OK:
                            logging.info("Stopping alarm!")
                            self.serial_handler.alarm_state.value = ALARM_STATE_OFF
                            self.web_handler.trigger_alarm.value = ALARM_STATE_OFF
                            alarm_enabled_time = 0
                            high_intensity_chunks_counter = 0
                            low_intensity_chunks_counter = 0

                    # Transpose chunk back to (chunk_size, 3), so len(chunk) will be = chunk_size
                    chunk = chunk.transpose()

                    # Start new file
                    if file is None:
                        file = self.start_file()
                        file_size_bytes = 0
                        jma_file_max = 0.
                        msk_file_max = 0.

                    # Write data to file
                    file.write(chunk.tobytes(order="C"))
                    file.flush()

                    # Add new bytes size (chunk size * 3 channels * 4 bytes per sample)
                    file_size_bytes += chunk_size * 3 * 4

                    # Stop file if target size reached or button was pressed or request_file_close has been set
                    if file is not None \
                            and (file_size_bytes + (chunk_size * 3 * 4) >= int(self.config["limit_size_to_bytes"])
                                 or button_flag
                                 or self.web_handler.request_file_close.value):
                        logging.info("Closing file")
                        try:
                            file.flush()
                            file.close()
                        except Exception as e:
                            logging.error("Error closing file!", exc_info=e)
                        file = None
                        self.web_handler.request_file_close.value = False
                        jma_file_max = 0.
                        msk_file_max = 0.

            # Exit requested
            except KeyboardInterrupt:
                logging.warning("KeyboardInterrupt @ processor_loop()")
                break

            # Oh no, error!
            except Exception as e:
                logging.error("Error processing data!", exc_info=e)
                time.sleep(1)

        # Why are we here?
        logging.warning("processor_loop() finished!")
