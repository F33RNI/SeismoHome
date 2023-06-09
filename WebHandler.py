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
import glob
import json
import logging
import math
import multiprocessing
import os.path
import struct
import time
from queue import Empty

import numpy as np
from flask import Flask, Response, render_template, request, stream_with_context

import DataProcessor
import LoggingHandler
from main import __version__

FILE_CHUNK_SIZE = 2048


class WebHandler:
    def __init__(self, config: dict):
        self.config = config

        # Multiprocessing queue for JSON data (limit to 1 second)
        self.json_packets_queue = multiprocessing.Queue(maxsize=int(self.config["sampling_rate"]))

        # Multiprocessing value for virtual hardware button
        self.button_flag = multiprocessing.Value(ctypes.c_bool, False)

        # Multiprocessing value for alarm triggers (for alarm test) 0 - OFF, 1 - LOW, 2 - HIGH
        self.trigger_alarm = multiprocessing.Value(ctypes.c_int8, 0)

        # Multiprocessing thresholds for alarm (for DataProcessor class)
        self.alarm_enable_low_jma = multiprocessing.Value(ctypes.c_float, 7.)
        self.alarm_enable_high_jma = multiprocessing.Value(ctypes.c_float, 7.)
        self.alarm_active_time_s = multiprocessing.Value(ctypes.c_uint16, 0)

        # Current filename to make sure it closed (DataProcessor class will set this variable)
        self.active_filename = multiprocessing.Array("c", 512)

        # Lock for proper reading self.active_filename value
        self.lock = multiprocessing.Lock()

        # We set this to True if we need to close currently processed file
        self.request_file_close = multiprocessing.Value(ctypes.c_bool, False)

        # -1 - real-time data stream, >= 0 - file index
        self.stream_mode = multiprocessing.Value(ctypes.c_int32, -1)
        self.stream_mode_last = multiprocessing.Value(ctypes.c_int32, -1)

        # Temp file to stream as packets
        self._file_for_stream = None

    def file_converter(self, file: dict, to_format: str) -> bool:
        """
        Converts raw file to wav or csv
        :param file: file to convert (from self.files_list)
        :param to_format: "wav" or "csv"
        :return: True if converted successfully
        """
        try:
            # Check output format
            if to_format == "wav" or to_format == "csv":
                # Generate input and output filenames and filepath
                filename = file["filename"]
                filename_without_format = str(file["timestamp_start"]).replace(".", "_") \
                    .replace(":", "_").replace(" ", "__")

                filepath = os.path.join(self.config["samples_directory"], filename + ".raw")
                output_filename = filename_without_format + "." + to_format
                output_filepath = os.path.join(self.config["samples_directory"], output_filename)
                output_file = None

                # Get initial timestamp and sampling rate
                timestamp_millis = 0.
                sampling_rate = int(file["sampling_rate"])

                # Convert file by 12 bytes (3 channels * 4 bytes per sample)
                with open(filepath, "rb") as file:
                    while True:
                        buf = file.read(12)
                        if len(buf) == 12:
                            # Convert to CSV
                            if to_format == "csv":
                                # Start new file and write header
                                if output_file is None:
                                    output_file = open(output_filepath, "w", encoding="utf-8")
                                    output_file.write("Timestamp (ms),"
                                                      "Acceleration X (m/s^2),"
                                                      "Acceleration Y (m/s^2),"
                                                      "Acceleration Z (m/s^2)\n")

                                # Get accelerations
                                [acc_x] = struct.unpack("<f", bytearray(buf[0: 4]))
                                [acc_y] = struct.unpack("<f", bytearray(buf[4: 8]))
                                [acc_z] = struct.unpack("<f", bytearray(buf[8: 12]))

                                # Round to 3 decimals
                                acc_x = round(acc_x, 3)
                                acc_y = round(acc_y, 3)
                                acc_z = round(acc_z, 3)

                                # Write to file
                                row = str(int(timestamp_millis)) + ","
                                row += str(acc_x) + "," + str(acc_y) + "," + str(acc_z)
                                row += "\n"
                                output_file.write(row)

                            # Convert to WAV
                            else:
                                # Start new file and write header
                                if output_file is None:
                                    output_file = open(output_filepath, "wb")
                                    bytes_in_file = os.path.getsize(filepath) // 3 * 3
                                    num_channels = 3
                                    sample_width = 4
                                    wav_header = struct.pack('<4sI4s4sIHHIIHH4sI',
                                                             b'RIFF', 36 + bytes_in_file, b'WAVE',
                                                             b'fmt ', 16, 3, num_channels, sampling_rate,
                                                             sampling_rate * num_channels * sample_width,
                                                             num_channels * sampling_rate, sample_width * 8, b'data',
                                                             bytes_in_file)
                                    output_file.write(wav_header)

                                # Write data
                                output_file.write(buf)
                        else:
                            break

                        # Increment timestamp
                        timestamp_millis += (1 / sampling_rate) * 1000

                # Try to close output file
                if output_file is not None:
                    output_file.close()

                # Set class variables and return true if file exists
                if os.path.exists(output_filepath):
                    self.filename = output_filename
                    self.filepath = output_filepath
                    return True

            else:
                raise ("Wrong format to convert to: " + to_format)

        # Error
        except Exception as e:
            logging.error("Error converting file!", exc_info=e)
        return False

    def download_file(self) -> Response:
        """
        Implements downloading a file as a stream
        :return:
        """

        def downloader_stream(filepath):
            # Stream file content by chunks
            with open(filepath, "rb") as file:
                while True:
                    buf = file.read(FILE_CHUNK_SIZE)
                    if buf:
                        yield buf
                    else:
                        break
                logging.info("Done downloading file!")

            # If we need to delete temp file after downloading it
            if self.delete_after_download:
                self.delete_after_download = False
                try:
                    logging.info("Deleting temp file: " + filepath)
                    os.remove(filepath)
                    if not os.path.exists(filepath):
                        logging.info("Deleted successfully")
                    else:
                        logging.error("Error deleting temp file!")
                except Exception as e:
                    logging.error("Error deleting temp file!", exc_info=e)

        # Check if we have file in queue
        if len(self.filename) > 0 and len(self.filepath) > 0 and os.path.exists(self.filepath):
            # Start downloading
            logging.info("Starting file downloading")
            return Response(
                stream_with_context(downloader_stream(self.filepath)),
                headers={
                    "Content-Disposition": f"attachment; filename={self.filename}"
                }
            )

        # Nothing to download
        else:
            return Response(status=404)

    def virtual_button(self) -> Response:
        """
        Presses virtual hardware button
        :return:
        """
        logging.info("Pressing hardware button")
        self.button_flag.value = True
        return Response(status=200)

    def test_alarm(self) -> Response:
        """
        Requests alarm test (low and high modes)
        :return:
        """
        if request.method == "POST":
            content = request.json
            if str(content["type"]).lower() == "low":
                logging.warning("Testing alarm on LOW mode!")
                self.trigger_alarm.value = DataProcessor.ALARM_STATE_LOW
            elif str(content["type"]).lower() == "high":
                logging.warning("Testing alarm on HIGH mode!")
                self.trigger_alarm.value = DataProcessor.ALARM_STATE_HIGH
            return Response(status=200)
        else:
            return Response(status=400)

    def get_post_alarm_config(self) -> Response:
        """
        Returns or saves alarm config as json data
        :return:
        """
        # POST request - save config, Other - get config
        if request.method == "POST":
            from main import save_json, ALARM_CONFIG_FILE
            content = request.json
            save_json(ALARM_CONFIG_FILE, content)
            self.alarm_enable_low_jma.value = content["alarm_enable_low_jma"]
            self.alarm_enable_high_jma.value = content["alarm_enable_high_jma"]
            self.alarm_active_time_s.value = content["alarm_active_time_s"]
            return Response(status=200)
        else:
            from main import load_json, ALARM_CONFIG_FILE
            alarm_config = load_json(ALARM_CONFIG_FILE)
            self.alarm_enable_low_jma.value = alarm_config["alarm_enable_low_jma"]
            self.alarm_enable_high_jma.value = alarm_config["alarm_enable_high_jma"]
            self.alarm_active_time_s.value = alarm_config["alarm_active_time_s"]
            return Response(json.dumps(alarm_config), status=200, content_type="application/json")

    def files(self) -> Response:
        """
        Returns list of available files and takes actions with file
        :return:
        """
        # POST request - download / delete file, Other - get list of files
        if request.method == "POST":
            # Check flag to prevent clicking again on button
            if not self.is_file_processing:
                try:
                    # Set lock
                    self.is_file_processing = True

                    # Try to find requested file
                    file = None
                    for i in range(len(self.files_list)):
                        if self.files_list[i]["filename"] == str(request.json["filename"]):
                            file = self.files_list[i]
                            break

                    # Check file
                    if file is None:
                        logging.error("Wrong filename: " + str(request.json["filename"]))
                        self.is_file_processing = False
                        return Response(status=440)

                    # Extract requested action
                    action = str(request.json["action"])

                    # Check action
                    if action == "raw" or action == "wav" or action == "csv" or action == "delete":
                        # Get filename
                        filename = file["filename"]

                        # Append extension
                        if not filename.lower().endswith(".raw"):
                            filename += ".raw"

                        # Check if file exist
                        filepath = os.path.join(self.config["samples_directory"], filename)
                        if os.path.exists(filepath):
                            # Get active_filename value
                            with self.lock:
                                active_filename = self.active_filename.value.decode("utf-8")

                            # Check if file is active and request close
                            if filename == active_filename:
                                logging.warning("Requested action with currently active file! Waiting until it closes")
                                self.request_file_close.value = True
                                while filename == active_filename:
                                    with self.lock:
                                        active_filename = self.active_filename.value.decode("utf-8")
                                    time.sleep(0.1)

                            # Delete file
                            if action == "delete":
                                logging.info("Deleting file...")
                                # Stop file stream
                                self.stream_mode.value = -1
                                time.sleep(1)
                                os.remove(filepath)
                                if not os.path.exists(filepath):
                                    logging.info("Deleted successfully")
                                    self.is_file_processing = False
                                    return Response(status=200)
                                else:
                                    logging.error("Error deleting file!")
                                    self.is_file_processing = False
                                    return Response(status=500)

                            # Download RAW file
                            elif action == "raw":
                                logging.info("Downloading file as RAW")
                                self.filename = filename
                                self.filepath = filepath
                                self.is_file_processing = False
                                self.delete_after_download = False
                                return Response(status=200)

                            # Download WAV file
                            elif action == "wav":
                                logging.info("Downloading file as WAV")
                                if self.file_converter(file, "wav"):
                                    self.is_file_processing = False
                                    self.delete_after_download = True
                                    return Response(status=200)

                                # Unable to convert
                                else:
                                    self.is_file_processing = False
                                    return Response(status=500)

                            # Download CSV file
                            elif action == "csv":
                                logging.info("Downloading file as CSV")
                                if self.file_converter(file, "csv"):
                                    self.is_file_processing = False
                                    self.delete_after_download = True
                                    return Response(status=200)

                                # Unable to convert
                                else:
                                    self.is_file_processing = False
                                    return Response(status=500)

                        # File not exists
                        else:
                            logging.error("File " + filepath + " not exists!")
                            self.is_file_processing = False
                            return Response(status=404)
                # Error while processing file
                except Exception as e:
                    logging.error("Error processing file!", exc_info=e)

                # Release flag
                self.is_file_processing = False

            # Send too many requests if already processing
            else:
                return Response(status=429)

            return Response(status=400)
        else:
            raw_files = [f for f in glob.glob(os.path.join(str(self.config["samples_directory"]), "*.raw"))]
            self.files_list = []
            for raw_file in raw_files:
                try:
                    if os.path.exists(raw_file) and os.path.getsize(raw_file) > 0:
                        filename = os.path.splitext(os.path.basename(raw_file))[0].strip()
                        filename_parts = filename.split("__")

                        # Check format. Currently, supports only 3ch float32 little-endian
                        # YYYY_MM_DD HH_MM_SS 40sps 3ch float32 l_endian
                        if len(filename_parts) == 6 \
                                and filename_parts[3] == "3ch" \
                                and filename_parts[4] == "float32" \
                                and filename_parts[5] == "l_endian":
                            # Parse datetime string into a datetime object in UTC
                            dt_utc = datetime.datetime.strptime(filename_parts[0] + "__" + filename_parts[1],
                                                                "%Y_%m_%d__%H_%M_%S").replace(tzinfo=None)

                            # Convert datetime object from UTC to local time
                            tz_offset = time.timezone if (time.localtime().tm_isdst == 0) else time.altzone
                            dt_local = dt_utc - datetime.timedelta(seconds=tz_offset)

                            # Get timestamp in seconds
                            timestamp_seconds = int(dt_local.timestamp())

                            # Get file sampling rate
                            sampling_rate = int(str(filename_parts[2]).lower().replace("sps", "").strip())

                            # Calculate number of seconds (12 bytes per one sample)
                            file_length = int((os.path.getsize(raw_file) / 12) / sampling_rate)

                            # Get start and end time of file
                            timestamp_start = time.strftime("%d.%m.%Y %H:%M:%S",
                                                            time.gmtime(timestamp_seconds))
                            timestamp_end = time.strftime("%d.%m.%Y %H:%M:%S",
                                                          time.gmtime(timestamp_seconds + file_length))

                            # Get active_filename value
                            with self.lock:
                                active_filename = os.path.splitext(self.active_filename.value.decode("utf-8"))[0]

                            # Append filename, timestamp and length to list
                            dictionary = {
                                "filename": filename,
                                "filepath": raw_file,
                                "timestamp_seconds": timestamp_seconds,
                                "timestamp_start": timestamp_start,
                                "timestamp_end": "Now" if filename == active_filename else timestamp_end,
                                "file_length": file_length,
                                "sampling_rate": sampling_rate
                            }
                            self.files_list.append(dictionary)
                except Exception as e:
                    logging.warning("Error checking file " + raw_file + "!", exc_info=e)

            # Log available files
            logging.info("Found: " + str(len(self.files_list)) + " files!")

            # Sort by timestamp
            self.files_list = sorted(self.files_list,
                                     key=lambda x: x["timestamp_seconds"],
                                     reverse=True)

            # Return as JSON array
            return Response(json.dumps(self.files_list), status=200, content_type="application/json")

    def stream_mode_change(self) -> Response:
        """
        Request to change stream mode ({"filename": "-"} for real-time stream, or {"filename": "FILE"} for file)
        :return:
        """
        request_filename = str(request.json["filename"])
        logging.info("Requested stream mode: " + request_filename)

        # File streaming
        if request_filename != "-":
            # Try to find requested file
            file = None
            file_index = 0
            for i in range(len(self.files_list)):
                if self.files_list[i]["filename"] == request_filename:
                    file = self.files_list[i]
                    file_index = i
                    break

            # Check file
            if file is None:
                self.stream_mode.value = -1
                return Response(status=440)

            # File OK
            else:
                self.stream_mode.value = file_index
                return Response(status=200)

        # Real-time streaming
        else:
            self.stream_mode.value = -1
            return Response(status=200)

    def get_file_stream_data(self, chunk,
                             chunk_cursor,
                             ffts_prev,
                             ffts,
                             sample_counter,
                             ffts_linspace,
                             pgas,
                             jmas,
                             msks,
                             jma_peak,
                             msk_peak) -> (str, int, int, int, int):
        """
        Sends file as packets stream
        :return:
        """
        result = ""

        # Calculate and stream file content
        if self._file_for_stream is not None:
            # Read 12 bytes (3 channels * 4 bytes per sample)
            buf = self._file_for_stream.read(12)
            if len(buf) == 12:
                # Calculate timestamp
                timestamp_seconds = self.files_list[self.stream_mode_last.value]["timestamp_seconds"]
                timestamp_seconds += sample_counter / self.files_list[self.stream_mode_last.value]["sampling_rate"]

                # Create data packet for webpage
                dict_packet = {
                    "timestamp": round(timestamp_seconds * 1000),
                    "stream_mode": self.stream_mode.value,
                    "intensity_jma_peak": float(jma_peak),
                    "intensity_msk_peak": float(msk_peak),
                    "accelerations": chunk[chunk_cursor].tolist(),
                    "ffts": ffts_linspace[:, :, chunk_cursor].tolist(),
                    "fft_range_from": 0,
                    "fft_range_to": int(self.config["low_pass_filter_cutoff"]),
                }

                # Parse data
                [acc_x] = struct.unpack("<f", bytearray(buf[0: 4]))
                [acc_y] = struct.unpack("<f", bytearray(buf[4: 8]))
                [acc_z] = struct.unpack("<f", bytearray(buf[8: 12]))

                # Write new filtered data to buffer
                chunk[chunk_cursor][0] = acc_x
                chunk[chunk_cursor][1] = acc_y
                chunk[chunk_cursor][2] = acc_z
                chunk_cursor += 1

                # Buffer is full
                if chunk_cursor >= len(chunk):
                    # Reset buffer position
                    chunk_cursor = 0

                    # Transpose chunk to (3, chunk_size), so len(chunk) will be = 3
                    chunk = chunk.transpose()

                    # Process each axis independently
                    for i in range(3):
                        # Store previous FFTs
                        ffts_prev[i, :] = ffts[i, :]

                        # Calculate FFT in JMA scale
                        ffts[i] = DataProcessor.fft_to_jma(DataProcessor.compute_fft_mag(chunk[i]))

                        # Calculate FFT gradient
                        for fft_bin_n in range(len(ffts[i])):
                            ffts_linspace[i][fft_bin_n] = np.linspace(ffts_prev[i][fft_bin_n],
                                                                      ffts[i][fft_bin_n],
                                                                      num=len(chunk[0]) + 1)
                        # Calculate RMS value of chunk
                        rms = np.sqrt(np.mean(np.square(chunk[i])))

                        # Approximately convert EMS to peak ground acceleration
                        pgas[i] = rms * 2. * math.sqrt(2)
                        if pgas[i] < 0.:
                            pgas[i] = 0.

                        # Calculate intensities
                        jmas[i] = DataProcessor.pga_to_jma(pgas[i])
                        msks[i] = DataProcessor.jma_to_msk(jmas[i])

                # Calculate intensities
                jma_current = np.max(jmas)
                msk_current = np.max(msks)

                # Calculate maximum (peak) intensities\
                if jma_current > jma_peak:
                    jma_peak = jma_current
                if msk_current > msk_peak:
                    msk_peak = msk_current

                # Increment number of sample (for timestamp calculations)
                sample_counter += 1

                # Return packet as json string
                return json.dumps(dict_packet), chunk_cursor, sample_counter, jma_peak, msk_peak

        return result, chunk_cursor, sample_counter, jma_peak, msk_peak

    def stream(self) -> Response:
        """
        Streams data to web
        :return: data stream
        """
        # Data stream
        if request.headers.get("accept") == "text/event-stream":
            def stream():
                logging.info("Starting data stream to web page")

                # Clear queue
                try:
                    while True:
                        self.json_packets_queue.get_nowait()
                except Empty:
                    pass

                # Reset stream mode
                self.stream_mode.value = -1
                self.stream_mode_last.value = -1

                # chunk size = samplerate to make 1s chunks
                chunk_size = int(self.config["sampling_rate"])

                # Buffer for data from file
                chunk = np.zeros((chunk_size, 3), dtype=np.float32)
                chunk_cursor = 0
                sample_counter = 0

                # Intensities
                pgas = np.zeros(3, dtype=np.float32)
                jmas = np.zeros(3, dtype=np.float32)
                msks = np.zeros(3, dtype=np.float32)
                jma_peak = 0
                msk_peak = 0

                # FFTs
                ffts_prev = np.zeros((3, chunk_size // 2 + 1), dtype=np.float32)
                ffts = np.zeros((3, chunk_size // 2 + 1), dtype=np.float32)
                ffts_linspace = np.zeros((3, chunk_size // 2 + 1, chunk_size + 1), dtype=np.float32)

                # Start data stream
                while True:
                    # Start new file and clear data
                    if self.stream_mode.value != self.stream_mode_last.value:
                        self.stream_mode_last.value = self.stream_mode.value
                        # Clear data
                        chunk.fill(0)
                        ffts_prev.fill(0)
                        ffts.fill(0)
                        ffts_linspace.fill(0)
                        chunk_cursor = 0
                        sample_counter = 0
                        jma_peak = 0
                        msk_peak = 0

                        # Close previous file
                        if self._file_for_stream is not None:
                            try:
                                self._file_for_stream.close()
                            except Exception as e:
                                logging.error("Error closing file", exc_info=e)
                            self._file_for_stream = None

                        # Check index
                        if 0 <= self.stream_mode_last.value < len(self.files_list):
                            # Load file
                            self._file_for_stream = open(self.files_list[self.stream_mode_last.value]["filepath"], "rb")

                    # Send real-time stream
                    if self.stream_mode.value < 0:
                        # Read and send packet from the queue
                        json_str = self.json_packets_queue.get(block=True)
                        yield "data: " + json_str + "\n\n"

                    # Send file stream
                    else:
                        # Clear queue
                        try:
                            while True:
                                self.json_packets_queue.get_nowait()
                        except Empty:
                            pass

                        # Get packet
                        json_str, chunk_cursor, sample_counter, jma_peak, msk_peak \
                            = self.get_file_stream_data(chunk,
                                                        chunk_cursor,
                                                        ffts_prev,
                                                        ffts,
                                                        sample_counter,
                                                        ffts_linspace,
                                                        pgas,
                                                        jmas,
                                                        msks,
                                                        jma_peak,
                                                        msk_peak)

                        # Sleep if no more data
                        if not json_str or len(json_str) < 0:
                            time.sleep(0.1)

                        # Send data at file sampling rate
                        # TODO: Make it load all at ones
                        else:
                            time.sleep(1 / self.files_list[self.stream_mode.value]["sampling_rate"])
                            yield "data: " + json_str + "\n\n"

            # Make response
            response = Response(stream(), content_type="text/event-stream")

            # Set header to disable cache (not tested if actually do something)
            response.headers["Connection"] = "close"
            response.headers["Max-Age"] = "0"
            response.headers["Expires"] = "0"
            response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, " \
                                                "pre-check=0, post-check=0, max-age=0"
            response.headers["Pragma"] = "no-cache"
            response.headers["Access-Control-Allow-Origin"] = "*"

            # Send stream response
            return response

        # Wrong request
        return Response(status=400)

    def index(self) -> str:
        """
        Renders index.html page
        :return: index.html content
        """
        # Send content of index page
        return render_template("index.html", version=__version__)

    def server_start(self, logging_queue: multiprocessing.Queue):
        """
        Starts Flask web server
        :return:
        """
        # Setup logging for current process
        LoggingHandler.worker_configurer(logging_queue)

        # Read alarm settings for DataProcessor class
        from main import load_json, ALARM_CONFIG_FILE
        alarm_config = load_json(ALARM_CONFIG_FILE)
        self.alarm_enable_low_jma.value = alarm_config["alarm_enable_low_jma"]
        self.alarm_enable_high_jma.value = alarm_config["alarm_enable_high_jma"]
        self.alarm_active_time_s.value = alarm_config["alarm_active_time_s"]

        # Flag to preventing double click on button
        self.is_file_processing = False

        # List of files (as list of dictionaries)
        self.files_list = []

        # Filename and path for downloading file
        self.filename = ""
        self.filepath = ""
        self.delete_after_download = False

        # Initialize Flask app
        self.app = Flask(__name__)

        # Connect main page
        self.app.add_url_rule("/", "index", self.index)

        # Connect data stream
        self.app.add_url_rule("/stream", "stream", self.stream)

        # Connect alarm config GET and POST requests
        self.app.add_url_rule("/alarm_config", "alarm_config", self.get_post_alarm_config, methods=["GET", "POST"])

        # Connect alarm test request
        self.app.add_url_rule("/alarm_test", "alarm_test", self.test_alarm, methods=["POST"])

        # Connect virtual hardware button (cancel alarm / close file + calibrate)
        self.app.add_url_rule("/button", "button", self.virtual_button, methods=["GET"])

        # Files processor
        self.app.add_url_rule("/files", "files", self.files, methods=["GET", "POST"])

        # File downloader
        self.app.add_url_rule("/download", "download", self.download_file, methods=["GET"])

        # Change stream mode
        self.app.add_url_rule("/stream_mode", "stream_mode", self.stream_mode_change, methods=["POST"])

        # Start server
        self.app.run(host=str(self.config["flask_server_host"]),
                     debug=False,
                     threaded=True,
                     port=int(self.config["flask_server_port"]))
