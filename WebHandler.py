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
import multiprocessing
import os.path
import struct
import time
from queue import Empty

from flask import Flask, Response, render_template, request, stream_with_context

import DataProcessor

FILE_CHUNK_SIZE = 2048


class WebHandler:
    def __init__(self, config: dict):
        self.config = config

        self.app = None

        # Flag to preventing double click on button
        self.is_file_processing = False

        # List of files (as list of dictionaries)
        self.files_list = []

        # Filename and path for downloading file
        self.filename = ""
        self.filepath = ""
        self.delete_after_download = False

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

                    # Extract requested file and action
                    file = self.files_list[int(request.json["index"])]
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
                            # Get timestamp
                            timestamp_seconds \
                                = int(time.mktime(datetime.datetime.strptime(filename_parts[0] + "__" +
                                                                             filename_parts[1],
                                                                             "%Y_%m_%d__%H_%M_%S").timetuple()))

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

                # Start data stream
                while True:
                    json_str = self.json_packets_queue.get(block=True)
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
        return render_template("index.html")

    def server_start(self):
        """
        Starts Flask web server
        :return:
        """
        # Setup logging for current process
        from main import logging_setup
        logging_setup()

        # Read alarm settings for DataProcessor class
        from main import load_json, ALARM_CONFIG_FILE
        alarm_config = load_json(ALARM_CONFIG_FILE)
        self.alarm_enable_low_jma.value = alarm_config["alarm_enable_low_jma"]
        self.alarm_enable_high_jma.value = alarm_config["alarm_enable_high_jma"]
        self.alarm_active_time_s.value = alarm_config["alarm_active_time_s"]

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

        # Start server
        self.app.run(host=str(self.config["flask_server_host"]),
                     debug=False,
                     threaded=True,
                     port=int(self.config["flask_server_port"]))
