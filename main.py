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
import json
import logging
import multiprocessing
import os
import sys
import time

import DataProcessor
import LoggingHandler
import SerialHandler
import WebHandler

# Version of SeismoHome
__version__ = "2.0.0"

# Files with settings
CONFIG_FILE = "config.json"
ALARM_CONFIG_FILE = "alarm_config.json"


def load_json(file_name: str):
    """
    Loads json from file_name
    :return: json if loaded or empty json if not
    """
    try:
        if os.path.exists(file_name):
            logging.info("Loading " + file_name)
            messages_file = open(file_name, encoding="utf-8")
            json_content = json.load(messages_file)
            messages_file.close()
            if json_content is not None and len(str(json_content)) > 0:
                logging.info("Loaded json: " + str(json_content))
            else:
                json_content = None
                logging.error("Error loading json data from file " + file_name)
        else:
            logging.warning("No " + file_name + " file! Returning empty json")
            return {}
    except Exception as e:
        json_content = None
        logging.error("Error loading JSON file!", exc_info=e)
    if json_content is None:
        json_content = {}
    return json_content


def save_json(file_name: str, content):
    """
    Saves json data to file
    :param file_name: filename to save
    :param content: JSON dictionary
    :return:
    """
    logging.info("Saving to " + file_name)
    file = open(file_name, "w")
    json.dump(content, file, indent=4)
    file.close()


def main() -> None:
    # Multiprocessing fix for Windows
    if sys.platform.startswith("win"):
        multiprocessing.freeze_support()

    # Initialize logging and start listener as process
    logging_handler = LoggingHandler.LoggingHandler()
    logging_handler_process = multiprocessing.Process(target=logging_handler.configure_and_start_listener)
    logging_handler_process.start()
    LoggingHandler.worker_configurer(logging_handler.queue)
    logging.info("LoggingHandler PID: " + str(logging_handler_process.pid))

    # Log software version and GitHub link
    logging.info("Simple Earthquake Detector (SED) version: " + str(__version__))
    logging.info("https://github.com/F33RNI/SeismoHome")

    # Read and parse settings from config file
    config_dict = load_json(CONFIG_FILE)

    # Make them multiprocessing
    config = multiprocessing.Manager().dict(config_dict)

    # Initialize SerialHandler class
    serial_handler = SerialHandler.SerialHandler(config)

    # Start web handler as process
    web_handler = WebHandler.WebHandler(config)
    web_handler_process = multiprocessing.Process(target=web_handler.server_start,
                                                  args=(logging_handler.queue,))
    web_handler_process.start()
    logging.info("WebHandler PID: " + str(web_handler_process.pid))

    # Start data processor as process
    data_processor = DataProcessor.DataProcessor(config, serial_handler, web_handler)
    data_processor_process = multiprocessing.Process(target=data_processor.processor_loop,
                                                     args=(logging_handler.queue,))
    data_processor_process.start()
    logging.info("DataProcessor PID: " + str(web_handler_process.pid))

    # Start serial handler in the main thread and stay in sender_loop()
    serial_handler.start()

    # If we are here, then sender_loop() is exited (interrupt?)
    # Stop reader_loop()
    serial_handler.stop_all()

    # Stop web handler
    logging.warning("Stopping web server")
    web_handler_process.terminate()
    while web_handler_process.is_alive():
        time.sleep(0.1)

    # Stop data processor
    logging.warning("Stopping data processor")
    data_processor.processor_loop_running.Value = False
    data_processor_process.terminate()
    while data_processor_process.is_alive():
        time.sleep(0.1)

    # Exited (I hope)
    logging.warning("I was killed. Bye bye =(")
    logging_handler.queue.put(None)


if __name__ == "__main__":
    main()
