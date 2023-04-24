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

import logging
import os
import platform
import re
import shutil
import subprocess
import sys

# Version of Builder (NOT APP)
__version__ = "1.0.0"

########################
#     BUILD CONFIG     #
########################
# Remove all files before building?
import time

CLEAN = True

# Name of main script
MAIN_SCRIPT = "main.py"

# Name of project (output directory and file) (using --name argument)
OUTPUT_NAME = "SeismoHome"

# Files and folders to include (copy) into final build directory after build
COPY_FILES = ["icon.png",
              "README.md",
              "LICENSE",
              "templates",
              "static",
              "config.json",
              "alarm_config.json"]

# Files and folders to delete from final build directory after build
DELETE_FILES = []

# *.py files (scripts) to exclude from build
EXCLUDE_SCRIPTS_FROM_BUILD = ["Builder.py"]

# Modules to exclude (using --exclude-module argument)
EXCLUDE_MODULES = ["_bootlocale"]

# Collect all data from the specified packages or modules (using --collect-data argument)
COLLECT_DATA_FROM = []

# Collect all submodules, data files and binaries from the specified packages or modules (using --collect-all argument)
COLLECT_ALL_FROM = []

# Copy metadata for the specified packages (using --copy-metadata argument)
COPY_METADATA_FROM = []

# Packages to import (using --hidden-import argument)
HIDDEN_IMPORTS = []

# Icon for app. Leave emtpy if you don't have one
APP_ICON = "icon.ico"

# Set True for apps with GUI or False for cli apps
WINDOWED = False

# Set True to enable console GUI after opening executable
CONSOLE = True

########################

# Text to add to the spec file to set build directory
SPEC_FILE_HEADER = "import PyInstaller.config\n" \
                   "PyInstaller.config.CONF[\'workpath\'] = \'./build\'\n"


def makespec(build_name: str):
    """
    Generates temp .spec file using pyi-makespec command
    :param build_name: Name of build (directory and executable file)
    :return:
    """
    pyi_command = ["pyi-makespec"]
    scripts = []

    # Remove previously generated spec files
    if os.path.exists(OUTPUT_NAME + ".spec"):
        logging.warning("Deleting " + OUTPUT_NAME + ".spec")
        os.remove(OUTPUT_NAME + ".spec")
    if os.path.exists(MAIN_SCRIPT + ".spec"):
        logging.warning("Deleting " + MAIN_SCRIPT + ".spec")
        os.remove(MAIN_SCRIPT + ".spec")
    if os.path.exists(build_name + ".spec"):
        logging.warning("Deleting " + build_name + ".spec")
        os.remove(build_name + ".spec")

    # Add all .py files (excluding main script)
    for file in os.listdir("./"):
        if file.endswith(".py") and str(file) != MAIN_SCRIPT \
                and str(file) not in EXCLUDE_SCRIPTS_FROM_BUILD:
            scripts.append(str(file))
    logging.info("Found " + str(len(scripts)) + " .py scripts to include into build: " + " ".join(scripts))

    # Add modules to exclude
    for exclude_module in EXCLUDE_MODULES:
        pyi_command.append("--exclude-module")
        pyi_command.append(exclude_module)

    # Windowed?
    if WINDOWED:
        pyi_command.append("--windowed")

    # Console?
    if CONSOLE:
        pyi_command.append("--console")
    else:
        pyi_command.append("--noconsole")

    # Add icon
    if APP_ICON and os.path.exists(APP_ICON):
        pyi_command.append("--icon=" + APP_ICON)

    # Add name
    pyi_command.append("--name")
    pyi_command.append(build_name)

    # Add main script
    pyi_command.append(MAIN_SCRIPT)

    # Append all scripts
    for script in scripts:
        pyi_command.append(script)

    # Execute pyi_command
    logging.info("Starting pyi-makespec with command: " + " ".join(pyi_command))
    pyi_process = subprocess.run(pyi_command, text=True)
    time.sleep(1)
    logging.info("pyi-makespec finished with code " + str(pyi_process.returncode))


def main():
    # Initialize logging
    log_formatter = logging.Formatter("[%(asctime)s] [%(levelname)-8s] %(message)s",
                                      datefmt="%Y-%m-%d %H:%M:%S")
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(log_formatter)
    root_logger = logging.getLogger()
    root_logger.addHandler(console_handler)
    root_logger.setLevel(logging.INFO)
    logging.info("Starting builder version " + str(__version__))

    # Check some files
    if not os.path.exists(MAIN_SCRIPT):
        logging.error(MAIN_SCRIPT + " doesn't exist")
        exit(-1)
        return
    for include_file in COPY_FILES:
        if not os.path.exists(include_file):
            logging.error(include_file + " doesn't exist")
            exit(-1)
            return
    if APP_ICON and not os.path.exists(APP_ICON):
        logging.error(APP_ICON + " doesn't exist")
        exit(-1)
        return

    # Get version of main script
    version_regex = re.search(r"^__version__ = ['\"]([^'\"]*)['\"]", open(MAIN_SCRIPT, "rt").read(), re.M)
    if version_regex:
        main_script_version = version_regex.group(1)
    else:
        logging.error("Unable to find version string in %s." % MAIN_SCRIPT)
        exit(-1)
        return
    logging.info(MAIN_SCRIPT + " version: " + main_script_version)

    if CLEAN:
        # Warn about CLEAN mode
        logging.warning("Performing clean build")

        # Remove dist folder is exists
        if "dist" in os.listdir("./"):
            logging.warning("Deleting dist directory")
            shutil.rmtree("dist", ignore_errors=False)
            logging.info("dist directory deleted")

        # Remove build folder is exists
        if "build" in os.listdir("./"):
            logging.warning("Deleting build directory")
            shutil.rmtree("build", ignore_errors=True)
            logging.info("build directory deleted")

    # Generate final build name
    build_name = OUTPUT_NAME + "-" + main_script_version + "-" + str(platform.system()) + "-" + str(platform.machine())
    logging.info("Target build name: " + build_name)

    # Generate temporary spec file
    makespec(build_name)

    # Check spec file
    if not os.path.exists(build_name + ".spec"):
        logging.error("No " + build_name + ".spec" + " file!")
        exit(-1)
        return

    # Make pyinstaller command
    pyi_command = ["pyinstaller"]
    if CLEAN:
        pyi_command.append("--clean")
    pyi_command.append(build_name + ".spec")

    # Execute pyinstaller
    logging.info("Starting pyinstaller with command: " + " ".join(pyi_command))
    pyi_process = subprocess.run(pyi_command, text=True)
    time.sleep(1)
    logging.info("pyinstaller finished with code " + str(pyi_process.returncode))

    # Check output directory
    if not os.path.exists(os.path.join("dist", build_name)):
        logging.error("No " + os.path.join("dist", build_name) + "directory!")
        exit(-1)
        return

    # Copy COPY_FILES files to final directory
    for file in COPY_FILES:
        try:
            logging.info("Copying " + file)
            if os.path.isfile(file):
                shutil.copy(file, os.path.join("dist", build_name, file))
            elif os.path.isdir(file):
                shutil.copytree(file, os.path.join("dist", build_name, file))
            logging.info(file + " copied to " + os.path.join("dist", build_name, file) + " directory")
        except Exception as e:
            logging.error("Error copying file or directory: " + file, exc_info=e)
    time.sleep(1)

    # Delete DELETE_FILES files from final directory
    for file in DELETE_FILES:
        try:
            file = os.path.join("dist", build_name, file)
            logging.info("Deleting " + file)
            if os.path.isfile(file):
                os.remove(file)
            elif os.path.isdir(file):
                shutil.rmtree(file)
            logging.info(file + " deleted")
        except Exception as e:
            logging.error("Error deleting file or directory: " + file, exc_info=e)
    time.sleep(1)

    # Remove temporary spec files
    if os.path.exists(OUTPUT_NAME + ".spec"):
        logging.info("Deleting " + OUTPUT_NAME + ".spec")
        os.remove(OUTPUT_NAME + ".spec")
    if os.path.exists(MAIN_SCRIPT + ".spec"):
        logging.info("Deleting " + MAIN_SCRIPT + ".spec")
        os.remove(MAIN_SCRIPT + ".spec")
    if os.path.exists(build_name + ".spec"):
        logging.info("Deleting " + build_name + ".spec")
        os.remove(build_name + ".spec")

    # Remove build directory is exists
    if CLEAN:
        if "build" in os.listdir("./"):
            logging.info("Deleting build directory")
            shutil.rmtree("build", ignore_errors=True)
            logging.info("build directory deleted")

    # Done?
    logging.info("Done! Build is located in the directory: " + os.path.join("dist", build_name))


if __name__ == "__main__":
    main()
