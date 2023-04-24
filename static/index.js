/*
 * Copyright (C) 2023 Fern Lane, SeismoHome earthquake detector project
 * This software is part of Liberty Drones Project aka AMLS (Autonomous Multirotor Landing System)
 *
 * Licensed under the GNU Affero General Public License, Version 3.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.gnu.org/licenses/agpl-3.0.en.html
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// Color map for FFTs
const COLOR_MAP = "magma";

// Default range of Y axis (from -INITIAL_ACCELERATION_RANGE to INITIAL_ACCELERATION_RANGE)
const DEFAULT_ACCELERATION_RANGE = 0.5;

// Data for FFTs
const fftDataX = [];
const fftDataY = [];
const fftDataZ = [];

// Data display size
let dataSize = 1000;

// uPlot for accelerations data
let accelerationsPlot = null;

// Arrays with accelerations data [Timestamps in s], [X], [Y], [Z]
let graphData = [[], [], [], []];

// Flags for auto-resizing in case of large data
let accelerationsPlotYRangeFixed = true;
let accelerationsPlotYRangeFixedPrev = true;

// Initialize HTML Canvas elements for FFT as null for future initialization
let canvasX = null;
let canvasY = null;
let canvasZ = null;
let canvasXContext = null;
let canvasYContext = null;
let canvasZContext = null;

// Alarm config as object (JSON)
let alarmConfig = null;

// Variable for alarm testing 0 -> 1 = LOW, 1 -> 2 = HIGH
let alarmTestState = 0;

// Array of files
let rawFiles = [];

// Connect onLoad function
window.onload = onLoad;

// Calibration states
const CALIBRATION_STATE_OK = 3;

/**
 * Data container for accelerations plot uPlot
 * @type {{series: [{},{label: string, stroke: string},{label: string, stroke: string},{label: string, stroke: string}], scales: {y: {range: (function(*, *, *): [number,number]|[number,number])}}, axes: [{space: number}], pxAlign: boolean}}
 */
const accelerationsPlotOptions = {
	pxAlign: false,
	scales: {
		y: {
			range: (self, dataMin, dataMax) => accelerationsPlotYRangeFixed ?
				[-DEFAULT_ACCELERATION_RANGE, DEFAULT_ACCELERATION_RANGE] :
				uPlot.rangeNum(dataMin, dataMax, 0.1, true)
		}
	},
	axes: [{
		space: 300,
	}],
	series: [{},
		{
			label: "Acceleration X (m/s^2)",
			stroke: "red",
		},
		{
			label: "Acceleration Y (m/s^2)",
			stroke: "green",
		},
		{
			label: "Acceleration Z (m/s^2)",
			stroke: "blue",
		}
	],
};

/**
 * Returns a number whose value is limited to the given range.
 *
 * Example: limit the output of this computation to between 0 and 255
 * (x * 255).clamp(0, 255)
 *
 * @param {Number} min The lower boundary of the output range
 * @param {Number} max The upper boundary of the output range
 * @returns A number in the range [min, max]
 * @type Number
 */
Number.prototype.clamp = function(min, max) {
  return Math.min(Math.max(this, min), max);
};

/**
 * Converts RGB color to #HEX
 * @param {Number} r Red channel 0-255
 * @param {Number} g Green channel 0-255
 * @param {Number} b Blue channel 0-255
 * @returns {string}
 * @type String
 */
function rgbToHex(r, g, b) {
	return "#" + (1 << 24 | r << 16 | g << 8 | b).toString(16).slice(1);
}

/**
 * Writes new data to dataArray and shows it over canvas
 * @param {Array} newData New fft data in JMA scale (0-7)
 * @param {Array} dataArray Array to append to
 * @param {HTMLCanvasElement} canvas FFT canvas
 * @param {CanvasRenderingContext2D} canvasContext canvas.getContext("2d", { willReadFrequently: true })
 */
function updateFFTCanvas(newData, dataArray, canvas, canvasContext) {
	// Append new data to array
	dataArray.push(newData);

	// Check is canvas is available
	if (canvas != null) {
		// Get image from canvas to paste it after resizing and for moving
		let imageData = null;
		if (canvas.width > 0) {
			imageData = canvasContext.getImageData(0, 0, canvas.width, canvas.height);

			// Clear canvas
			canvasContext.clearRect(0, 0, canvas.width, canvas.height);
		}

		// Increase or decrease width to match data size
		if (canvas.width < dataSize) {
			canvas.width++;
		}
		else if (canvas.width > dataSize) {
			canvas.width = dataSize;
		}

		// Fit all FFT bins
		if (canvas.height !== dataArray[0].length)
			canvas.height = dataArray[0].length;

		// Check if we need to scroll canvas
		if (dataArray.length > dataSize) {
			// Paste image
			if (canvas.width > 0)
				canvasContext.putImageData(imageData, dataSize - dataArray.length, 0);

			// Splice data to maximum size
			dataArray.splice(0, dataArray.length - dataSize);
		}

		// We don't need to scroll, so put image back
		else if (canvas.width > 0)
			canvasContext.putImageData(imageData, 0, 0);

		// Fill new data (column)
		for (let row = 0; row < dataArray[0].length; row++) {
			// Convert to 0-1 range
			const dataValue = (dataArray[dataArray.length - 1][row] / 7.).clamp(0., 1.);

			// Get color using colormap
			const pixelColor = evaluate_cmap(dataValue, COLOR_MAP, false);

			// Fill current pixel
			canvasContext.fillStyle = rgbToHex(pixelColor[0], pixelColor[1], pixelColor[2]);
			canvasContext.fillRect(dataArray.length - 1, row, 1, 1);
		}
	}
}

/**
 * Requests alarm settings (thresholds + time) from the server using GET request to /alarm_config url
 */
function requestAlarmConfig() {
	const xhr = new XMLHttpRequest();
	const url = "/alarm_config";
	xhr.open("GET", url, false);
	xhr.onreadystatechange = function () {
		if (xhr.readyState === 4 && xhr.status === 200) {
			alarmConfig = JSON.parse(xhr.responseText);
		}
	};
	xhr.send();
}

/**
 * Sends virtual button click GET request to /button url
 */
function virtualButtonClick() {
	const xhr = new XMLHttpRequest();
	const url = "/button";
	xhr.open("GET", url, false);
	xhr.send();
}

/**
 * Sends alarm test POST request to /alarm_test url
 */
function testAlarm() {
	if (alarmTestState < 2) {
		const xhr = new XMLHttpRequest();
		const url = "/alarm_test";
		xhr.open("POST", url, false);
		xhr.setRequestHeader("Content-Type", "application/json");
		xhr.onreadystatechange = function () {
			if (xhr.readyState === 4 && xhr.status === 200) {
				alarmTestState++;

				// Disable button if we reached high mode
				if (alarmTestState >= 2)
					document.getElementById("button-test").disabled = true;
			}
		};
		const data = JSON.stringify({type: alarmTestState === 0 ? "low" : "high"});
		xhr.send(data);
	}
}

/**
 * Saves alarm settings (thresholds + time) to the server using POST request to /alarm_config url
 */
function saveAlarmConfig() {
	if (alarmConfig != null) {
		const xhr = new XMLHttpRequest();
		const url = "/alarm_config";
		xhr.open("POST", url, false);
		xhr.setRequestHeader("Content-Type", "application/json");
		const data = JSON.stringify(alarmConfig);
		xhr.send(data);
	}
}

/**
 * Requests available data files from server using GET request to /files url
 */
function listFiles() {
	const xhr = new XMLHttpRequest();
	const url = "/files";
	xhr.open("GET", url, false);
	xhr.onreadystatechange = function () {
		if (xhr.readyState === 4 && xhr.status === 200) {
			// Get files list
			rawFiles = JSON.parse(xhr.responseText);

			// Remove options from selector
			clearSelector()

			// Add new files
			const filesSelector = document.getElementById("files-selector");
			for (let i = 0; i < rawFiles.length; i++) {
				const option = document.createElement("option");

				// Calculate file length
				const hoursTotal = Number(rawFiles[i].file_length) / 3600;
				const minutesTotal = Number(rawFiles[i].file_length) / 60;
				const secondsTotal = Number(rawFiles[i].file_length);
				let timeLength = "";
				if (hoursTotal >= 1.)
					timeLength = hoursTotal.toFixed(0) + " hours";
				else if (minutesTotal >= 1.)
					timeLength = minutesTotal.toFixed(0) + " minutes";
				else
					timeLength = secondsTotal + " seconds";

				option.text = rawFiles[i].timestamp_start + " - " + rawFiles[i].timestamp_end + " (" + timeLength + ")";
				option.value = rawFiles[i].filename;
				option.className = "file-selector-option";
				filesSelector.add(option);
			}
		}
	};
	xhr.send();
}

/**
 * Downloads or deletes file based on action by sending POST request to /files url
 * @param {String} action raw / wav / csv / delete
 */
function processFile(action) {
	// Get selected filename and index
	const filename = document.getElementById("files-selector").value;
	const selectedIndex = Number(document.getElementById("files-selector").selectedIndex);

	switch (action) {
		case "raw":
			break;
		case "wav":
			break;
		case "csv":
			break;
		case "delete":
			if (confirm("Are you sure you want to permanently delete file\n" + filename + "?") !== true)
				action = "";
			break;
	}

	// Send request
	if (action.length > 0) {
		const xhr = new XMLHttpRequest();
		const url = "/files";
		xhr.open("POST", url, true);
		xhr.setRequestHeader("Content-Type", "application/json");
		const data = JSON.stringify({"index": selectedIndex, "action": action});
		xhr.onreadystatechange = function () {
			if (xhr.readyState === 4) {
				if (xhr.status === 200) {
					// Redirect to download actual file
					if (action === "raw" || action === "wav" || action === "csv")
						window.location.href = "/download";

					// Show OJ if deleted and refresh files list
					if (action === "delete") {
						alert("Deleted successfully!");
						listFiles();
					}
				}

				// Still processing
				else if (xhr.status === 429)
					alert("The file is still being processed! Be patient!");

				// Error
				else
					alert("Error: " + xhr.status);
			}
		}
		xhr.send(data);
	}
}

/**
 * Removes all options from files selector
 */
function clearSelector() {
	let i, L = document.getElementById("files-selector").options.length - 1;
	for (i = L; i >= 0; i--) {
		document.getElementById("files-selector").remove(i);
	}
}

/**
 * Initializes HTML elements
 */
function onLoad() {
	// Initialize elements from page
	const accelerationsPlotContainer = document.getElementById("accelerations-plot");
	canvasX = document.getElementById("canvas-x");
	canvasX.width = 1;
	canvasX.height = 1;
	canvasXContext = canvasX.getContext("2d", { willReadFrequently: true });
	canvasY = document.getElementById("canvas-y");
	canvasY.width = 1;
	canvasY.height = 1;
	canvasYContext = canvasY.getContext("2d", { willReadFrequently: true });
	canvasZ = document.getElementById("canvas-z");
	canvasZ.width = 1
	canvasZ.height = 1;
	canvasZContext = canvasZ.getContext("2d", { willReadFrequently: true });
	const intensityLowSlider = document.getElementById("intensity-low-threshold");
	const intensityHighSlider =  document.getElementById("intensity-high-threshold");
	const alarmTimeSlider = document.getElementById("alarm-time-active");

	// Initialize accelerations uPlot
	accelerationsPlot = new uPlot(accelerationsPlotOptions, graphData, accelerationsPlotContainer);

	// Connect ResizeObserver to automatically change size of accelerationsPlot
	const accelerationsPlotResizeObserver = new ResizeObserver(() => {
		accelerationsPlot.setSize({
			width: accelerationsPlotContainer.clientWidth,
			height: accelerationsPlotContainer.clientHeight,
		});
	});
	accelerationsPlotResizeObserver.observe(accelerationsPlotContainer);

	// Get data size slider
	const dataSizeSlider = document.getElementById("data-size");

	// Set initial value
	dataSizeSlider.value = dataSize;

	// Show slider value
	document.getElementById("data-size-label").innerText = dataSizeSlider.value;

	// Connect updater
	dataSizeSlider.oninput = function () {
		dataSize = this.value;
		document.getElementById("data-size-label").innerText = this.value;
	}

	// Request alarm settings
	requestAlarmConfig();

	// Set initial config to sliders and labels
	intensityLowSlider.value = (alarmConfig.alarm_enable_low_jma * 10).toFixed(0);
	intensityHighSlider.value = (alarmConfig.alarm_enable_high_jma * 10).toFixed(0);
	alarmTimeSlider.value = alarmConfig.alarm_active_time_s;
	document.getElementById("intensity-low-threshold-label").innerText
			= alarmConfig.alarm_enable_low_jma.toString();
	document.getElementById("intensity-high-threshold-label").innerText
			= alarmConfig.alarm_enable_high_jma.toString();
	document.getElementById("alarm-time-active-label").innerText
			= alarmConfig.alarm_active_time_s.toString();

	// Connect alarm sliders
	intensityLowSlider.oninput = function () {
		alarmConfig.alarm_enable_low_jma = Number(this.value) / 10.;
		document.getElementById("intensity-low-threshold-label").innerText
			= alarmConfig.alarm_enable_low_jma.toString();
		saveAlarmConfig();
	}
	intensityHighSlider.oninput = function () {
		alarmConfig.alarm_enable_high_jma = Number(this.value) / 10.;
		document.getElementById("intensity-high-threshold-label").innerText
			= alarmConfig.alarm_enable_high_jma.toString();
		saveAlarmConfig();
	}
	alarmTimeSlider.oninput = function () {
		alarmConfig.alarm_active_time_s = Number(this.value);
		document.getElementById("alarm-time-active-label").innerText
			= alarmConfig.alarm_active_time_s.toString();
		saveAlarmConfig();
	}

	// Refresh available files
	listFiles();
}

if (!!window.EventSource) {
	const source = new EventSource("/stream");
	source.onmessage = function (e) {
		// Ignore if page elements are not initialized
		if (accelerationsPlot === null || canvasX === null || canvasY === null || canvasZ === null)
			return;

		// Parse incoming data as JSON
		const jsonData = JSON.parse(e.data);

		// Make copy of dataSize to make sure it didn't change
		const dataSize_ = dataSize;

		// Calculate absolute minimum and maximum across all acceleration data we have
		const absoluteData = graphData[1].concat(graphData[2], graphData[3]);
		const accelerationMin = Math.min(...absoluteData);
		const accelerationMax = Math.max(...absoluteData);

		// Enable auto-range if data is above -DEFAULT_ACCELERATION_RANGE; DEFAULT_ACCELERATION_RANGE
		accelerationsPlotYRangeFixed = !(accelerationMin < -DEFAULT_ACCELERATION_RANGE
			|| accelerationMax > DEFAULT_ACCELERATION_RANGE);

		// Redraw accelerations plot if accelerationsPlotYRangeFixed changed
		if (accelerationsPlotYRangeFixedPrev !== accelerationsPlotYRangeFixed) {
			accelerationsPlotYRangeFixedPrev = accelerationsPlotYRangeFixed;
			accelerationsPlot.redraw();
		}

		// Append new data
		graphData[0].push(jsonData.timestamp / 1000.);
		graphData[1].push(jsonData.accelerations[0]);
		graphData[2].push(jsonData.accelerations[1]);
		graphData[3].push(jsonData.accelerations[2]);

		// Splice data (Scroll plot)
		graphData[0].splice(0, graphData[0].length - dataSize_);
		graphData[1].splice(0, graphData[1].length - dataSize_);
		graphData[2].splice(0, graphData[2].length - dataSize_);
		graphData[3].splice(0, graphData[3].length - dataSize_);

		// Update plot
		accelerationsPlot.setData(graphData);

		// Update FFTs
		updateFFTCanvas(jsonData.ffts[0].reverse(), fftDataX, canvasX, canvasXContext);
		updateFFTCanvas(jsonData.ffts[1].reverse(), fftDataY, canvasY, canvasYContext);
		updateFFTCanvas(jsonData.ffts[2].reverse(), fftDataZ, canvasZ, canvasZContext);

		// Set FFTs range text
		document.getElementById("fft-text-top-x").innerText
			= jsonData.fft_range_to.toFixed(0);
		document.getElementById("fft-text-bottom-x").innerText
			= jsonData.fft_range_from.toFixed(0);
		document.getElementById("fft-text-top-y").innerText
			= jsonData.fft_range_to.toFixed(0);
		document.getElementById("fft-text-bottom-y").innerText
			= jsonData.fft_range_from.toFixed(0);
		document.getElementById("fft-text-top-z").innerText
			= jsonData.fft_range_to.toFixed(0);
		document.getElementById("fft-text-bottom-z").innerText
			= jsonData.fft_range_from.toFixed(0);

		// Update text data on page
		document.getElementById("intensity-jma").innerText = jsonData.intensity_jma.toFixed(1);
		document.getElementById("intensity-msk").innerText = jsonData.intensity_msk.toFixed(1);
		document.getElementById("intensity-jma-max").innerText = jsonData.intensity_jma_max.toFixed(1);
		document.getElementById("intensity-msk-max").innerText = jsonData.intensity_msk_max.toFixed(1);
		document.getElementById("alarm-state").innerText = jsonData.alarm_state;
		document.getElementById("battery-voltage").innerText =
			(jsonData.battery_voltage_mv / 1000.).toFixed(2);
		document.getElementById("battery-state").innerText = jsonData.battery_state;
		document.getElementById("temperature").innerText = jsonData.temperature.toFixed(0);

		// Set alarm test state (in case we did not get right response from test request)
		if (jsonData.alarm_state === "Test low")
			alarmTestState = 1;
		else if (jsonData.alarm_state === "Test high")
			alarmTestState = 2;
		else
			alarmTestState = 0;

		// Set virtual button text and make it enabled only if alarm or calibrated
		if (jsonData.alarm_state !== "Off") {
			document.getElementById("button-hardware").innerText = "Cancel alarm";
			document.getElementById("button-hardware").disabled = false;
		}
		else if (jsonData.calibration_state === CALIBRATION_STATE_OK) {
			document.getElementById("button-hardware").innerText = "Close file & calibrate";
			document.getElementById("button-hardware").disabled = false;
		}
		else {
			document.getElementById("button-hardware").innerText = "Close file & calibrate";
			document.getElementById("button-hardware").disabled = true;
		}

		// Disable test alarm if it is real alarm, or we reached high test state
		if ((jsonData.alarm_state !== "Off" && !jsonData.alarm_state.includes("Test")) || alarmTestState >= 2) {
			document.getElementById("button-test").disabled = true;
			document.getElementById("button-test").innerText = "Test alarm";
		}

		// Test button enabled
		else {
			document.getElementById("button-test").disabled = false;
			document.getElementById("button-test").innerText = alarmTestState === 0 ?
				"Test low alarm" : "Test high alarm";
		}
	}
}
