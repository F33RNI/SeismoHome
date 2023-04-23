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

import math

FILTER_TYPE_LOWPASS = 0
FILTER_TYPE_HIGHPASS = 1


class Filter:
    def __init__(self, pass_type: int, sample_rate: int, cutoff_frequency: float, resonance=math.sqrt(2), order=1):
        """
        Multi-order butterworth filter
        :param pass_type: FILTER_TYPE_LOWPASS or FILTER_TYPE_HIGHPASS
        :param sample_rate: Sampling rate of source signal
        :param cutoff_frequency: Cutoff frequency of filter
        :param resonance: Amount of resonance. Values from sqrt(2) (no resonance) to ~0.1 (high resonance)
        :param order: Through how many consecutive filters the signal will pass
        """
        # Initialize filters
        # Each order = +1 filter
        self.filters = []
        for order_n in range(order):
            self.filters.append(_ButterworthFilter(pass_type, sample_rate, cutoff_frequency, resonance))

    def filter(self, input_value: float):
        """
        Filters input_value trough each stage (order)
        :param input_value: value to filter
        :return: filtered value
        """
        for filter_ in self.filters:
            input_value = filter_.filter(input_value)
        return input_value


class _ButterworthFilter:
    def __init__(self, pass_type: int, sample_rate: int, cutoff_frequency: float, resonance=math.sqrt(2)):
        """
        Python version of butterworth filter from https://github.com/filoe/cscore
        :param pass_type: FILTER_TYPE_LOWPASS or FILTER_TYPE_HIGHPASS
        :param sample_rate: Sampling rate of source signal
        :param cutoff_frequency: Cutoff frequency of filter
        :param resonance: Amount of resonance. Values from sqrt(2) (no resonance) to ~0.1 (high resonance)
        """
        self.input_history = [0., 0.]
        self.output_history = [0., 0., 0.]

        # Initialize constants for low-pass type
        if pass_type == FILTER_TYPE_LOWPASS:
            c = 1.0 / math.tan(math.pi * cutoff_frequency / sample_rate)
            self.a1 = 1.0 / (1.0 + resonance * c + c * c)
            self.a2 = 2 * self.a1
            self.a3 = self.a1
            self.b1 = 2.0 * (1.0 - c * c) * self.a1
            self.b2 = (1.0 - resonance * c + c * c) * self.a1

        # Initialize constants for high-pass type
        elif pass_type == FILTER_TYPE_HIGHPASS:
            c = math.tan(math.pi * cutoff_frequency / sample_rate)
            self.a1 = 1.0 / (1.0 + resonance * c + c * c)
            self.a2 = -2 * self.a1
            self.a3 = self.a1
            self.b1 = 2.0 * (c * c - 1.0) * self.a1
            self.b2 = (1.0 - resonance * c + c * c) * self.a1

        else:
            raise ValueError("Wrong pass_type! Must be 0 (Low-pass) or 1 (High-pass)!")

    def filter(self, input_value: float):
        """
        Filters input_value
        :param input_value: value to filter
        :return: filtered value
        """
        new_output = self.a1 * input_value \
                     + self.a2 * self.input_history[0] \
                     + self.a3 * self.input_history[1] \
                     - self.b1 * self.output_history[0] \
                     - self.b2 * self.output_history[1]

        self.input_history[1] = self.input_history[0]
        self.input_history[0] = input_value

        self.output_history[2] = self.output_history[1]
        self.output_history[1] = self.output_history[0]
        self.output_history[0] = new_output

        return self.output_history[0]
