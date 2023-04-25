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

#include <Arduino.h>

#include <Wire.h>
#include <TimerOne.h>
#include <TimerTwo.h>

// User defined literal for unsigned char (uint8_t)
inline constexpr unsigned char operator "" _uint8(unsigned long long arg) noexcept {
  return static_cast<unsigned char> (arg);
}

/**
 * ALARM SETTINGS
*/
// Period of alarm buzzer sound 2273us = 0.004545s = ~440Hz
#define ALARM_SOUND_PERIOD    2273U

// Maximum alarm PWM value (0 - 255) to not overload the buzzer
#define ALARM_PWM_MAX         230.f

// LFO effect decrement of pwm value for each alarm cycle (230 / 3) * 2273us = ~0.37s =~2.69Hz (~Mexico alarm)
#define ALARM_LFO_CYCLE_DECR  1.402f

// On time (in cycles) for alarm in low mode. 132cycles * 2273us = ~0.227s
#define ALARM_LOW_ON_CYCLES   132U

// Off time (in cycles) for alarm in low mode. To make total ~1s: (1000000us / 2273us) - 132cycles
#define ALARM_LOW_OFF_CYCLES  308U

/**
 * BATTERY VOLTAGE SETTINGS
*/
// Stop charging battery if voltage is above this threshold
#define VBAT_STOP_CHARGE_MV   4000.f

// Start charging battery if voltage is below this threshold
#define VBAT_START_CHARGE_MV  3800.f

// Turn on low battery LED if voltage is below this threshold
#define VBAT_LOW_VOLTAGE_L_MV 3600.f

// Turn off low battery LED if voltage is above this threshold
#define VBAT_LOW_VOLTAGE_U_MV 3650.f

// Actual voltage at 1.1V reference (MEASURE IT BEFORE FINAL USE AT VREF PIN)
#define VREF_1_1_ACTUAL_MV    1086.f

// Resistor between battery and analog pin (in Ohms)
#define VBAT_RESISTOR_BAT     50900.f

// Resistor between ground and analog pin (in Ohms)
#define VBAT_RESISTOR_GND     14910.f

// On and off time (in ms) for blinking LED (will blink if no charger is connected)
#define ON_BATTERY_LED_ON     100U
#define ON_BATTERY_LED_OFF    900U

// Filter (0-1) for battery voltage the higher the slower
#define VOLTAGE_FILTER_K      0.9995f

/**
 * COMMUNICATION SETTINGS
*/
// Consider serial connection is lost if there is no data in that time (in ms)
#define RX_WATCHDOG_TIMEOUT   1000U

// Speed of serial port (must be equal to Python program settings)
#define SERIAL_BAUD_RATE      57600U

// Starting, ending and escaping bytes in packet
#define PACKET_SOH            0x01_uint8
#define PACKET_EOT            0x04_uint8
#define PACKET_ESC            0x1B_uint8

/**
 * ACCELEROMETER SETTINGS
*/
// Low-pass filter config (0 - 260Hz, 1 - 184Hz, 2 - 94Hz, 3 - 44Hz, 4 - 21Hz, 5 - 10Hz, 6 - 5Hz)
#define IMU_LOW_PASS_FILTER   4_uint8

// Period of IMU reading (sampling rate) 25000us = 0.025s = 40Hz. Must be >= band-pass filter frequency x2
#define IMU_READ_PERIOD       25000U

/**
 * BUTTON SETTINGS
*/
// Time to wait (in ms) after button was pressed before reading new state
#define BUTTON_DEBOUNCING_MS   200U

/**
 * CALIBRATION STATUS LED SETTINGS
*/
// Blink each 100ms to show that we are in calibration mode
#define CALIBRATION_LED_BLINK 100U

/**
 * PHYSICAL PIN AND I2C ADDRESS SETTINGS
*/
// Analog (ADC) pin of battery voltage sensor
#define PIN_ADC_VBAT          A0

// Digital pin of button
#define PIN_BTN               8_uint8

// Digital pin of buzzer
#define PIN_BUZZER            3_uint8

// Digital pin of charger detect signal
#define PIN_CHARGER_DETECT    10_uint8

// Digital pin of disable charging signal
#define PIN_CHARGING_STOP     7_uint8

// Digital pins of LEDs
#define PIN_LED_CHARGER       6_uint8
#define PIN_LED_CALIBRATION   5_uint8
#define PIN_LED_IMU_READING   2_uint8
#define PIN_LED_LOW_BAT       A3
#define PIN_LED_ALARM         4_uint8

// MPU-6050 I2C address
#define IMU_ADDRESS           0x68_uint8

/**
 * SOME DEFINITIONS FOR BETTER CODE READABILITY
*/
// Alarm states defines
#define ALARM_STATE_OFF       0_uint8
#define ALARM_STATE_LOW       1_uint8
#define ALARM_STATE_HIGH      2_uint8

// Power states defines
#define POWER_STATE_IDLE      0_uint8
#define POWER_STATE_CHARGING  1_uint8
#define POWER_STATE_ON_BAT    2_uint8

// Calibration states defines
#define CALIBRATION_STATE_NO          0_uint8
#define CALIBRATION_STATE_IN_PROCESS  1_uint8
#define CALIBRATION_STATE_OK          2_uint8

// IMU variables
int16_t temperature;
float temperature_c;
uint8_t temperature_byte;
int16_t acc_x, acc_y, acc_z;
boolean led_imu_state;

// Alarm variables
volatile uint8_t alarm_state = ALARM_STATE_OFF;
float alarm_pwm;
uint16_t alarm_cycles_counter;
boolean alarm_low_state;

// Voltage measurement variables
float battery_voltage, battery_voltage_filtered, vbat_pin_voltage;
uint16_t battery_voltage_mv_int;
boolean battery_low_flag;

// Charging variables
volatile uint8_t power_state = POWER_STATE_ON_BAT;
boolean power_state_led_state;
uint64_t power_state_led_timer;
boolean charger_connected;

// Calibration LED variables
uint8_t calibration_state;
boolean calibration_led_state;
uint64_t calibration_led_timer;

// Button variables
boolean button_pressed, button_flag, clear_button_flag;
uint64_t button_timer;

// Communication variables
boolean stream_enabled;
uint8_t rx_buffer_cursor, rx_buffer_start, rx_packet_cursor, check_byte;
uint64_t rx_watchdog_timer;

// Transmit packet (13 bytes including checksum, big-ending)
// acc_x acc_y acc_z battery_voltage_mv_int power_state battery_low temperature button_flag checksum
//  0 1   2 3   4 5          6 7                 8          9            10          11        12
// Buffer size = packet size because we DON'T NEED to store starting, ending and escaping bytes
uint8_t tx_buffer[13];

// Receive packet (4 bytes, big-ending)
// alarm_state stream_enabled clear_button_flag calibration_state
//      0            1                2                 3
uint8_t rx_packet[4];

// Buffer size = 1 (SOH) + packet size * 2 + 2 (checksum and it's possible escaping byte) + 1 (EOT)
uint8_t rx_buffer[12];

// Functions defines
void calibration(void);
void serial_read(void);
void send_data(void);
void button_read(void);
void charging_manager(void);
void vbat_measure(void);
void buzzer_timer_callback(void);
void imu_read_interrupt_callback(void);
void imu_read(void);
void imu_setup(void);
void led_charger(boolean state);
void led_calibration(boolean state);
void led_imu_reading(boolean state);
void led_low_bat(boolean state);
void led_alarm(boolean state);
void set_charging(boolean state);
void check_charging(void);
void check_button(void);

void setup(void) {
  // Initialize all pins
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_CHARGER_DETECT, INPUT);
  pinMode(PIN_CHARGING_STOP, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_CHARGER, OUTPUT);
  pinMode(PIN_LED_CALIBRATION, OUTPUT);
  pinMode(PIN_LED_IMU_READING, OUTPUT);
  pinMode(PIN_LED_LOW_BAT, OUTPUT);
  pinMode(PIN_LED_ALARM, OUTPUT);

  // Enable all LEDs to test them
  led_charger(true);
  led_calibration(true);
  led_imu_reading(true);
  led_low_bat(true);
  led_alarm(true);

  // Turn off charging
  set_charging(false);

  // Initialize I2C with 400KHz clock
  Wire.begin();
  delay(100);
  
  // Initialize IMU
  imu_setup();
  delay(100);

  // Get first data from IMU
  imu_read();
  delay(100);

  // Setup serial port
  Serial.begin(SERIAL_BAUD_RATE);
  delay(100);

  // Enable internal reference and wait for settle
  analogReference(INTERNAL);
  analogRead(A0);
  delay(100);

  // Initialize buzzer using TimerTwo library
  Timer2.init(ALARM_SOUND_PERIOD, buzzer_timer_callback);
  Timer2.start();

  // Initialize IMU reading interrupt using TimerOne library
  Timer1.initialize(IMU_READ_PERIOD);
  Timer1.attachInterrupt(imu_read_interrupt_callback);
  Timer1.start();

  // Wait 1s to settle and to test all LEDs
  while (millis() < 1000);;

  // Turn all LEDs off
  led_charger(false);
  led_calibration(false);
  led_imu_reading(false);
  led_low_bat(false);
  led_alarm(false);
}

void loop(void) {
  // Measure battery voltage, set power state, enable or disable charging and control power LED
  charging_manager();

  // Read button state and set button_flag
  button_read();

  // Parse data from serial
  serial_read();
  
  // Show calibration state
  calibration();
}

/**
 * Controls calibration LED
*/
void calibration(void) {
  // Blink with LED indicating that we are in calibration mode
  if (calibration_state == CALIBRATION_STATE_IN_PROCESS) {
    // Change LED state every CALIBRATION_LED_BLINK ms
    if (millis() - calibration_led_timer >= CALIBRATION_LED_BLINK) {
      calibration_led_timer = millis();
      calibration_led_state = !calibration_led_state;
      led_calibration(calibration_led_state);
    }
  }

  // Turn LED on constantly if calibration is OK
  else if (calibration_state == CALIBRATION_STATE_OK)
    led_calibration(true);

  // Turn LED off if not charging
  else
    led_calibration(false);
}

/**
 * Reads and parses data from serial port
*/
void serial_read(void) {
  // Read all available bytes
  while (Serial.available())
  {
    // Read current byte
    rx_buffer[rx_buffer_cursor] = Serial.read();

    // Consider packet end if we have at least 2 bytes and current byte is EOT and previous byte was not ESC
    if (rx_buffer_cursor > 0 
        && rx_buffer[rx_buffer_cursor] == PACKET_EOT && rx_buffer[rx_buffer_cursor - 1] != PACKET_ESC) {
      
      // Parse packet only if we have at least RX_PACKET_SIZE bytes + 1 EOT byte
      if (rx_buffer_cursor > (uint8_t) sizeof(rx_packet)) {

        // Try to find packet start by searching for SOH without escaping byte
        rx_buffer_start = 0;
        for (uint8_t i = 0; i <= rx_buffer_cursor; i++) {
          // SOH at the first byte, so packet starts from 1 (because we don't need to parse SOH byte)
          if (i == 0 && rx_buffer[i] == PACKET_SOH) {
            rx_buffer_start = 1;
            break;
          }

          // SOH without previous ESC, so packet starts from i + 1 (because we don't need to parse SOH byte)
          if (i > 0 && rx_buffer[i - 1] != PACKET_ESC && rx_buffer[i] == PACKET_SOH) {
            rx_buffer_start = i + 1;
            break;
          }
        }

        // Check if we successfully found packet start
        if (rx_buffer_start > 0 && rx_buffer_start + (uint8_t) sizeof(rx_packet) <= (uint8_t) sizeof(rx_buffer)) {
          
          // Try to extract packet (without escaping bytes) and calculate checksum at the same time
          // Stop at rx_buffer_cursor - 2 (Last two bytes are checksum byte and EOT)
          rx_packet_cursor = 0;
          check_byte = 0;
          for (uint8_t i = rx_buffer_start; i <= rx_buffer_cursor - 2; i++) {
            // Stop if we have all packet bytes
            if (rx_packet_cursor >= sizeof(rx_packet))
              break;

            // We found escape byte
            if (rx_buffer[i] == PACKET_ESC) {
              // Stop if we have escape byte before checksum byte
              if (i + 1 == rx_buffer_cursor - 1)
                break;

              // Append next byte to packet
              rx_packet[rx_packet_cursor] = rx_buffer[i + 1];
              rx_packet_cursor++;

              // Calculate checksum using only next byte 
              check_byte ^= rx_buffer[i + 1];

              // Skip one cycle
              i++;

              // Skip normal calculation
              continue;
            }

            // Append data byte
            rx_packet[rx_packet_cursor] = rx_buffer[i];
            rx_packet_cursor++;

            // Calculate checksum normally (no previous escape byte)
            check_byte ^= rx_buffer[i];
          }

          // Check if checksum match
          if (check_byte == rx_buffer[rx_buffer_cursor - 1]) {

            // Read alarm_state
            alarm_state = rx_packet[0];

            // Read stream_enabled flag
            stream_enabled = rx_packet[1];

            // Read clear_button_flag
            clear_button_flag = rx_packet[2];

            // Read calibration_state
            calibration_state = rx_packet[3];

            // Reset watchdog timer
            rx_watchdog_timer = millis();
          }
        }
      }

      // Reset bytes counter
      rx_buffer_cursor = 0;
    }
    else {
      // Increment bytes counter
      rx_buffer_cursor++;

      // Reset buffer position on overflow
      if (rx_buffer_cursor >= sizeof(rx_buffer))
        rx_buffer_cursor = 0;
    }
  }

  // Timeout?
  if (millis() - rx_watchdog_timer >= RX_WATCHDOG_TIMEOUT) {
    // Close stream
    stream_enabled = false;

    // Reset calibration LED
    calibration_state = CALIBRATION_STATE_NO;

    // Clear reset button flag
    clear_button_flag = false;

    // Reset alarm if button was pressed
    if (button_flag) {
      alarm_state = ALARM_STATE_OFF;
      button_flag = false;
    }
  }
}

/**
 * Combines data into tx_packet and sends it over serial port
*/
void send_data(void) {
  // X - acceleration as 2 bytes big-ending
  tx_buffer[0] = acc_x >> 8;
  tx_buffer[1] = acc_x;

  // Y - acceleration as 2 bytes big-ending
  tx_buffer[2] = acc_y >> 8;
  tx_buffer[3] = acc_y;

  // Z - acceleration as 2 bytes big-ending
  tx_buffer[4] = acc_z >> 8;
  tx_buffer[5] = acc_z;

  // Battery voltage as 2 bytes big-ending
  tx_buffer[6] = battery_voltage_mv_int >> 8;
  tx_buffer[7] = battery_voltage_mv_int;

  // Power state (0 / 1 / 2)
  tx_buffer[8] = power_state;

  // Low battery flag (0 / 1)
  tx_buffer[9] = battery_low_flag ? 1_uint8 : 0_uint8;

  // Temperature (in celsius)
  tx_buffer[10] = temperature_byte;

  // Button pressed flag (0 / 1)
  tx_buffer[11] = button_flag ? 1_uint8 : 0_uint8;

  // Calculate checksum
  tx_buffer[12] = 0;
  for (uint8_t i = 0; i <= 11; i++)
    tx_buffer[12] ^= tx_buffer[i];

  // Write SOH (start-of-header) byte as packet start
  Serial.write(PACKET_SOH);

  // Write data to serial port and replace SOH / EOT / ESC bytes with ESC + SOH / EOT / ESC (escaping them)
  for (uint8_t i = 0; i < sizeof(tx_buffer); i++) {
    if (tx_buffer[i] == PACKET_SOH || tx_buffer[i] == PACKET_EOT || tx_buffer[i] == PACKET_ESC)
      Serial.write(PACKET_ESC);
    Serial.write(tx_buffer[i]);
  }
  
  // Write EOT (end-of-transmission) byte as packet end
  Serial.write(PACKET_EOT);
}

/**
 * Reads button state and sets button_flag
*/
void button_read(void) {
  // Read button state
  check_button();

  // Set flag and start debouncing timer if button is pressed, no flag and debouncing time has passed
  if (button_pressed && !button_flag && !clear_button_flag && millis() - button_timer >= BUTTON_DEBOUNCING_MS) {
    button_flag = true;
    button_timer = millis();
  }

  // Clear button flag
  if (clear_button_flag)
    button_flag = false;
}

/**
 * Measures battery voltage, sets power state, enables or disables charging and controls power LED
*/
void charging_manager(void) {
  // Measure battery voltage
  vbat_measure();

  // Set low battery flag and alarm LED state
  if (battery_voltage_filtered < VBAT_LOW_VOLTAGE_L_MV) {
    battery_low_flag = true;
    led_low_bat(true);
  }
  else if (battery_voltage_filtered > VBAT_LOW_VOLTAGE_U_MV) {
    battery_low_flag = false;
    led_low_bat(false);
  }

  // Detect charger
  check_charging();

  // Set on bat state and stop charging if no charger detected
  if (!charger_connected) {
    power_state = POWER_STATE_ON_BAT;
    set_charging(false);
  }

  // Charger is connected
  else {
    // Stop charging?
    if (battery_voltage_filtered > VBAT_STOP_CHARGE_MV) {
      set_charging(false);
      power_state = POWER_STATE_IDLE;
    }
    
    // Start charging ?
    else if (battery_voltage_filtered < VBAT_START_CHARGE_MV) {
      set_charging(true);
      power_state = POWER_STATE_CHARGING;
    }

    // We are between thresholds and previous state was on battery, so change it to connected, not charging
    else if (power_state == POWER_STATE_ON_BAT) {
      set_charging(false);
      power_state = POWER_STATE_IDLE;
    }
  }

  // Blink with LED indicating that we are on battery
  if (power_state == POWER_STATE_ON_BAT) {
    // LED on
    if (power_state_led_state) {
      led_charger(true);
      if (millis() - power_state_led_timer >= ON_BATTERY_LED_ON) {
        power_state_led_timer = millis();
        power_state_led_state = false;
      }
    }

    // LED off
    else {
      led_charger(false);
      if (millis() - power_state_led_timer >= ON_BATTERY_LED_OFF) {
        power_state_led_timer = millis();
        power_state_led_state = true;
      }
    }
  }

  // Turn LED on constantly if charging
  else if (power_state == POWER_STATE_CHARGING)
    led_charger(true);

  // Turn LED off if not charging
  else
    led_charger(false);
}

/**
 * Measures battery voltage in mV and writes result into battery_voltage and battery_voltage_mv_int variables
*/
void vbat_measure(void) {
  // Measure and calculate battery voltage in mV
  vbat_pin_voltage = (float) analogRead(PIN_ADC_VBAT) / 1023.f * VREF_1_1_ACTUAL_MV;
  battery_voltage = vbat_pin_voltage / (VBAT_RESISTOR_GND / (VBAT_RESISTOR_GND + VBAT_RESISTOR_BAT));

  // Clip to uint16_t range
  if (battery_voltage < 0.f)
    battery_voltage = 0.f;
  else if (battery_voltage > (float) UINT16_MAX)
    battery_voltage = (float) UINT16_MAX;

  // Filter it
  if (battery_voltage_filtered == 0)
    battery_voltage_filtered = battery_voltage;
  else
    battery_voltage_filtered = battery_voltage_filtered * VOLTAGE_FILTER_K + battery_voltage * (1.f - VOLTAGE_FILTER_K);

  // Convert to uint16_t
  battery_voltage_mv_int = (uint16_t) battery_voltage_filtered;
}

/**
 * Callback for interrupt for IMU reading
*/
void buzzer_timer_callback(void) {
  // Low alarm
  if (alarm_state == ALARM_STATE_LOW) {
    // Reset variables from high state
    alarm_pwm = 0.f;

    // State on
    if (!alarm_low_state) {
      // Turn alarm on (led on, 50% square wave)
      analogWrite(PIN_BUZZER, 127);
      led_alarm(true);

      // Count on cycles
      if (alarm_cycles_counter >= ALARM_LOW_ON_CYCLES) {
        // Reset counter and switch to off state
        alarm_cycles_counter = 0;
        alarm_low_state = true;
      }
    }

    // State off
    else {
      // Turn alarm off
      analogWrite(PIN_BUZZER, 0);
      led_alarm(false);

      // Count off cycles
      if (alarm_cycles_counter >= ALARM_LOW_OFF_CYCLES) {
        // Reset counter and switch to on state
        alarm_cycles_counter = 0;
        alarm_low_state = false;
      }
    }

    // Increment number of alarm 
    alarm_cycles_counter++;
  }

  // High alarm
  else if (alarm_state == ALARM_STATE_HIGH) {
    // Reset variables from low state
    alarm_low_state = false;
    alarm_cycles_counter = 0;

    // Turn alarm led on constantly
    led_alarm(true);

    // Generate ramp down sawtooth
    if (alarm_pwm < ALARM_LFO_CYCLE_DECR)
      alarm_pwm = ALARM_PWM_MAX;
    alarm_pwm -= ALARM_LFO_CYCLE_DECR;

    // Write to buzzer
    analogWrite(PIN_BUZZER, (uint8_t) alarm_pwm);
  }

  // Alarm is off
  else {
    // Turn alarm off
    analogWrite(PIN_BUZZER, 0);
    led_alarm(false);

    // Reset variables
    alarm_pwm = 0.f;
    alarm_low_state = false;
    alarm_cycles_counter = 0;
  }
}

/**
 * Callback for interrupt for IMU reading and data sending
*/
void imu_read_interrupt_callback(void) {
  // Proceed only if readings are allowed
  if (stream_enabled) {
    // Enable interrupts (sei) to fix I2C reading
    interrupts();

    // Turn on / off IMU LED
    led_imu_reading(led_imu_state);
    led_imu_state = !led_imu_state;

    // Read data from IMU
    imu_read();

    // Send data to serial port
    send_data();
  }

  // Turn off IMU LED
  else
    led_imu_reading(false);
}

/**
 * Reads accelerometer data and temperature from IMU
*/
void imu_read(void) {
  // Request 8 bytes from IMU (accelerometer + temperature)
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B_uint8);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDRESS, 8_uint8);

  // Add the low and high byte to the acc_ variables
  acc_y = Wire.read() << 8 | Wire.read();
  acc_x = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();

  // Add the low and high byte to the temperature variable
  temperature = Wire.read() << 8 | Wire.read();

  // Convert temperature to celsius
  temperature_c = (((float) temperature) / 340.f) + 36.53;

  // Clip to int8 range
  if (temperature_c < (float) INT8_MIN)
    temperature_c = (float) INT8_MIN;
  else if (temperature_c > (float) INT8_MAX)
    temperature_c = (float) INT8_MAX;

  // Convert to int8 and then to uint8
  temperature_byte = (int8_t) temperature_c;
}

/**
 * Initializes MPU-6050
*/
void imu_setup(void) {
  // Power ON the IMU
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x6B_uint8);
  Wire.write(0x00_uint8);
  Wire.endTransmission();

  // Set accelerometer scale to +/-2g
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x1C_uint8);
  Wire.write(0x00_uint8);
  Wire.endTransmission();

  // Set Digital Low Pass Filter for accelerometer and gyroscope
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x1A_uint8);
  Wire.write(IMU_LOW_PASS_FILTER);
  Wire.endTransmission();
}

/**
 * Turns 2nd LED (charger state) ON or OFF
*/
void led_charger(boolean state) {
  digitalWrite(PIN_LED_CHARGER, !state);
}

/**
 * Turns 3rd LED (calibration state) ON or OFF
*/
void led_calibration(boolean state) {
  digitalWrite(PIN_LED_CALIBRATION, !state);
}

/**
 * Turns 4th LED (imu reading) ON or OFF
*/
void led_imu_reading(boolean state) {
  digitalWrite(PIN_LED_IMU_READING, !state);
}

/**
 * Turns 5th LED (low battery) ON or OFF
*/
void led_low_bat(boolean state) {
  digitalWrite(PIN_LED_LOW_BAT, !state);
}

/**
 * Turns 6th LED (alarm led) ON or OFF
*/
void led_alarm(boolean state) {
  digitalWrite(PIN_LED_ALARM, !state);
}

/**
 * Enables or disables charging process
*/
void set_charging(boolean state) {
  if (state) {
    digitalWrite(PIN_CHARGING_STOP, LOW);
    pinMode(PIN_CHARGING_STOP, INPUT);
  }
  else {
    pinMode(PIN_CHARGING_STOP, OUTPUT);
    digitalWrite(PIN_CHARGING_STOP, HIGH);
  }
}

/**
 * Checks if charger is connected or not
*/
void check_charging(void) {
  charger_connected = digitalRead(PIN_CHARGER_DETECT);
}

/**
 * Checks if button is pressed or not
*/
void check_button(void) {
  button_pressed = !digitalRead(PIN_BTN);
}
