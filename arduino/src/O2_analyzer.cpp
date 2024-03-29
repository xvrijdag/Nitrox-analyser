/*****************************************************************************
  Based on ej's o2 oled analyzer - v0.21
  http://ejlabs.net/arduino-oled-nitrox-analyzer

  License
  -------
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <EEPROM.h>
#include <RunningAverage.h>

void memory_write_int(int p_address, int p_value);
unsigned int memory_read_int(int p_address);

void calibrate_o2();
void calculate_o2_from_running_average();
void read_sensor_to_running_average();
void draw_status();
void draw_lock_screen();
void draw_help();
enum program_mode
{
  MODE_HELP,
  MODE_SHOw_O2,
  MODE_CALIBRATE
};
void switch_mode(program_mode mode);
int update_button_status(bool is_pressed);
void handle_button_press(int action);

// Running average of 10 values
const int running_average_size = 5;
RunningAverage running_average(running_average_size);

// i2c Address 0x3c or 0x48
Adafruit_ADS1115 adc_converter;
const adsGain_t adc_gain = GAIN_SIXTEEN;
const float adc_multiplier = 0.0078125;

///
/// Arduino Configuration
///
// Some displays have a oled reset pin
// this display does not
const int oled_reset = -1;
const int oled_address = 0x3C;
Adafruit_SSD1306 oled_display(oled_reset);

const int button_pin = 5; // push button
//const int led_pin = 13; // led

///
/// Oxygen state
///
double o2_calibration_voltage = 0;
double o2_percentage = 0.0;
double sensor_voltage = 0.0;

///
/// Button State
///
const int single_click_timing = 250; // a single click is when only one click occurs within 500ms
const int hold_click_timing = 1000;  // a long press click is when one click is constant for 2000ms

///
/// Menu / display state
///
int calibration_memory_address = 0;

program_mode current_mode = MODE_HELP;
bool draw_help_done = false;

int mode_counter = 0;
int loop_delay_ms = 5; // execute loop every 10 ms
bool mode_display_advanced = false;
bool mode_display_hold = false;

// last time the button was entered
unsigned long last_down = 0;
unsigned long last_up = 0;
bool double_click_started = true;
bool last_pressed_state = false;

int drawing_iteration = 0; // needed for flashing dot

void setup(void)
{
  Serial.begin(9600);
  oled_display.begin(SSD1306_SWITCHCAPVCC, oled_address);
  // rotate display 180 degrees..
  oled_display.setRotation(2);

  adc_converter.setGain(adc_gain);
  adc_converter.begin(); // ads1115 start

  pinMode(button_pin, INPUT_PULLUP);

  running_average.clear();
  for (int i = 0; i <= running_average_size; i++)
  {
    read_sensor_to_running_average();
  }

  o2_calibration_voltage = memory_read_int(calibration_memory_address);
  if (o2_calibration_voltage > 10000)
  {
    calibrate_o2();
  }
}

void loop(void)
{
  mode_counter++;
 
  if (current_mode == MODE_HELP)
  {
    // only the first time..
    draw_help();
    // detect button..

    if (digitalRead(button_pin) == LOW)
    {
      switch_mode(MODE_SHOw_O2);
    }
    return;
  }
 
  if (current_mode == MODE_SHOw_O2)
  {
    // detect button presses.
    if (!mode_display_hold)
    {
      // takes 15ms..
      // execute every 5 updates..
      if (mode_counter % 10 == 1)
      {
        calculate_o2_from_running_average();
      }
    }

    int button_action = update_button_status(digitalRead(button_pin) == LOW);
    if (button_action != 0)
    {
      Serial.println(button_action);
      handle_button_press(button_action);
    }

    // takes 170s ms, execute update every 2s
    if (mode_counter % 40 == 1 && !mode_display_hold)
    {
      draw_status();
    }
  }

  if (current_mode == MODE_CALIBRATE)
  {
    calibrate_o2();
    switch_mode(MODE_SHOw_O2);
    draw_status();
  }

  delay(loop_delay_ms);
}

void handle_button_press(int action)
{

  if (action == 1)
  {
    // single click
    mode_display_hold = !mode_display_hold;
    if (mode_display_hold)
    {
      draw_status();
      draw_lock_screen();
    }
    return;
  }

  if (action == 2)
  {
    // double click
    // advanced mode
    mode_display_advanced = !mode_display_advanced;
    if (mode_display_hold)
    {
      draw_status();
      draw_lock_screen();
    }
    return;
  }

  if (action == 3)
  {
    switch_mode(MODE_CALIBRATE);
    return;
  }
}

void reset_button_status(bool current_status)
{
  last_down = 0;
  last_up = 0;
  double_click_started = false;
  last_pressed_state = current_status;
}

int update_button_status(bool is_pressed)
{
  if (is_pressed && last_pressed_state != is_pressed)
  {
    if (millis() - last_up < single_click_timing)
    {
      double_click_started = true;
    }
    // button was pressed..
    last_down = millis();
    last_pressed_state = is_pressed;
  }
  else if (!is_pressed && last_pressed_state != is_pressed)
  {
    if (last_down == 0)
    {
      reset_button_status(is_pressed);
      return 0;
    }
    last_up = millis();
    // button was released..
    last_pressed_state = is_pressed;

    if (double_click_started)
    {
      reset_button_status(is_pressed);
      return 2;
    }
  }
  else if (is_pressed && last_pressed_state == is_pressed)
  {
    // button was pressed en still is pressed..
    if (last_down != 0 && millis() - last_down > hold_click_timing)
    {
      reset_button_status(is_pressed);
      return 3;
    }
    last_pressed_state = is_pressed;
  }
  else if (!is_pressed && last_pressed_state == is_pressed)
  {
    // button was not pressed and still is not pressed..
    if (last_down != 0 && millis() - last_down > single_click_timing)
    {
      reset_button_status(is_pressed);
      return 1;
    }
  }
  return 0;
}

void switch_mode(program_mode mode)
{
  // reset mode variables
  draw_help_done = true;
  mode_counter = 0;
  reset_button_status(digitalRead(button_pin) == LOW);
  current_mode = mode;
}

void calculate_o2_from_running_average()
{
  double currentmv = 0;
  read_sensor_to_running_average();
  currentmv = running_average.getAverage();
  currentmv = abs(currentmv);
  o2_percentage = (currentmv / o2_calibration_voltage) * 20.9;
  sensor_voltage = currentmv * adc_multiplier;
}

void read_sensor_to_running_average()
{
  int16_t millivolts = 0;
  millivolts = adc_converter.readADC_Differential_0_1();
  running_average.addValue(millivolts);
}

void calibrate_o2()
{
  for (int i = 0; i <= running_average_size; i++)
  {
    read_sensor_to_running_average();
    delay(loop_delay_ms);
  }
  double currentmv = abs(running_average.getAverage());
  memory_write_int(calibration_memory_address, currentmv); // write to eeprom
  o2_calibration_voltage = currentmv;
  // show "calibrating"
  oled_display.clearDisplay();
  oled_display.setTextColor(WHITE);
  oled_display.setCursor(0, 0);
  oled_display.setTextSize(2);
  oled_display.println(F("Calibrated"));
  oled_display.println(F("20.9% O2"));
  oled_display.display();
  delay(1000);
}

void draw_status()
{
  drawing_iteration++;
  oled_display.clearDisplay();
  oled_display.setTextColor(WHITE);
  oled_display.setCursor(0, 0);

  if (sensor_voltage < 0.05)
  {
    oled_display.setTextSize(1);
    oled_display.println(F("Error: O2 < 0.05 mv"));
    oled_display.println(F("Is the sensor"));
    oled_display.println(F("connected?"));
    oled_display.display();
    return;
  }
  //basic mode
  if (!mode_display_advanced)
  {
    oled_display.setTextSize(4);
    oled_display.print(o2_percentage, 1);
    if (o2_percentage < 100)
    {
      oled_display.println(F("%"));
    }

  } 
  //advanced screen
  else 
  {
    oled_display.setTextSize(2);
    oled_display.print(F("O2   "));
    oled_display.print(o2_percentage, 1);
    oled_display.println(F("%"));

    oled_display.setTextSize(1);
    oled_display.print(F("Voltage    "));
    oled_display.print(sensor_voltage, 2);
    oled_display.println(F("mv"));
    oled_display.print(F("Calibrated "));
    oled_display.print(o2_calibration_voltage*adc_multiplier, 2);
    oled_display.println(F("mv"));
  }
  //flashing dot in right lower corner
  if (drawing_iteration % 4 == 0) 
  {
    oled_display.setTextSize(1);
    oled_display.setCursor(115, 25);
    oled_display.setTextColor(WHITE);
    oled_display.print(F("."));
  }
  oled_display.display();
}

void draw_lock_screen()
{
  oled_display.setTextSize(1);
  oled_display.setTextColor(BLACK, WHITE);
  oled_display.setCursor(90, 25);
  oled_display.print(F(" HOLD "));
  oled_display.display();
}

void draw_help()
{
  if (draw_help_done)
  {
    return;
  }
  draw_help_done = true;
  oled_display.clearDisplay();
  oled_display.setTextColor(WHITE);
  oled_display.setCursor(0, 0);
  oled_display.setTextSize(1);
  oled_display.println(F("Short press: HOLD"));
  oled_display.println(F("Long press: CALIBRATE"));
  oled_display.println(F("Double press: ADV"));
  oled_display.println(F("Press to start"));
  oled_display.display();
}

void memory_write_int(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

unsigned int memory_read_int(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}
