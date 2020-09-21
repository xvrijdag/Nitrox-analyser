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
#include <Adafruit_ADS1015.h>
#include <EEPROM.h>
#include <RunningAverage.h>

void memory_write_int(int p_address, int p_value);
unsigned int memory_read_int(int p_address);

int calibrate_o2(int address);
void calculate_o2_from_running_average();
void read_sensor_to_running_average();
void display_status();
void lock_screen();
void oled_display_splash();
void oled_display_text(const __FlashStringHelper *text, int text_size);


// Running average of 3 values
const int running_average_size = 3;
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
const int led_pin = 13; // led

///
/// Oxygen state
/// 

double o2_calibration_voltage = 0;
double o2_percentage = 0.0;
double sensor_voltage = 0.0;

///
/// Button Configuration
/// 

const int button_holdtime_calibration = 1; // 1 sec button hold to calibration
const int button_holdtime_advanced = 2; // 2 sec hold to switch to basic screen
const int button_holdtime_maximum = 3;

///
/// Button State
/// 

long button_millis_held;    // How long the button was held (milliseconds)
long button_secs_held;      // How long the button was held (seconds)
long button_prev_secs_held; // How long the button was held in the previous check
byte button_previous = HIGH; // What was the previous state
unsigned long button_first_time_pressed; // how long since the button was first pressed
int button_active = 0;


///
/// Menu / display state
///

int display_advanced_mode = 0;
int adv_switched = 0;
bool show_splash = true;

void setup(void) {
  Serial.begin(9600);
  oled_display.begin(SSD1306_SWITCHCAPVCC, oled_address);
  adc_converter.setGain(adc_gain);
  adc_converter.begin(); // ads1115 start

  pinMode(button_pin, INPUT_PULLUP);

  running_average.clear();
  for (int i = 0; i <= running_average_size; i++) {
    read_sensor_to_running_average();
  }

  o2_calibration_voltage = memory_read_int(0);
  if (o2_calibration_voltage > 10000) {
    o2_calibration_voltage = calibrate_o2(0);
  }
}

void loop(void) {
  // show splash screen once..
  oled_display_splash();

  int current = digitalRead(button_pin);


  if (current == LOW && button_previous == HIGH && (millis() - button_first_time_pressed) > 200) {
    button_first_time_pressed = millis();
    button_active = 17;
  }

  button_millis_held = (millis() - button_first_time_pressed);
  button_secs_held = button_millis_held / 1000;

  if (button_millis_held > 2) {
    if (current == HIGH && button_previous == LOW) {
      if (button_secs_held < button_holdtime_calibration) {
        lock_screen();
      }
      if (button_secs_held >= button_holdtime_calibration && button_secs_held < button_holdtime_advanced) {
        o2_calibration_voltage = calibrate_o2(0);
      }
      if (button_secs_held >= button_holdtime_advanced && button_secs_held < button_holdtime_maximum && display_advanced_mode == 0) {
        display_advanced_mode = 1;
        adv_switched = 1;
      }
      if (button_secs_held >= button_holdtime_advanced && button_secs_held < button_holdtime_maximum && display_advanced_mode == 1 && adv_switched == 0) {
        display_advanced_mode = 0;
        adv_switched = 0;
      }
      adv_switched = 0;
    }
  }
  button_previous = current;
  button_prev_secs_held = button_secs_held;

  calculate_o2_from_running_average();
  delay(200);

  button_active++;
}

void calculate_o2_from_running_average() {
  double currentmv = 0;
  read_sensor_to_running_average();
  currentmv = running_average.getAverage();
  currentmv = abs(currentmv);
  o2_percentage = (currentmv / o2_calibration_voltage) * 20.9;
  sensor_voltage = currentmv * adc_multiplier;
  Serial.println(o2_percentage);
  
  if (o2_percentage > 40.0) {
    digitalWrite(led_pin, HIGH);
  } else {
    digitalWrite(led_pin, LOW);
  }
  display_status();
}

void read_sensor_to_running_average() {
  int16_t millivolts = 0;
  millivolts = adc_converter.readADC_Differential_0_1();
  running_average.addValue(millivolts);
}

int calibrate_o2(int p_address) {
  // show "calibrating"
  oled_display_text(F("Calibrating"), 2);

  for (int i = 0; i <= running_average_size; i++) {
    read_sensor_to_running_average();
  }

  double currentAverage = running_average.getAverage();
  currentAverage = abs(currentAverage);
  memory_write_int(p_address, currentAverage); // write to eeprom

  // show the user that we are currently calibrating for 1 second..
  delay(1000);
  button_active = 0;
  
  return currentAverage;
}

void oled_display_text(const __FlashStringHelper *text, int text_size) {
  oled_display.clearDisplay();
  oled_display.setTextColor(WHITE);
  oled_display.setCursor(0, 0);
  oled_display.setTextSize(text_size);
  oled_display.print(F("TEST"));
  oled_display.display();
}


void display_status()
{
  oled_display.clearDisplay();
  oled_display.setTextColor(WHITE);
  oled_display.setCursor(0, 0);

  if (sensor_voltage < 0.02) {
    oled_display.setTextSize(2);
    oled_display.println(F("Sensor"));
    oled_display.print(F("Error!"));
  } else {
    if (display_advanced_mode == 0) {
      oled_display.setTextSize(4);
      if (o2_percentage<100){
        oled_display.print(o2_percentage, 1);
        oled_display.println(F("%"));
      } else {
        oled_display.print(o2_percentage, 1);
      }
    }
    if (display_advanced_mode == 1) {

      oled_display.setTextSize(1);
      oled_display.println(F("   O2    "));

      oled_display.setTextSize(2);
      oled_display.print(o2_percentage, 1);
      oled_display.println(F("%"));

      oled_display.setTextSize(1);
      oled_display.print(sensor_voltage, 2);
      oled_display.print(F("mv"));
    }

    if (button_active % 4) {
      oled_display.setTextSize(1);
      oled_display.setCursor(115, 25);
      oled_display.setTextColor(WHITE);
      oled_display.print(F("."));
    }
    // menu
    if (button_secs_held < button_holdtime_maximum && button_active > 16) {
      oled_display.setTextSize(1);
      oled_display.setCursor(0, 18);
      oled_display.setTextColor(BLACK, WHITE);
      if (button_secs_held >= button_holdtime_calibration && button_secs_held < button_holdtime_advanced) {
        oled_display.print(F("   CAL    "));
      }
      if (button_secs_held >= button_holdtime_advanced && button_secs_held < button_holdtime_maximum) {
        oled_display.print(F("   ADV    "));
      }
    }
  }
  oled_display.display();
}

void lock_screen() {

  oled_display.setTextSize(1);
  if (display_advanced_mode == 0)
  {
    oled_display.setTextColor(BLACK, WHITE);
  }
  else oled_display.setTextColor(WHITE);
  oled_display.setCursor(90, 25);
  oled_display.print(F(" HOLD "));
  oled_display.display();
  
  int pause = 5000;
  for (int i = 0; i < pause; ++i) {
    while (digitalRead(button_pin) == HIGH) {
    }
  }
  button_active = 0;
}

void oled_display_splash() {
  if (!show_splash) {
    return;
  }

  show_splash = false;
  oled_display.clearDisplay();
  oled_display.setTextColor(WHITE);
  oled_display.setCursor(0, 0);
  oled_display.setTextSize(1);
  oled_display.println(F("O2 Analyzer"));
  oled_display.println(F("0s: HOLD"));
  oled_display.println(F("1s: CALIBRATE"));
  oled_display.println(F("2s: ADV / BASIC MODE"));
  oled_display.display();
  delay(3000);
  button_active = 0;
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