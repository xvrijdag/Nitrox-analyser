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


void read_sensor(int adc);
void EEPROMWriteInt(int p_address, int p_value);
unsigned int EEPROMReadInt(int p_address);
int calibrate(int x);
void analysing(int x, int cal);
void lock_screen(int advanced, long pause);
void splash();

#define RA_SIZE 3
RunningAverage RA(RA_SIZE);

Adafruit_ADS1115 ads; //(0x3c); // or 0x48 EHG20190321

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

const int buttonPin = 5; // push button
const int ledPin = 13; // led

double calibrationv = 0;
float multiplier;
int advanced = 0;
int adv_switched = 0;
bool startup = true;

const int cal_holdTime = 1; // 1 sec button hold to calibration
const int adv_holdTime = 2; // 2 sec hold to switch to basic screen
const int maxi_holdTime = 3;

long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_secs_held; // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime; // how long since the button was first pressed
int active = 0;

void read_sensor(int adc = 0) {
  int16_t millivolts = 0;
  millivolts = ads.readADC_Differential_0_1();
  RA.addValue(millivolts);
}

void setup(void) {

  Serial.begin(9600);
  Serial.println("Hello!");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  ads.setGain(GAIN_SIXTEEN);
  multiplier = 0.0078125F;
  ads.begin(); // ads1115 start

  pinMode(buttonPin, INPUT_PULLUP);

  RA.clear();
  for (int cx = 0; cx <= RA_SIZE; cx++) {
    read_sensor(0);
  }

  calibrationv = EEPROMReadInt(0);
  if (calibrationv > 10000) {
    calibrationv = calibrate(0);
  }

}

void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

int calibrate(int p_address) {

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print(F("Calibrate"));
  display.display();

  double result;
  for (int cx = 0; cx <= RA_SIZE; cx++) {
    read_sensor(0);
  }
  result = RA.getAverage();
  result = abs(result);
  EEPROMWriteInt(p_address, result); // write to eeprom

  delay(1000);
  active = 0;
  return result;
}

void analysing(int cal) {
  double currentmv = 0;
  double result;
  double mv = 0.0;

  read_sensor(0);
  currentmv = RA.getAverage();
  currentmv = abs(currentmv);

  result = (currentmv / cal) * 20.9;
  mv = currentmv * multiplier;
  Serial.println(result);
  
  if (result > 40.0) digitalWrite(ledPin, HIGH);
  else digitalWrite(ledPin, LOW);

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  if (mv < 0.02) {
    display.setTextSize(2);
    display.println(F("Sensor"));
    display.print(F("Error!"));
  } else {
    if (advanced == 0) {
      display.setTextSize(4);
      if (result<100){
        display.print(result, 1);
        display.println(F("%"));
      } else {
        display.print(result, 1);
      }
    }
    if (advanced == 1) {

      display.setTextSize(1);
      display.println(F("   O2    "));

      display.setTextSize(2);
      display.print(result, 1);
      display.println(F("%"));

      display.setTextSize(1);
      display.print(mv, 2);
      display.print(F("mv"));
    }

    if (active % 4) {
      display.setTextSize(1);
      display.setCursor(115, 25);
      display.setTextColor(WHITE);
      display.print(F("."));
    }
    // menu
    if (secs_held < maxi_holdTime && active > 16) {
      display.setTextSize(1);
      display.setCursor(0, 18);
      display.setTextColor(BLACK, WHITE);
      if (secs_held >= cal_holdTime && secs_held < adv_holdTime) {
        display.print(F("   CAL    "));
      }
      if (secs_held >= adv_holdTime && secs_held < maxi_holdTime) {
        display.print(F("   ADV    "));
      }
    }

  }
  display.display();
}

void lock_screen(int advanced, long pause = 5000) {
  display.setTextSize(1);
  if (advanced == 0)
  {
    display.setTextColor(BLACK, WHITE);
  }
  else display.setTextColor(WHITE);
  display.setCursor(90, 25);
  display.print(F(" HOLD "));
  display.display();
  for (int i = 0; i < pause; ++i) {
    while (digitalRead(buttonPin) == HIGH) {
    }
  }
  active = 0;
}

void splash() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println(F("O2 Analyzer"));
  display.println(F("0s: HOLD"));
  display.println(F("1s: CALIBRATE"));
  display.println(F("2s: ADV / BASIC MODE"));
  display.display();
  delay(3000);
  active = 0;
}

void loop(void) {

  if (startup == true) {
    startup = false;
    splash();
  }

  int current = digitalRead(buttonPin);

  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
    firstTime = millis();
    active = 17;
  }

  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  if (millis_held > 2) {
    if (current == HIGH && previous == LOW) {
      if (secs_held < cal_holdTime) {
        lock_screen(advanced);
      }
      if (secs_held >= cal_holdTime && secs_held < adv_holdTime) {
        calibrationv = calibrate(0);
      }
      if (secs_held >= adv_holdTime && secs_held < maxi_holdTime && advanced == 0) {
        advanced = 1;
        adv_switched = 1;
      }
      if (secs_held >= adv_holdTime && secs_held < maxi_holdTime && advanced == 1 && adv_switched == 0) {
        advanced = 0;
        adv_switched = 0;
      }
      adv_switched = 0;
    }
  }
  previous = current;
  prev_secs_held = secs_held;

  analysing(calibrationv);
  delay(200);

  active++;
}
