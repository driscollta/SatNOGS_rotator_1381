/*!
* @brief main starts the various modules and continuously checks for WiFi, Serial and Sensor activity
*
* @section dependencies Dependencies
*
* This firmware depends on
* Adafruit Unified Sensor v1.1.3 Library
* Adafruit BNO055 v1.2.0
* Adafruit BusIO v1.3.0
* Adafruit PWM Servo Driver v2.4.0
* being present on your system. Please make sure you have installed the 
* latest version before using this firmware.
* 
* @section license License

* Copyright (c) 2020 Tom Driscoll. 
* Based on code by Elwood Downey found on Clearskyinstitute.com and published in QEX Mar/Apr 2016.
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions: The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
* WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN 
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
* OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include "Webpage.h"
#include "Sensor.h"
#include "NV.h"
#include "Gimbal.h"
#include "Easycomm.h"
#include "UpgradeESP32.h"

#define BAUDRATE        115200  ///<  Baudrate of Easycomm II protocol
#define WP_INTERVAL      401     ///<  milliseconds interval for checking WebPage
#define EC_INTERVAL      50      ///<  milliseconds interval for checking Serial for Easycomm commands
#define SENSOR_INTERVAL  233 ///<  milliseconds interval for reading Sensor
#define CHECK_SENSOR_INTERVAL   30017 ///<  milliseconds interval for checking Sensor status

Sensor *sensor;
Webpage *webpage;
NV *nv;
Gimbal *gimbal;
Easycomm *easycomm;
UpgradeESP32 *upgradeESP32;

bool is_timed_out(uint32_t start_time, uint32_t time_interval);
uint32_t previous_time_ec;
uint32_t previous_time_wp;
uint32_t previous_time_sensor;
uint32_t previous_time_check_sensor;

void setup() {
  previous_time_ec = millis();
  previous_time_wp = millis();
  previous_time_sensor = millis();
  previous_time_check_sensor = millis();
  Serial.begin(BAUDRATE);
  delay(1000);
  nv = new NV();
  sensor = new Sensor();
  gimbal = new Gimbal();
  webpage = new Webpage();
  upgradeESP32 = new UpgradeESP32();

  delay(1000);
  sensor->checkSensor();
}

void loop() {
  if (!gimbal->isCalibrating){
    // check for rotctl activity on Serial port
    if (is_timed_out(previous_time_ec, EC_INTERVAL)) {
      previous_time_ec = millis();
      easycomm->easycomm_process();
    }
    // check for WiFi activity
    if (is_timed_out(previous_time_wp, WP_INTERVAL)) {
      previous_time_wp = millis();
      webpage->checkEthernet();
      upgradeESP32->checkPortServer();
    }

    // read Sensor position, Temperature
    if (is_timed_out(previous_time_sensor, SENSOR_INTERVAL)) {
      previous_time_sensor = millis();
      sensor->readAzElT();
    }
    // read Sensor status
    if (is_timed_out(previous_time_check_sensor, CHECK_SENSOR_INTERVAL)) {
      previous_time_check_sensor = millis();
      sensor->checkSensor();
    }
  }
}

bool is_timed_out(uint32_t startTime_ms, uint32_t time_out_time) {
  uint32_t delta_time = millis() - startTime_ms;
  if (delta_time < 0) {
    // rolled-over since we started! startTime_ms is really big, millis() is very small.
    // delta_time is negative, have to add the maximum unsigned long
    delta_time += MAX_UNSIGNED_LONG;
  }
  return (delta_time > time_out_time);
}