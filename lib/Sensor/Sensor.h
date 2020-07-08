/*!
* @brief Class that contains information about the 9 dof spatial sensor
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

#ifndef _SENSOR_H
#define _SENSOR_H

#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define MAG_DECLINATION 13.23	///< default magnetic declination; should be set thru Webpage

class Sensor {

    private:
	int8_t temperature;
	float az_sensor, el_sensor;
	//< bit, status for debugging and display
	/* Self Test Results: 1 = test passed, 0 = test failed
    Bit 0 = Accelerometer self test
    Bit 1 = Magnetometer self test
    Bit 2 = Gyroscope self test
    Bit 3 = MCU self test
   	*/
	uint8_t sys, gyro, accel, mag;
	uint8_t system_status, self_test_results, system_error;
	bool calok;
	Adafruit_BNO055 *bno;	// sensor detail
	const bool DEBUG_SENSOR = false;
	bool sensor_found;		//< whether sensor is connected
	bool calibrated(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag);
	enum {
	    I2CADDR = 0x28,		// I2C bus address of BNO055
	};

    public:

	Sensor();
	void checkSensor();
	int8_t getTempC();
	void saveCalibration(void);
	float getSensorAz ();
	float getSensorEl ();
	void readAzElT ();
	void sendNewValues (WiFiClient client);
	bool connected() { return sensor_found; };
	void installCalibration(void);
	bool overrideValue (char *name, char *value);
};

extern Sensor *sensor;

#endif // _SENSOR_H
