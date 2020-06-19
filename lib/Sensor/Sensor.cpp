/*!
* @brief Class that contains information about the 9 dof spatial sensor
*
* @section license License
*
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

#include "Sensor.h"
#include "NV.h"
#include "Webpage.h"
uint8_t sensor_time_out;

/*! @brief class constructor
 */
Sensor::Sensor()
{
	//< instantiate, discover and initialize
	bno = new Adafruit_BNO055(-1, I2CADDR);
	sensor_found = bno->begin(Adafruit_BNO055::OPERATION_MODE_NDOF);
	installCalibration();
	sensor_time_out = 0;
}

/*! @brief test that the Sensor is still working
*
*	@param client WiFiClient reference when called from sendNewValues
*/
void Sensor::checkSensor(WiFiClient client)
{
	/* Get the system status values (mostly for debugging purposes) */
  	uint8_t system_status, self_test_results, system_error;
  	system_status = self_test_results = system_error = 0;
  	bno->getSystemStatus(&system_status, &self_test_results, &system_error);
		// check Sensor if waited > 10 seconds since sendNewValues is called every 0.5 seconds
		/* Self Test Results: 1 = test passed, 0 = test failed
     	Bit 0 = Accelerometer self test
     	Bit 1 = Magnetometer self test
     	Bit 2 = Gyroscope self test
     	Bit 3 = MCU self test
   		*/
	client.print (F("SS_STSStatus=")); client.println (0x08 & self_test_results?"pass+":"fail!");
	client.print (F("SS_STGStatus=")); client.println (0x04 & self_test_results?"pass+":"fail!");
	client.print (F("SS_STMStatus=")); client.println (0x02 & self_test_results?"pass+":"fail!");
	client.print (F("SS_STAStatus=")); client.println (0x01 & self_test_results?"pass+":"fail!");
	if (system_error > 0 || system_status == 1 || !sensor_found) {
		sensor_found = bno->begin(Adafruit_BNO055::OPERATION_MODE_NDOF);	//< restart Sensor
		delay(20);
		if (sensor_found) {
	    	if (sensor->DEBUG_SENSOR) {
				Serial.println (F("Sensor found"));
			}
			bno->setExtCrystalUse(true);
			webpage->setUserMessage(F("Sensor error... restarting sensor!"));
		}
		installCalibration();
	} else { // no error
		if (system_status == 5) {
			webpage->setUserMessage(F("Sensor fusion algorithm running+"));
		} else if (system_status == 6){
			webpage->setUserMessage(F("Sensor fusion algorithm not running!"));
		} else if (system_status == 4){
			webpage->setUserMessage(F("Executing Sensor Self-Test"));
		} else if (system_status == 3){
			webpage->setUserMessage(F("Sensor System Iniitalizing"));
		} else if (system_status == 2){
			webpage->setUserMessage(F("Initializing Sensor Peripherals"));
		}
	}

	/* System Status (see section 4.3.58)
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusion algorithm running
     6 = System running without fusion algorithms

		System Error (see section 4.3.59)
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error
   */
}

/*! @brief install previously stored calibration data from EEPROM if it looks valid.
*
* Use stock Adafruit library so pulled from
* post by protonstorm at https://forums.adafruit.com/viewtopic.php?f=19&t=75497
*/
void Sensor::installCalibration()
{
	byte nbytes = (byte)sizeof(nv->BNO055cal);
	//< read from EEPROM, qualify
	nv->get();	//< read from EEPROM, qualify
	uint8_t i;
	for(i = 0; i < nbytes; i++) {
	    if (nv->BNO055cal[i] != 0)
		break;
	}
	if (i == nbytes) {
		webpage->setUserMessage(F("Sensor calibration not valid"));
	    return;	//< all zeros can't be valid
	}
	bno->setMode (Adafruit_BNO055::OPERATION_MODE_CONFIG);	//< put into config mode
	delay(25);

	// set from NV
	for(uint8_t i = 0; i < nbytes; i++) {
	    Wire.beginTransmission((uint8_t)I2CADDR);
	    Wire.write((Adafruit_BNO055::adafruit_bno055_reg_t)
	    		((uint8_t)(Adafruit_BNO055::ACCEL_OFFSET_X_LSB_ADDR)+i));
	    Wire.write(nv->BNO055cal[i]);
	    Wire.endTransmission();
	}
	bno->setMode (Adafruit_BNO055::OPERATION_MODE_NDOF);	//< restore NDOF mode
	delay(25);
}

/*! @brief read the sensor calibration values and save into EEPROM.
*
* Use stock Adafruit library so pulled from post by protonstorm at
* https://forums.adafruit.com/viewtopic.php?f=19&t=75497
*/
void Sensor::saveCalibration()
{
	byte nbytes = (byte)sizeof(nv->BNO055cal);
	// put into config mode
	bno->setMode (Adafruit_BNO055::OPERATION_MODE_CONFIG);
	delay(25);

	// request all bytes starting with the ACCEL
	Wire.beginTransmission((uint8_t)I2CADDR);
	Wire.write((uint8_t)(Adafruit_BNO055::ACCEL_OFFSET_X_LSB_ADDR));
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)I2CADDR, nbytes);

	// wait for all 22 bytes to be available
	while (Wire.available() < nbytes);

	// copy to NV
	for (uint8_t i = 0; i < nbytes; i++) {
	    nv->BNO055cal[i] = Wire.read();
	}
	// restore NDOF mode
	bno->setMode (Adafruit_BNO055::OPERATION_MODE_NDOF);
	delay(25);
	// save in EEPROM
	nv->put();
}

/*! @brief read the current temperature, in degrees C
*
* @return the sensor temperature
*/
int8_t Sensor::getTempC()
{
	if (sensor_found) {
	    return temperature;
	}
	return (-1);
}

/*! @brief return whether sensor is connected and calibrated
*
* internal sensor calibration routine returns values in the range 0 - 3
* @param sys system calibration value
* @param gyro gyroscope calibration value
* @param accel accelerometer calibration value
* @param mag magnetometer calibration value
*
* @return true (calibrated) if all sensors are > 1
*/
bool Sensor::calibrated(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag)
{
	if (!sensor_found)
	    return (false);
	sys = 0;
	gyro = 0;
	accel = 0;
	mag = 0;;
	bno->getCalibration(&sys, &gyro, &accel, &mag);
	return (sys >= 1 && gyro >= 1 && accel >= 1 && mag >= 1);
}

/*! @brief return the current sensor azimuth 
*
* This does not query the BNO055 sensor, just returns the last value read
* 
* @return the last measured Sensor azimuth
*/
float Sensor::getSensorAz ()
{
  return az_sensor;
}

/*! @brief  return the current sensor elevation 
*
* This does not query the BNO055 sensor, just returns the last value read
* 
* @return the last measured Sensor elevation
*/
float Sensor::getSensorEl ()
{
  return el_sensor;
}

/*! @brief  read the current az and el, corrected for mag decl but not necessarily calibrated.

 * N.B. we assume this will only be called if we know the sensor is connected.
 * N.B. Adafruit board:
 *   the short dimension is parallel to the antenna boom,
 *   the populated side of the board faces upwards and
 *   the side with the control signals (SDA, SCL etc) points in the rear direction of the antenna pattern.
 * Note that az/el is a left-hand coordinate system.
 * This is only called repeatedly from loop()
 */
void Sensor::readAzElT ()
{
  imu::Vector<3> euler = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
  az_sensor = fmod (euler.x() + nv->mag_decl + 540, 360);
  el_sensor = (euler.z());
  temperature = bno->getTemp();
}

/*! @brief send latest values to web page
*
* N.B. labels must match ids in web page
*/

void Sensor::sendNewValues (WiFiClient client)
{
	if (!sensor_found) {
	    client.println (F("SS_Status=Not found!"));
	    client.println (F("SS_Save=false"));
	    // restart Sensor
		sensor_found = bno->begin(Adafruit_BNO055::OPERATION_MODE_NDOF);
		delay(25);
		if (sensor_found) {
			webpage->setUserMessage(F("Sensor error... restarting sensor!"));
		}
	}

	client.print (F("SS_Az=")); client.println (az_sensor, 1);
	client.print (F("SS_El=")); client.println (el_sensor, 1);

	client.print (F("SS_Temp=")); client.println (temperature);
	if (sensor_time_out++ > 20) {
		sensor_time_out = 0;
		uint8_t sys, gyro, accel, mag;
		bool calok = calibrated (sys, gyro, accel, mag);
		if (calok) {
	    	client.println (F("SS_Status=Ok+"));
		}
		else {
	    	client.println (F("SS_Status=Uncalibrated!"));
		}
		client.print (F("SS_SCal=")); client.println (sys);
		client.print (F("SS_GCal=")); client.println (gyro);
		client.print (F("SS_MCal=")); client.println (mag);
		client.print (F("SS_ACal=")); client.println (accel);

		client.print (F("SS_Save="));
		if (calok && sys == 3 && gyro == 3 && accel == 3 && mag == 3) {
	    	client.println (F("true"));
		} else {
	    	client.println (F("false"));
		}
		checkSensor(client);
	}
}

/*! @brief process name = value pair
*
* @param name the web page id where value was entered
* @param value a value to operate on, if needed
* @return return true if Sensor handles this 'name' command, false if Sensor doesn't recognize it
*/
bool Sensor::overrideValue (char *name, char *value)
{
	if (!strcmp (name, "SS_Save")) {
	    saveCalibration();
	    webpage->setUserMessage (F("Sensor calibrations saved to EEPROM+"));
	    return (true);
	}
	return (false);
}
