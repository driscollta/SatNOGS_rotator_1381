/*!
 * @brief Class to control two motors to track a target az and el using the Adafruit I2C interface
 * 
* @section dependencies Dependencies
*
* This firmware depends on
* 	Adafruit Unified Sensor v1.1.3 Library
* 	Adafruit BNO055 v1.2.0
* 	Adafruit BusIO v1.3.0
* 	Adafruit PWM Servo Driver v2.4.0
* being present on your system. Please make sure you have installed the 
* latest version before using this firmware.
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

#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include "Util.h"
#include "Gimbal.h"
#include "NV.h"
#include "Webpage.h"
#include "Sensor.h"
//use pin 21 to read the PCA9685 OE. This pin has a pull-down resistor built into it
#define PCA9685OEPin 21

/*! @brief Constructor for Gimbal class
 */
Gimbal::Gimbal()
{
	// set up PCA9685 OE monitor pin
	pinMode(PCA9685OEPin, INPUT);
	//< first confirm whether controller is present
	Wire.begin();
	Wire.beginTransmission(I2C_ADDR);
	gimbal_found = (Wire.endTransmission() == 0);
	if (!gimbal_found && gimbal->DEBUG_GIMBAL) {
		Serial.println(F("PWM controller not found"));
		return;
	}

	//< instantiate PWM controller
	pwm = new Adafruit_PWMServoDriver(I2C_ADDR);
	pwm->begin();
	pwm->setPWMFreq(SERVO_FREQ);

	//< record axis assignments
	motor[0].servo_num = MOT1_UNIT;
	motor[1].servo_num = MOT2_UNIT;

	//< initalize each motor state
	nv->get();
	motor[0].min = nv->mot0min;
	motor[0].max = nv->mot0max;
	motor[0].pos = 0;
	motor[0].atmin = false;
	motor[0].atmax = false;
	motor[0].az_scale = 0;
	motor[0].el_scale = 0;

	motor[1].min = nv->mot1min;
	motor[1].max = nv->mot1max;
	motor[1].pos = 0;
	motor[1].atmin = false;
	motor[1].atmax = false;
	motor[1].az_scale = 0;
	motor[1].el_scale = 0;

	//< initalize to arbitrary, but at least defined, state
	init_step = 0;
	best_azmotor = 0;
	last_update = 0;
	prevfast_az = prevfast_el = -1000;
	prevstop_az = prevstop_el = -1000;
	isCalibrating = false;
	installCalibration();
}

/*! @brief reset initStep to 0 to redo calibration
 */
void Gimbal::resetInitStep()
{
	Gimbal::init_step = 0;
}

/*! @brief move motors towards the given new target az and el 
*
* @param az_t "target" azimuth in degrees
* @param el_t "target" elevation in degrees
*
* This function blocks while reading Sensor.
* Make sure sensor is connected before calling!
*/
void Gimbal::moveToAzEl(float az_t, float el_t)
{
	uint32_t _now = millis();
	//< only update every UPD_PERIOD
	if (_now < last_update + UPD_PERIOD) {
		return;
	}
	last_update = _now;
	//< read current sensor orientation
	//< this blocks main->loop() especially while calibrating
	sensor->readAzElT();
	//< get sensor azimuth angle in degrees
	float _az_s = sensor->getSensorAz();
	//< get sensor elevation angle in degrees
	float _el_s = sensor->getSensorEl();
	if (_az_s < 0 || _az_s > 360 || _el_s < 0 || _el_s > 90) {
		return;
	}
	if (gimbal->DEBUG_GIMBAL) {
		Serial.print("prevfast_el, el_s: (");
		Serial.print(prevfast_el); Serial.print(", "); Serial.print(_el_s); Serial.println(")");
		Serial.print("prevfast_az, az_s: (");
		Serial.print(prevfast_az); Serial.print(", "); Serial.print(_az_s); Serial.println(")");
		Serial.print("az distance: "); Serial.println(azDist(prevfast_az, _az_s));
	}
	//< only check further when motion has stopped as evidenced by stable sensor values
	if (fabs(azDist(prevfast_az, _az_s)) < MAX_SETTLE && fabs(_el_s - prevfast_el) < MAX_SETTLE) {
		//< seek target if calibrated, else calibrate
		if (calibrated()) {
			seekTarget(az_t, el_t, _az_s, _el_s);
		} else {
			calibrate(_az_s, _el_s);
		}
		//< preserve sensor angles for next stopped iteration; only for calibration or re-calibration
		prevstop_az = _az_s;
		prevstop_el = _el_s;
	}
	/* Preserve sensor angles for next fast (UPD_PERIOD) iteration, to decide if motion has stopped.
	* This means we only respond to every other az, el command, but GPredict will 
	* keep sending commands as long as the gimbal is not pointing properly.
	*/
	prevfast_az = _az_s;
	prevfast_el = _el_s;
	if (gimbal->DEBUG_GIMBAL) {
		Serial.println("Saving prevfast_az & prevfast_el");
		Serial.print("prevfast_el, el_s: (");
		Serial.print(prevfast_el); Serial.print(", "); Serial.print(_el_s); Serial.println(")");
		Serial.print("prevfast_az, az_s: (");
		Serial.print(prevfast_az); Serial.print(", "); Serial.print(_az_s); Serial.println(")");
	}
}

/*! @brief run the next step of the initial scale calibration series.
*
* steps proceed using init_step up to N_INIT_STEPS
* @param az_s is current sensor azimuth angle in degrees
* @param el_s is current sensor elevation angle in degrees
*/
void Gimbal::calibrate(float &az_s, float &el_s)
{
	//< handy step ranges
	uint16_t _range0 = motor[0].max - motor[0].min;
	uint16_t _range1 = motor[1].max - motor[1].min;
	webpage->setUserMessage(F("Calibrating gimbal"));
	isCalibrating = true;
	//< init_step starts at 0; incremented each time function is called
	switch (init_step++) {

	case 0:
		if (gimbal->DEBUG_GIMBAL) {
			Serial.print(F("Init 0: Mot 0 Moves: "));
			Serial.println(motor[0].min + _range0 * (1 - CAL_FRAC) / 2, 0);
			Serial.print(F("Init 0: Mot 1 Moves: "));
			Serial.println(motor[1].min + _range1 * (1 - CAL_FRAC) / 2, 0);
		}
		//< move near min of each range. setMotorPosition() uses microseconds
		setMotorPosition(0, motor[0].min + _range0 * (1 - CAL_FRAC) / 2);
		delay(100); //< wait until motor 0 starts moving before starting the other motor
		setMotorPosition(1, motor[1].min + _range1 * (1 - CAL_FRAC) / 2);
		delay(500);
		break;

	case 1:
		//< move just motor 0 a subtantial distance
		if (gimbal->DEBUG_GIMBAL) {
			Serial.print(F("Init 1: Mot 0 starts at:\t"));
			Serial.print(az_s); Serial.print(F("\t")); Serial.print(el_s);
			Serial.print(F("\tMoves\t"));
			Serial.println(_range0 * CAL_FRAC, 0);
		}
		setMotorPosition(0, motor[0].pos + _range0 * CAL_FRAC);
		delay(500);
		break;

	case 2:
		//< calculate scale of motor 0
		motor[0].az_scale = _range0 * CAL_FRAC / azDist(prevstop_az, az_s);
		motor[0].el_scale = _range0 * CAL_FRAC / (el_s - prevstop_el);
		if (gimbal->DEBUG_GIMBAL) {
			Serial.print(F("Init 2: Mot 0 ended  at (az/el): ("));
			Serial.print(az_s, 1); Serial.print(F(", ")); Serial.print(el_s, 1);
			Serial.print(F(") us: "));
			Serial.print(_range0 * CAL_FRAC, 0);
			Serial.print(F("\tDelta us/Deg (az, el): ("));
			Serial.print(motor[0].az_scale, 2); Serial.print(F(", ")); Serial.print(motor[0].el_scale, 2);
			Serial.println(F(")"));
		}
		//< repeat procedure for motor 1
		setMotorPosition(1, motor[1].pos + _range1 * CAL_FRAC);
		delay(500);
		if (gimbal->DEBUG_GIMBAL) {
			Serial.print(F("Init 2: Mot 1 starts at (az, el): ("));
			Serial.print(az_s, 1); Serial.print(F(", ")); Serial.print(el_s, 1);
			Serial.print(F(") Moves ")); Serial.print(_range1 * CAL_FRAC, 0); Serial.println(F(" us"));
		}
		break;

	case 3:
		//< calculate scale of motor 1
		motor[1].az_scale = _range1 * CAL_FRAC / azDist(prevstop_az, az_s);
		motor[1].el_scale = _range1 * CAL_FRAC / (el_s - prevstop_el);
		if (gimbal->DEBUG_GIMBAL) {
			Serial.print(F("Init 3: Mot 1 ended  at (az, el): "));
			Serial.print(az_s, 1); Serial.print(F(", "));
			Serial.print(el_s, 1); Serial.print(F(") or  "));
			Serial.print(_range1 * CAL_FRAC);
			Serial.print(F(" us. Del us/Deg scale (az, el): ("));
			Serial.print(motor[1].az_scale, 2); Serial.print(F(", "));
			Serial.print(motor[1].el_scale, 2); Serial.println(F(")"));
		}
		//< select best motor for az
		best_azmotor = fabs(motor[0].az_scale) < fabs(motor[1].az_scale) ? 0 : 1;
		if (gimbal->DEBUG_GIMBAL) {
			Serial.print(F("Best Az motor: ")); Serial.print(best_azmotor);
			Serial.print(F("\tScale: ")); Serial.print(motor[best_azmotor].az_scale);
			Serial.print(F("\tEl motor: ")); Serial.print(!best_azmotor);
			Serial.print(F("\tScale: ")); Serial.println(motor[!best_azmotor].el_scale);
		}
		saveCalibration(); //< finished calibration, save to EEPROM
		isCalibrating = false;
		break;

	default:
		webpage->setUserMessage(F("BUG! Bogus init_step"));
		break;
	}
}

/*! @brief seek the target given the current stable az/el sensor values
* @param az_t is desired 'target' azimuth in degrees
* @param el_t is desired 'target' elevation in degrees
* @param az_s is current sensor azimuth in degrees
* @param el_s is current sensor elevation in degrees
*/
void Gimbal::seekTarget(float &az_t, float &el_t, float &az_s, float &el_s)
{
	//< find pointing error in each dimension as a move from sensor to target, in degrees
	float _az_err = azDist(az_s, az_t);
	float _el_err = el_t - el_s;

	/* If scale is wrong and pointing error is large, we won't end at az_t, el_t
	* but the next seekTarget command will get us closer
	*/
	//< correct each error using motor with most effect in that axis
	MotorInfo *azmip = &motor[best_azmotor];
	MotorInfo *elmip = &motor[!best_azmotor];
	if (gimbal->DEBUG_GIMBAL) {
		Serial.print(F("Seeking target at (az, el): ("));
		Serial.print(az_t, 1); Serial.print(F(", ")); Serial.print(el_t, 1); Serial.println(F(")"));
		Serial.print(F("curr Az pos: ")); Serial.print(az_s, 1);
		Serial.print(F(", us: ")); Serial.print(azmip->pos);
		Serial.print(F("\taz_error, deg: ")); Serial.print(_az_err, 1);
		Serial.print(F(", us: ")); Serial.println(_az_err * azmip->az_scale, 0);

		Serial.print(F("curr El pos: ")); Serial.print(el_s, 1);
		Serial.print(F(", us ")); Serial.print(elmip->pos);
		Serial.print(F("\tel_error, deg: ")); Serial.print(_el_err, 1); 
		Serial.print(F(", us: ")); Serial.println(_el_err * elmip->el_scale, 0);
	}
	// tweak scale if move was substantial and sanity check by believing only small changes
	reCal(az_s, el_s);
	// move each motor to reduce error, but if at Az limit then swing back to near opposite limit
	if (azmip->atmin) {
		setMotorPosition(best_azmotor, azmip->min + 0.9 * (azmip->max - azmip->min));
	} else if (azmip->atmax) {
		setMotorPosition(best_azmotor, azmip->min + 0.1 * (azmip->max - azmip->min));
	} else {
		setMotorPosition(best_azmotor, azmip->pos + _az_err * azmip->az_scale);
	}
	// set elevation motor (!best_azmotor)
	setMotorPosition(!best_azmotor, elmip->pos + _el_err * elmip->el_scale);
}

/*! @brief given two azimuth values, return path length going shortest direction
 */
float Gimbal::azDist(float &from, float &to)
{
	float _d = to - from;
	if (_d < -180) {
		_d += 360;
	}
	else if (_d > 180) {
		_d -= 360;
	}
	return (_d);
}

/** read the Gimbal calibration values and save into EEPROM.
 */
void Gimbal::saveCalibration()
{
	nv->best_az_motor = best_azmotor;
	nv->m0_azscale = motor[0].az_scale;
	nv->m0_elscale = motor[0].el_scale;
	nv->m1_azscale = motor[1].az_scale;
	nv->m1_elscale = motor[1].el_scale;
	nv->init_step = init_step;
	// save in EEPROM
	nv->put();
}

/*! @brief install previously stored calibration data from EEPROM if it looks valid.
 */
void Gimbal::installCalibration()
{
	nv->get();				   //< set from NV
	init_step = nv->init_step; //< indicates gimbal calibrated
	best_azmotor = nv->best_az_motor;
	motor[0].az_scale = nv->m0_azscale;
	motor[0].el_scale = nv->m0_elscale;
	motor[1].az_scale = nv->m1_azscale;
	motor[1].el_scale = nv->m1_elscale;
	//< sanity check on calibration scales
	if (fabs(motor[best_azmotor].az_scale > 50) || fabs(motor[!best_azmotor].el_scale > 50) 
			|| init_step != 4 || best_azmotor < 0 || best_azmotor > 1) {
		//< request new calibration
		init_step = 0;
	}
}

/*! @brief re-calibrate motor scales if motion is great enough and scale difference is small enough
*  during seekTarget move
*/
void Gimbal::reCal(float &az_s, float &el_s)
{
	MotorInfo *azmip = &motor[best_azmotor];
	MotorInfo *elmip = &motor[!best_azmotor];
	const float MIN_ANGLE = 30;	  //< min acceptable move
	const float MAX_CHANGE = 0.1; //< max fractional scale change
	float _az_move = azDist(prevstop_az, az_s);
	if (fabs(_az_move) >= MIN_ANGLE) {
		float _new_az_scale = azmip->del_pos / _az_move;
		if (fabs((_new_az_scale - azmip->az_scale) / azmip->az_scale) < MAX_CHANGE) {
			if (gimbal->DEBUG_GIMBAL) {
				Serial.print(F("New Az scale: "));
				Serial.print(azmip->az_scale); Serial.print(F("\t->\t")); Serial.println(_new_az_scale);
			}
			azmip->az_scale = _new_az_scale;
		}
	}
	float _el_move = el_s - prevstop_el;
	if (fabs(_el_move) >= MIN_ANGLE) {
		float _new_el_scale = elmip->del_pos / _el_move;
		if (fabs((_new_el_scale - elmip->el_scale) / elmip->el_scale) < MAX_CHANGE) {
			if (gimbal->DEBUG_GIMBAL) {
				Serial.print(F("New El scale: "));
				Serial.print(elmip->el_scale); Serial.print(F("\t->\t")); Serial.println(_new_el_scale);
			}
			elmip->el_scale = _new_el_scale;
		}
	}
}

/*! @brief issue raw motor command in microseconds pulse width, clamped at limit
* @param motn is the motor number to move
* @param newpos is the new position in microseconds
*/
void Gimbal::setMotorPosition(uint8_t motn, uint16_t newpos)
{
	if (motn >= NMOTORS || !gimbal_found) {
		return;
	}
	MotorInfo *mip = &motor[motn];
	mip->atmin = (newpos <= mip->min);
	if (mip->atmin) {
		newpos = mip->min;
	}
	mip->atmax = (newpos >= mip->max);
	if (mip->atmax) {
		newpos = mip->max;
	}
	mip->del_pos = (int)newpos - (int)mip->pos;
	mip->pos = newpos;
	pwm->setPWM(mip->servo_num, 0, mip->pos / US_PER_BIT);
}

/*! @brief send latest values to web page
* @param client is the calling WiFi client
* N.B. labels must match ids in web page
*/
void Gimbal::sendNewValues(WiFiClient client)
{
	if (!gimbal_found) {
		client.println(F("G_Status=Not found!"));
		return;
	}

	client.print(F("G_Mot1Pos="));
	client.println(motor[0].pos);
	client.print(F("G_Mot2Pos="));
	client.println(motor[1].pos);

	client.print(F("G_Mot1Max="));
	client.println(motor[0].max);
	client.print(F("G_Mot1Min="));
	client.println(motor[0].min);

	client.print(F("G_Mot1AzCal="));
	client.println(1 / motor[0].az_scale);
	client.print(F("G_Mot1ElCal="));
	client.println(1 / motor[0].el_scale);

	client.print(F("G_Mot2Min="));
	client.println(motor[1].min);
	client.print(F("G_Mot2Max="));
	client.println(motor[1].max);

	client.print(F("G_Mot2AzCal="));
	client.println(1 / motor[1].az_scale);
	client.print(F("G_Mot2ElCal="));
	client.println(1 / motor[1].el_scale);
	// HIGH means limit switch is applying 3v to OE input of PCA9685,
	// which is read by PCA9685OEPin. OE is normally pulled down or 0v
	//https://learn.adafruit.com/16-channel-pwm-servo-driver/pinouts
	bool pca9685_is_disabled = digitalRead(PCA9685OEPin);
	client.print(F("G_Status="));
	if (pca9685_is_disabled) {
		client.println(F("Gimbal fault!"));
	}
	else if (motor[0].atmin) {
		client.println(F("Servo 1 at Min!"));
	}
	else if (motor[0].atmax) {
		client.println(F("Servo 1 at Max!"));
	}
	else if (motor[1].atmin) {
		client.println(F("Servo 2 at Min!"));
	}
	else if (motor[1].atmax) {
		client.println(F("Servo 2 at Max!"));
	} else if (!calibrated()) {
		client.println(F("Uncalibrated!"));
	} else {
		client.println(F("Ok+"));
	}
}

/*! @brief process name = value pair
*
* @param name the web page id where value was entered
* @param value a value to operate on, if needed
* @return return true if Gimbal handles this 'name' command, false if Gimbal doesn't recognize it
*/

bool Gimbal::overrideValue(char *name, char *value)
{
	const __FlashStringHelper *nog = F("No gimbal!");

	if (!strcmp(name, "G_Mot1Pos")) {
		if (gimbal_found) {
			setMotorPosition(0, atoi(value));
		} else {
			webpage->setUserMessage(nog);
		}
		return (true);
	}
	if (!strcmp(name, "G_Mot1Min")) {
		if (gimbal_found) {
			nv->mot0min = motor[0].min = atoi(value);
			nv->put();
			webpage->setUserMessage(F("Servo 1 minimum saved in EEPROM+"));
		} else {
			webpage->setUserMessage(nog);
		}
		return (true);
	}
	if (!strcmp(name, "G_Mot1Max")) {
		if (gimbal_found) {
			nv->mot0max = motor[0].max = atoi(value);
			nv->put();
			webpage->setUserMessage(F("Servo 1 maximum saved in EEPROM+"));
		} else {
			webpage->setUserMessage(nog);
		}
		return (true);
	}
	if (!strcmp(name, "G_Mot2Pos")) {
		if (gimbal_found) {
			setMotorPosition(1, atoi(value));
		} else {
			webpage->setUserMessage(nog);
		}
		return (true);
	}
	if (!strcmp(name, "G_Mot2Min")) {
		if (gimbal_found) {
			nv->mot1min = motor[1].min = atoi(value);
			nv->put();
			webpage->setUserMessage(F("Servo 2 minimum saved in EEPROM+"));
		} else {
			webpage->setUserMessage(nog);
		}
		return (true);
	}
	if (!strcmp(name, "G_Mot2Max")) {
		if (gimbal_found) {
			nv->mot1max = motor[1].max = atoi(value);
			nv->put();
			webpage->setUserMessage(F("Servo 2 maximum saved in EEPROM+"));
		} else {
			webpage->setUserMessage(nog);
		}
		return (true);
	}
	if (!strcmp(name, "G_Save")) {
		if (gimbal_found) {
			if (sensor->connected()) {
				//< set init_step to 0 so we'll do calibration
				resetInitStep();
				//< have to do this for 4 steps, incrementing init_step each time
				//< we can use 'delay()' since this is called by pressing a button
				while (!calibrated()) {
					//< the values passed, "init_step * 5" are arbitrary
					moveToAzEl(init_step * 5, init_step * 5);
					delay(200);
				}
				delay(1000);
				moveToAzEl(G_HOME_AZ, G_HOME_EL);
				delay(200);
				//< may have to do this twice
				moveToAzEl(G_HOME_AZ, G_HOME_EL);
				webpage->setUserMessage(F("Gimbal calibrated+"));
			} else {
				webpage->setUserMessage(F("no Sensor!"));
			}
		} else {
			webpage->setUserMessage(nog);
		}
		return (true);
	}
	return (false);
}