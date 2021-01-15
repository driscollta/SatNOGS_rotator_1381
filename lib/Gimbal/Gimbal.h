/*!
* @brief Class to control two motors to track a target az and el using the Adafruit I2C interface
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

#ifndef _GIMBAL_H
#define _GIMBAL_H

#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>

#include "Sensor.h"

class Gimbal {

    private:
	const bool DEBUG_GIMBAL = false;

	// I2C servo interface
	Adafruit_PWMServoDriver *pwm;
	static const uint8_t I2C_ADDR = 0x40;	// I2C bus address of servo controller
	static const uint8_t SERVO_FREQ = 50;	// typical servo pulse frequency, Hz
	static constexpr float US_PER_BIT = (1e6/SERVO_FREQ/4096);	// usec per bit @ 12 bit resolution
	static const uint8_t MOT1_UNIT = 0;		// motor 1 I2C unit number
	static const uint8_t MOT2_UNIT = 1;		// motor 2 I2C unit number
	bool gimbal_found;						// whether PWM controller is present
	static constexpr float G_HOME_AZ = 0.0;	// gimbal az home position for calibration
	static constexpr float G_HOME_EL = 45.0; // gimbal el home position for calibration
	// motor info
	typedef struct {
	    float az_scale, el_scale;			// az and el scale: steps (del usec) per degree
	    uint16_t min, max;					// position limits, usec
	    uint16_t pos;						// last commanded position, usec
	    int16_t del_pos;					// change in pos since previous move
	    bool atmin, atmax;					// (would have been commanded to) limit
	    uint8_t servo_num;					// I2C bus address 0..15
	} MotorInfo;
	static const uint8_t NMOTORS = 2;		// not easily changed
	MotorInfo motor[NMOTORS];

	// search info
	// N.B.: max az physical motion must be < 180/CAL_FRAC
	static const uint16_t UPD_PERIOD = 500;		// ms between moveToAzEl updates
	static constexpr float MAX_SETTLE = 0.5;	// considered stopped, degs
	static const uint8_t N_INIT_STEPS = 4;		// number of init_steps
	static constexpr float CAL_FRAC = 0.333;	// fraction of full range to move for calibration
												// N.B.: max physical motion must be < 180/CAL_FRAC
	uint8_t init_step;							// initialization sequencing
	uint8_t best_azmotor;						// after cal, motor[] index with most effect in az
	uint32_t last_update;						// millis() time of last moveToAzEl
	float prevfast_az, prevfast_el;				// previous pointing position
	float prevstop_az, prevstop_el;				// previous stopped position for calibration
	
	void setMotorPosition (uint8_t motn, uint16_t newpos);
	void calibrate (float &az_s, float &el_s);
	void seekTarget (float& az_t, float& el_t, float& az_s, float& el_s);
	float azDist (float &from, float &to);
	void reCal(float& az_s, float& el_s);
	void installCalibration();
	void saveCalibration();
	
    public:
	boolean isCalibrating;
	Gimbal();
	void resetInitStep();
	void moveToAzEl (float az_t, float el_t);
	void sendNewValues (WiFiClient client);
	bool overrideValue (char *name, char *value);
	bool connected() { return (gimbal_found); };
	bool calibrated() { return (init_step >= N_INIT_STEPS); }
};

extern Gimbal *gimbal;

#endif // _GIMBAL_H
