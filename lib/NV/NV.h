/*!
* @brief Class to organize variables stored in EEPROM.
*
* The public class variables are mirrored in RAM, so change nv then call put(), or call get() then access 
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

#ifndef _NV_H
#define _NV_H

#include <EEPROM.h>

class NV {

    private:

        enum {
            MAGIC  = 0x5a5aa5a5,            //< used to validate the EEPROM stored values
            EEBASE = 0,                     //< address of the EEPROM storage area
            NBNO055CALBYTES = 22,           //< bytes needed for the BNO055 calibration
        };

    public:

        NV()
        {
            EEPROM.begin(sizeof(*this));    //< for ESP32, call .begin()
        }

        uint32_t magic;                     //< magic # read from EEPROM
        uint16_t mot0min, mot0max;          //< motor 0 minimum and maximum pulse durations
        uint16_t mot1min, mot1max;          //< motor 1 minimum and maximum pulse durations
        uint8_t BNO055cal[NBNO055CALBYTES]; //< BNO055 sensor calibration values
        float_t mag_decl;                   //< magnetic declination of user location
        float_t m0_azscale, m0_elscale;     //< motor 0 azimuth and elevation conversion from degrees to usec pulse width
        float_t m1_azscale, m1_elscale;     //< motor 1 azimuth and elevation conversion from degrees to usec pulse width
        uint8_t best_az_motor;              //< motor that has the most effect in azimuth direction either 0 or 1
        uint8_t init_step;                  //< step number that the calibration routine ran when this cal was saved. s.b. 4

        void get() {
            EEPROM.get (EEBASE, *this);
            if (magic != MAGIC) {               //< EEPROM was never written, or it was corrupted
                memset (this, 0, sizeof(*this));//< erase EEPROM values
                magic = MAGIC;
                EEPROM.put (EEBASE, *this);     //< save all NV values to EEPROM
                EEPROM.commit();
            }
        }

        void put() {
            EEPROM.put (EEBASE, *this);
            EEPROM.commit();
        }
};

extern NV *nv;

#endif // _NV_H