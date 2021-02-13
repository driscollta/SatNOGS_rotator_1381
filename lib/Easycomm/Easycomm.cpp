/*!
* @brief handle Serial port communication with rotctl on Raspberry Pi or PC
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

#include "Easycomm.h"
#include "Gimbal.h"
#include "NV.h"
#include "Webpage.h"
#include "Sensor.h"

char buffer[BUFFER_SIZE];

/*! @brief Read new commands on USB serial interface
*
* '\n' is new-line command terminator for positioning commands 
* commands like IP, GE are terminated with carriage return '\r'
*/
void Easycomm::easycomm_process() 
{
    float az_input, el_input;
    char incomingByte;
    static uint16_t bufferCnt = 0;
    //< Read from serial
    while (Serial.available() > 0) {
        incomingByte = Serial.read();

        if (incomingByte == '\n' || incomingByte == '\r') {
            //< process command

            buffer[bufferCnt] = 0;
            if (buffer[0] == 'A' && buffer[1] == 'Z' && buffer[2] ==  ' ' 
                    && buffer[3] == 'E' && buffer[4] == 'L') {
            //< read 'AZ EL ' and do reportPosition() right away
                reportPosition();
            } else if (buffer[0] == 'A' && buffer[1] == 'Z') {
            //< positioning command
                if (buffer[2] != ' ') {
                //< There was data after "AZ", get the absolute position in deg for azimuth
                //< There is a command "AZ EL" (p) that just asks for the current position.
                    if (readAzEl(&az_input, &el_input, buffer)){
                        gimbal->moveToAzEl(az_input, el_input);        
                    }
                    reportPosition();
                } else {
                    //< no az/el goto data, just report position to acknowledge command
                    reportPosition();
                }
            } else if (buffer[0] == 'S' && buffer[1] == 'A' 
                && buffer[2] == ' ' && buffer[3] == 'S' && buffer[4] == 'E') {
                //< SA SE: Stop Moving
                reportPosition();
            } else if (buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'S' 
                && buffer[3] == 'E' && buffer[4] == 'T') {
                //< Reset the rotator, go to home position
                reportPosition();
                ESP.restart();
            } else if (buffer[0] == 'P' && buffer[1] == 'A' 
                && buffer[2] == 'R' && buffer[3] == 'K') {
                //< Park the rotator
                gimbal->moveToAzEl(0.0, 0.0);
                reportPosition();
            } else if (buffer[0] == 'V' && buffer[1] == 'E') {
                //< Get the version of rotator controller
                Serial.print("VESatNOGS-v2.2\n RPRT 0\n");
            } else if (buffer[0] == 'I' && buffer[1] == 'P') {
                //< status command
                dealWithStatusCommand(buffer[2]);

            } else if (buffer[0] == 'G' && buffer[1] == 'S') {
                //< Get the status of rotator
                Serial.print("GS, 0\n RPRT 0\n");

            } else if (buffer[0] == 'G' && buffer[1] == 'E') {
                //< Get the error of rotator
                Serial.print("GE, 0\n RPRT 0\n");
            }
            //< After dealing with command, reset the buffer counter & clean the serial buffer
            bufferCnt = 0;
        } else {
            //< Did not get a command terminator, add incoming byte to buffer
            buffer[bufferCnt] = incomingByte;
            bufferCnt++;
            //< don't overflow buffer[]
            if (bufferCnt >= BUFFER_SIZE) {
                bufferCnt = 0;
            }
        }
        //< continue until there is no more data in the Serial buffer
    }
}

/*! @brief parse the rotctl command buffer to extract the desired az, el position
* @param az_input the azimuth value in the command
* @param el_input the elevation value in the command
* @param buffer the message received over the Serial port
* @return true if we found valid az, el positioning numbers
*/
bool Easycomm::readAzEl(float *az_input, float *el_input, char buffer[])
{
    char _buffcpy[BUFFER_SIZE];
    memset(_buffcpy, '\0', sizeof(_buffcpy));
    strcpy(_buffcpy, buffer);
    char *token;

    //< get the first token
    token = strtok(_buffcpy, " ");
    if (isNumber(token + 2)) {
        *az_input = atof(token + 2);
    }
    //< Get the absolute position in deg for elevation
    token = strtok(NULL, " ");
    //< check that the first two chars are "EL" and copy to "data", skipping the "EL"
    if (token[0] == 'E' && token[1] == 'L') {
        if (isNumber(token + 2)) {
            *el_input = atof(token + 2);
            return true;
        }
    }
    return false;
}

/*! @brief utility to report whether the input[] is a numeric value
* @param input the char[] to inspect for numeric characters
* @return true if all characters in input are numeric
*/
bool Easycomm::isNumber(char input[]) 
{
  for (uint16_t i = 0; input[i] != '\0'; i++) {
    if (isalpha(input[i]))
      return false;
    }
  return true;
}

/*! @brief send back the current sensor reading formatted for rotctl
*/
void Easycomm::reportPosition() {
    Serial.print("AZ");Serial.print(sensor->getSensorAz(), 1);
    Serial.print(" EL"); Serial.print(sensor->getSensorEl(), 1);
    Serial.println("\n");   
}

/*! @brief generate reply to status rotctl commands starting with 'IP'
*  most are not relevant
*/
void Easycomm::dealWithStatusCommand(char statusNumber) 
{
    String statusString;
    
    switch (statusNumber) {
    case '0':
        //< Get the inside temperature
        statusString = String(sensor->getTempC(), DEC);
        break;
    case '1':
        //< Get the status of end-stop, azimuth
        statusString = "0";
        break;
    case '2':
        //< Get the status of end-stop, elevation
        statusString = "0";
        break;
    case '3':
        //< Get the current position of azimuth in deg
        statusString = String(sensor->getSensorAz(), 1);
      break;
    case '4':
      //< Get the current position of elevation in deg
      statusString = String(sensor->getSensorEl(), 1);
        break;
    case '5':
      //< Get the load of azimuth, in range of 0-1023
      statusString = "1";
      break;
    case '6':
      //< Get the load of elevation, in range of 0-1023
      statusString = "1";
      break;
    case '7':
      //< Get the speed of azimuth in deg/s
      statusString = "1";
      break;
    case '8':
      //< Get the speed of elevation in deg/s
      statusString = "1";
      break;
    default:
      statusString = "";
      break;
  }
  Serial.print(String("IP") + statusNumber + String(", ") + statusString + String("\nRPRT 0\n"));
}

/*! @brief send latest web values, only report the last rotctl command in the 'buffer'
*
* @param client a reference to the WiFiClient
* N.B. must match id's in main web page
*/
void Easycomm::sendNewValues(WiFiClient client)
{
    client.print(F("rotctl_message="));
    client.println(buffer);
}