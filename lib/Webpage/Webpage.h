/*!
* @brief Class to handle the port-server display and interactions
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


#ifndef _WEBPAGE_H
#define _WEBPAGE_H
#include <ctype.h>
#include <WiFi.h>

#define WIFI_SSID "tigger"				//< WiFi SSID. Change this value
#define WIFI_PASS "Belridge#117"		//< WiFi password. Change this value
#define MAX_UNSIGNED_LONG 4294967295ULL	//< maximum value of unsigned long. correct for roll-over
#define TIMEOUT_WIFI 10000				//< time, msec, to wait for WiFi to connect

class Webpage
{

    public:

	Webpage();
	void checkEthernet();
	void setUserMessage (const __FlashStringHelper *ifsh);
	void setUserMessage (const __FlashStringHelper *ifsh, const char *msg, char state);

    private:

	WiFiServer *httpServer;
	const __FlashStringHelper *user_message_F;
	char user_message_s[100];
	const bool DEBUG_WEBPAGE = true;
    bool is_timed_out(uint32_t startTime_ms, uint32_t time_out_time);
	void overrideValue (WiFiClient client);
	void printHTMLStyle (WiFiClient client);
	void printHTMLSensorTable(WiFiClient client);
	void printHTMLScripts (WiFiClient client);
	void printHTMLGimbalTable (WiFiClient client);
	void printHTMLTopTable (WiFiClient client);
	char readNextClientChar (WiFiClient client, uint32_t *to);
	void reboot();
	void sendMainPage (WiFiClient client);
	void sendNewValues (WiFiClient client);
	void sendPlainHeader (WiFiClient client);
	void sendHTMLHeader (WiFiClient client);
	void sendEmptyResponse (WiFiClient client);
	void send404Page (WiFiClient client);

};

extern Webpage *webpage;

#endif // _WEBPAGE_H
