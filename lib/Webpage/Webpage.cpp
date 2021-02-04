/*! @brief Class to handle the port-server display and interactions
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

#include <Wire.h>
#include <ctype.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>
#include <pgmspace.h>

#include "NV.h"
#include "Webpage.h"
#include "Sensor.h"
#include "Gimbal.h"
#include "Easycomm.h"

uint8_t wifi_time_out; //< time between checks for active WiFi

/*! constructor
 */
Webpage::Webpage()
{
    wifi_time_out = 0;
    //< init user message mechanism
	user_message_F = F("Hello+");		// <page welcome message
    memset (user_message_s, 0, sizeof(user_message_s));

    //< start the Server on port 80
	httpServer = new WiFiServer(80);
    delay(1000);
	WiFi.begin (WIFI_SSID, WIFI_PASS);
    delay(1000);
	httpServer->begin();
}

/*! @brief record a brief F() message to inform the user
*
* Message will be sent on the next sendNewValues() sweep
*/
void Webpage::setUserMessage (const __FlashStringHelper *ifsh)
{
	user_message_F = ifsh;
	user_message_s[0] = '\0';
}

/*! @brief record a brief message to inform the user
*
* Message will be sent on the next sendNewValues() sweep.
* The message consists of an F() string, then a stack string, then a trailing character, typically
*  '!' to indicate an alarm, '+' to indicate good progress, or '\0' for no effect.
*/
void Webpage::setUserMessage (const __FlashStringHelper *ifsh, const char *msg, char state)
{
	user_message_F = ifsh;
	strncpy (user_message_s, msg, sizeof(user_message_s)-2);	// room for state and EOS
	user_message_s[strlen(user_message_s)] = state;
}

/*! @brief call this occasionally to check for Ethernet activity
 */
void Webpage::checkEthernet()
{
    //< check WiFi if not connected and waited > 5 seconds
    if (WiFi.status() != WL_CONNECTED && (wifi_time_out++ > 25)) {
        WiFi.begin (WIFI_SSID, WIFI_PASS);
        wifi_time_out = 0;
    }
	//< now check our page
	WiFiClient client = httpServer->available();
	if (!client) {
	    return;
    }
	uint32_t to = millis();	//< init timeout
	char firstline[128];	//< first line
	unsigned fll = 0;		//< firstline length
	bool eoh = false;		//< set when see end of header
	bool fldone = false;	//< set when finished collecting first line
	char c, prevc = 0;		//< new and previous character read from client

	//< read header, collecting first line and discarding rest until find blank line
	while (!eoh && (c = readNextClientChar (client, &to))) {
	    if (c == '\n') {
		    if (!fldone) {
		        firstline[fll] = '\0';
		        fldone = true;
		    }
		    if (prevc == '\n') {
		        eoh = true;
            }
	    } else if (!fldone && fll < sizeof(firstline)-1) {
		    firstline[fll++] = c;
	    }
	    prevc = c;
	}
	if (c == 0) {
	    client.stop();
	    return;
	}

	//< client socket is now at first character after blank line
	//< replace trailing ?time cache-buster with blank
	char *q = strrchr (firstline, '?');
	if (q) {
	    *q = ' ';
    }
	//< what we do next depends on first line
	if (strstr (firstline, "GET / ")) {
	    sendMainPage (client);
	} else if (strstr (firstline, "GET /getvalues.txt ")) {
	    sendNewValues (client);
	} else if (strstr (firstline, "POST / ")) {
	    overrideValue (client);
	    sendEmptyResponse (client);
	} else if (strstr (firstline, "POST /reboot ")) {
	    sendEmptyResponse (client);
	    reboot();
	} else {
	    send404Page (client);
	}
	//< finished
	client.stop();
}

/*! @brief read next character, return 0 if client disconnects or times out.
*
* '\r' is discarded completely.
* @param client a reference to the calling WiFi client
* @param to time when we started reading
*/
char Webpage::readNextClientChar (WiFiClient client, uint32_t *to)
{
	static const int timeout = 1000;		//< client socket timeout, ms
	while (client.connected()) {
	    if (millis() > *to + timeout ) {
		    return (0);
	    }
	    if (!client.available()){
		    continue;
        }
        char c = client.read();
	    *to = millis();
	    if (c == '\r'){
		    continue;
        }
        return (c);
	}
	return (0);
}

/*! @brief operator has entered manually a value to be overridden.
*
* client is at beginning of NAME=VALUE line, parse and send to each subsystem
* N.B. a few are treated specially.
* @param client a reference to the calling WiFi client
*/
void Webpage::overrideValue (WiFiClient client)
{
	char c, buf[200];			//< must be at least enough for a typical input
	uint8_t nbuf = 0;			//< type must be large enough to count to sizeof(buf)

	//< read next line into buf
	uint32_t to = millis();		//< initalize timeout
	while ((c = readNextClientChar (client, &to)) != 0) {
	    if (c == '\n') {
		    buf[nbuf++] = '\0';
		    break;
	    } else if (nbuf < sizeof(buf)-1) {
		    buf[nbuf++] = c;
        }
	}
	if (c == 0) {
	    return;		//< bogus; let caller close
    }
	//< break at = into name, value
	char *valu = strchr (buf, '=');
	if (!valu) {
	    return;		//< bogus; let caller close
    }
    *valu++ = '\0';	//< replace = with 0 then valu starts at next char
	//< now buf is NAME and valu is VALUE
    if (strcmp (buf, "Decl") == 0) {
		nv->mag_decl = (float) atof(valu);
	    nv->put();
	    setUserMessage (F("Saved new magnetic declination+"));
	} else {
    //< not ours, give to each other subsystem in turn until one accepts
	    if (!sensor->overrideValue (buf, valu)
			    && !gimbal->overrideValue (buf, valu)) {
		    setUserMessage (F("Bug: unknown override -- see Serial Monitor!"));
        }
    }
}

/*! @brief inform each subsystem to send its latest values, including ours
* @param client a reference to the calling WiFi client
 */
void Webpage::sendNewValues (WiFiClient client)
{
    sendPlainHeader(client);
    // send user message
    client.print ("op_message=");
    if (user_message_F != NULL){
        client.println(user_message_F);
    }
    if (user_message_s[0]){
        client.println(user_message_s);
    }
    client.print ("Decl=");
	client.println(nv->mag_decl);
    client.print ("SS_wifi=");
    client.println(WiFi.RSSI());
    sensor->sendNewValues(client);
    gimbal->sendNewValues(client);
    easycomm->sendNewValues(client);
}

/*! @brief print the HTML style section of main page
* @param client a reference to the calling WiFi client
*/

void Webpage::printHTMLStyle (WiFiClient client)
{
    client.print(F(
        "    <style> \r\n\r\n"
        "        body { \r\n"
        "            background-color:#888; \r\n"
        "            font-family:sans-serif; \r\n"
        "            font-size:13px; \r\n"
        "        } \r\n"
        "        table { \r\n"
        "            border-collapse: collapse; \r\n"
        "            border: 3px solid; \r\n"
        "            border-color: #0036CC; \r\n"
        "            background-color:#F8F8F8; \r\n"
        "            float:left; \r\n"
        "        } \r\n"
        "        th { \r\n"
        "            padding: 6px; \r\n"
        "            border: 1px solid; \r\n"
        "            border-color: #0036CC; \r\n"
        "        } \r\n"
        "        .even-row { \r\n"
        "            background-color:#F8F8F8; \r\n"
        "        } \r\n"
        "        .odd-row { \r\n"
        "            background-color:#D8D8D8; \r\n"
        "        } \r\n"
        "        #title-row { \r\n"
        "            text-align: center; \r\n"
        "            padding: 2px; \r\n"
        "            border-bottom: 6px double; \r\n"
        "            border-color: #0036CC; \r\n"
        "        } \r\n"
        "        #title-label { \r\n"
        "            font-size: 18px; \r\n"
        "            font-weight: bold; \r\n"
        "            color: #0066CC; \r\n"
        "        } \r\n"
    ));
    client.print (F(
        "        #op_message { \r\n"
        "            font-size:16px; \r\n"
        "            display: block; \r\n"
        "            padding: 10px; \r\n"
        "        } \r\n"
        "        td { \r\n"
        "            padding: 6px; \r\n"
        "            border: 1px solid; \r\n"
        "            border-color: #0066CC; \r\n"
        "        } \r\n"
        "        .major-section { \r\n"
        "            border-top: 6px double; \r\n"
        "            border-color: #0036CC; \r\n"
        "        } \r\n"
        "        .minor-section { \r\n"
        "            border-top: 4px double; \r\n"
        "            border-color: #0036CC; \r\n"
        "        } \r\n"
        "        .override { \r\n"
        "            background-color:#FFF; \r\n"
        "            padding: 0px; \r\n"
        "            font-family:monospace; \r\n"
        "            resize:none; \r\n"
        "            font-size:inherit; \r\n"
        "            width:7em; \r\n"
        "        } \r\n"
        "        .group-head { \r\n"
        "            text-align:center; \r\n"
        "            vertical-align:top; \r\n"
        "            border-right: 4px double; \r\n"
        "            border-color: #0036CC; \r\n"
        "        } \r\n"
        "        .datum-label { \r\n"
        "            text-align:left; \r\n"
        "            vertical-align:top; \r\n"
        "            color:black; \r\n"
        "        } \r\n"
        "        .datum { \r\n"
        "            font-family:monospace; \r\n"
        "            text-align:right; \r\n"
        "            color:black \r\n"
        "        } \r\n"
        "        #tracking { \r\n"
        "            font-size: 14px; \r\n"
        "            font-weight: bold; \r\n"
        "        } \r\n"
        "    </style> \r\n"
        " \r\n"
    ));
}
/*! @brief print the Scripts section of the HTML page header
*
* @param client a reference to the calling WiFi client
*
*/
void Webpage::printHTMLScripts (WiFiClient client)
{        
    client.print (F(            
        "    <script> \r\n"
        " \r\n"
        "        // handy shortcut \r\n"
        "        function byId (id) { \r\n"
        "            return document.getElementById(id); \r\n"
        "        } \r\n"
        " \r\n"
        "        // called once after DOM is loaded \r\n"
        "        window.onload = function() { \r\n"
        "            queryNewValues(); \r\n"
        "        } \r\n"
        " \r\n"
        "        // handy function that modifies a URL to be unique so it voids the cache \r\n"
        "        function UniqURL (url) { \r\n"
        "            return (url + '?' + (new Date()).getTime()); \r\n"
        "        } \r\n"
        " \r\n"
        "        // handy function to POST a name=value pair \r\n"
        "        function POSTNV (name, value) { \r\n"
        "            var xhr = new XMLHttpRequest(); \r\n"
        "            xhr.open('POST', UniqURL('/'), true); \r\n"
        "            xhr.send(name + '=' + String(value) + '\\r\\n'); \r\n"
        "        } \r\n"
    ));
    client.print (F(
        "        // send new value in response to operator typing an override value. \r\n"
        "        function onOvd() { \r\n"
        "            var event = this.event; \r\n"
        "            if (event.keyCode == 13) { \r\n"
        "                var oid = event.target.id; \r\n"
        "                var nam = oid.replace ('_Ovd', ''); \r\n"
        "                var vid = byId(nam); \r\n"
        "                if (vid) { \r\n"
        "                    var val = event.target.value.trim(); \r\n"
        "                    POSTNV (nam, val); \r\n"
        "                } \r\n"
        "            } \r\n"
        "        } \r\n"
        "        // called to perform Gimbal calibration \r\n"
        "        function onGSave() { \r\n"
        "            POSTNV ('G_Save', 'true'); \r\n"
        "        } \r\n"
        " \r\n"        
        "        // called to save Sensor calibration to EEPROM \r\n"
        "        function onSSSave() { \r\n"
        "            POSTNV ('SS_Save', 'true'); \r\n"
        "        } \r\n"
        " \r\n"
        "        // called to upload a new magnetic declination,"
        "       // either with Set (k==0) or by typing Enter (k==1) \r\n"
        "        function onDecl(k) { \r\n"
        "            if (k && this.event.keyCode != 13) \r\n"
        "                return;        // wait for Enter \r\n"
        "            var decl = byId ('Decl').value.trim(); \r\n"
        "            POSTNV ('Decl', decl); \r\n"
        "        } \r\n"
        " \r\n"
    ));
    client.print (F(
        "        // called to display the current magnetic declination."  
        "        // N.B. leave text alone if it or Set currently has focus \r\n"
        "        function setNewDecl(decl) { \r\n"
        "            var decl_text = byId('Decl'); \r\n"
        "            var decl_set  = byId('Decl-set'); \r\n"
        "            var focus = document.activeElement; \r\n"
        "            if (focus != decl_text && focus != decl_set) \r\n"
        "                decl_text.value = decl; \r\n"
        "        } \r\n"
        " \r\n"
        "        // called to set visibility of SS_Save \r\n"
        "        function setSSSave (whether) { \r\n"
        "            var sid = byId ('SS_Save'); \r\n"
        "            sid.style.visibility = (whether == 'true') ? 'visible' : 'hidden'; \r\n"
        "        } \r\n"
        " \r\n"
        "        // send command to reboot the ESP32 then reload our page after a short while  \r\n"
        "        function onReboot() { \r\n"
        "            if (confirm('Are you sure you want to reboot the ESP32?')) { \r\n"
        " \r\n"
        "                var xhr = new XMLHttpRequest(); \r\n"
        "                xhr.open ('POST', UniqURL('/reboot'), true); \r\n"
        "                xhr.send (); \r\n"
        " \r\n"
        "                byId ('op_message').style.color = 'red'; \r\n"
        " \r\n"
        "                function reloadMessage (n) { \r\n"
        "                    var msg = 'This page will reload in ' + n + ' second' + ((n == 1) ? '' : 's'); \r\n"
        "                    byId ('op_message').innerHTML = msg; \r\n"
        "                    if (n == 0) \r\n"
        "                        location.reload(); \r\n"
        "                    else \r\n"
        "                        setTimeout (function() {reloadMessage(n-1);}, 1000); \r\n"
        "                } \r\n"
        "                reloadMessage(10); \r\n"
        "            } \r\n"
        "        } \r\n"
        " \r\n"
    ));
    client.print (F(
        "       // query for new values forever \r\n"
        "       function queryNewValues() { \r\n"
        "           var xhr = new XMLHttpRequest(); \r\n"
        "           xhr.onreadystatechange = function() { \r\n"
        "               if (xhr.readyState==4 && xhr.status==200) { \r\n"
        "                   var lines = xhr.responseText.replace(/\\r/g,'').split('\\n'); \r\n"
        "                   for (var i = 0; i < lines.length; i++) { \r\n"
        "                       console.log('getvalues line ' + i + ': ' + lines[i]); \r\n"
        "                       var nv = lines[i].trim().split('='); \r\n"
        "                       if (nv.length != 2) \r\n"
        "                           continue; \r\n"
        "                       var id = byId (nv[0]); \r\n"
        "                       if (nv[0] == 'SS_Save') { \r\n"
        "                           setSSSave(nv[1]); \r\n"
        "                       } else if (nv[0] == 'Decl') { \r\n"
        "                           setNewDecl(nv[1]); \r\n"
        "                       } else { \r\n"
        "                           var l = nv[1].length; \r\n"
        "                           if (nv[1].substr(l-1) == '!') { \r\n"
        "                               id.innerHTML = nv[1].substr(0,l-1); \r\n"
        "                               id.style.color = 'red'; \r\n"
        "                           } else if (nv[1].substr(l-1) == '+') { \r\n"
        "                               id.innerHTML = nv[1].substr(0,l-1); \r\n"
        "                               id.style.color = '#297'; \r\n"
        "                           } else { \r\n"
        "                               // normal \r\n"
        "                               id.innerHTML = nv[1]; \r\n"
        "                               id.style.color = 'black'; \r\n"
        "                           } \r\n"
        "                       } \r\n"
        "                   } \r\n"
        " \r\n"
        "                   // repeat after a short breather \r\n"
        "                   setTimeout (queryNewValues, 750); \r\n"
        "               } \r\n"
        "           } \r\n"
        "           xhr.open('GET', UniqURL('/getvalues.txt'), true); \r\n"
        "           xhr.send(); \r\n"
        "       } \r\n"
        " \r\n"
    ));
    client.print (F(
        "    </script>  \r\n"
    ));
}

/*! @brief print the top table section of the HTML page
*
* @param client a reference to the calling WiFi client
*
*/
void Webpage::printHTMLTopTable(WiFiClient client)
{
    client.print (F(
        "   <table> \r\n"
        "       <tr> \r\n"
        "           <td id='title-row' colspan='7' > \r\n"
        "               <table style='border:none;' width='100%'> \r\n"
        "                   <tr> \r\n"
        "                       <td width='25%' style='text-align:left; border:none' > \r\n"
        "                           Magnetic Declination: \r\n"
        "                           <input id='Decl' type='text' onkeypress='onDecl(1)'  class='override' > </input> \r\n"
        "                           <button id='Decl-set' onclick='onDecl(0)'>Set</button> \r\n"
        "                       </td> \r\n"
        "                       <td width='50%' style='border:none' > \r\n"
        "                           <label id='title-label' title='Version 20200527' >Gimbal Diagnostics</label> \r\n"
        "                       </td> \r\n"
        "                       <td width='25%' style='text-align:right; border:none' > \r\n"
        "                           <button id='reboot_b' onclick='onReboot()'> Reboot ESP32 </button> \r\n"
        "                           <br> \r\n"
        "                       </td> \r\n"
        "                   </tr> \r\n"
        "                   <tr> \r\n"
        "                       <td colspan='3' width='100%' style='text-align:center; border:none'> \r\n"
        "                           <label id='rotctl_message' > Hello </label> \r\n"
        "                       </td> \r\n"
        "                   </tr> \r\n"
        "               </table> \r\n"
        "           </td> \r\n"
        "       </tr> \r\n"
        " \r\n"
    ));
}

/*! @brief print the Sensor table section of the HTML page
*
* @param client a reference to the calling WiFi client
*
*/

void Webpage::printHTMLSensorTable (WiFiClient client)
{
    client.print (F(
        "   <tr> \r\n"
        "   <td colspan='7' style='text-align:left; border: none; ' > \r\n"
        "       <table> \r\n"
        "           <tr> \r\n"
        "               <td></td> \r\n"
        "               <th colspan='3' scope='col'>Measurement</th> \r\n"
        "               <th colspan='2' scope='col'>Cal Status 0..3</th> \r\n"
        "               <th colspan='2' scope='col'>Self-test</th> \r\n"
        "           </tr>"
        " \r\n"
        "           <tr class='minor-section even-row' > \r\n"
        "               <th rowspan='4' class='group-head' > \r\n"
        "                       Spatial sensor \r\n"
        "                   <br> \r\n"
        "                   <label id='SS_Status'></label> \r\n"
        "                   <br> \r\n"
        "                   <button id='SS_Save' onclick='onSSSave()' > Save Cal </button> \r\n"
        "               </th> \r\n"
        " \r\n"
        "               <td class='datum-label' > Azimuth, &deg; E of N </td> \r\n"
        "               <td id='SS_Az' class='datum' width = 50 > </td> \r\n"
        "               <td width = 10></td> \r\n"
        "               <td class='datum-label' > System </td> \r\n"
        "               <td id='SS_SCal' class='datum' width = 20 >-</td> \r\n"
        "               <td id='SS_STSStatus' class='datum' >----</td> \r\n"
        "               <td width = 10></td> \r\n"
        "           </tr> \r\n"
        "           <tr class='odd-row' > \r\n"
        "               <td class='datum-label' > Elevation, &deg; Up </td> \r\n"
    ));
    client.print (F(
        "               <td id='SS_El' class='datum'  width = 50> </td> \r\n"
        "               <td width = 10></td> \r\n"
        "               <td class='datum-label' > Gyro </td> \r\n"
        "               <td id='SS_GCal' class='datum' width = 20 >-</td> \r\n"
        "               <td id='SS_STGStatus' class='datum' >----</td> \r\n"
        "               <td width = 10></td> \r\n"
        "           </tr> \r\n"
        "           <tr class='even-row' > \r\n"
        "               <td class='datum-label' > Temperature, &deg;C </td> \r\n"
        "               <td id='SS_Temp' class='datum' width = 50>-</td> \r\n"
        "               <td width = 10></td> \r\n"
        "               <td class='datum-label' > Magnetometer </td> \r\n"
        "               <td id='SS_MCal' class='datum' width = 20 >-</td> \r\n"
        "               <td id='SS_STMStatus' class='datum' >----</td> \r\n"
        "               <td width = 10></td> \r\n"
        "          </tr> \r\n"
        "          <tr class='odd-row' > \r\n"
        "               <td class='datum-label' > WiFi signal RSSI (dBm) </td> \r\n"
        "               <td id='SS_wifi' class='datum' width = 50> </td> \r\n"
        "               <td width = 10></td> \r\n"
        "               <td class='datum-label' > Accelerometer </td> \r\n"
        "               <td id='SS_ACal' class='datum' width = 20 >-</td> \r\n"
        "               <td id='SS_STAStatus' class='datum' >----</td> \r\n"
        "               <td width = 10></td> \r\n"
        "           </tr> \r\n"
        "           <tr class='even-row' > \r\n"
        "               <th rowspan='1' class='group-head' > \r\n"
        "               </th> \r\n"
        "               <td colspan='5' style='text-align:center; border:none'> \r\n"
        "                   <label id='op_message' > Hello </label> \r\n"
        "               </td> \r\n"
        "           </tr> \r\n"
        "       </table> \r\n"
        "       </td> \r\n"
        "   </tr> \r\n"
        " \r\n"
    ));
}

/*! @brief print the Gimbal table section of the HTML page
*
* @param client a reference to the calling WiFi client
*
*/
void Webpage::printHTMLGimbalTable (WiFiClient client)
{
    client.print(F(
        "   <!-- N.B. beware that some ID's are used in a match in onOvd() --> \r\n"
        "   <tr> \r\n"
        "   <td colspan='7' style='text-align:left; border: none; ' > \r\n"
        "      <table> \r\n"
        "          <tr>"
        "             <td></td> \r\n"
        "              <th colspan='2' scope='col'>Servo1</th> \r\n"
        "              <th colspan='1' scope='col'>override</th> \r\n"
        "              <th colspan='2' scope='col'>Servo2</th> \r\n"
        "              <th colspan='1' scope='col'>override</th> \r\n"
        "          </tr>"
        " \r\n"
        "           <tr class='minor-section even-row ' > \r\n"
        "               <th rowspan='3' class='group-head' > \r\n"
        "                      Gimbal \r\n"
        "                  <br> \r\n"
        "                  <label id='G_Status'></label> \r\n"
        "                 <br> \r\n"
        "                  <button id='G_Save' onclick='onGSave()' > Home </button> \r\n"
        "               </th> \r\n"
        " \r\n"
        "               <td class='datum-label' > pulse length, &micro;s </td> \r\n"
        "               <td id='G_Mot1Pos' class='datum' width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "                   <input id='G_Mot1Pos_Ovd' type='number' onkeypress='onOvd()' class='override' > \r\n"
        "                   </input> \r\n"
        "               </td> \r\n"
        " \r\n"
        "               <td class='datum-label' > pulse length, &micro;s </td> \r\n"
        "               <td id='G_Mot2Pos' class='datum'  width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "                   <input id='G_Mot2Pos_Ovd' type='number' onkeypress='onOvd()' class='override' > \r\n"
        "                   </input> \r\n"
        "               </td> \r\n"
        "           </tr> \r\n"
        "           <tr class='odd-row' > \r\n"
        "               <td class='datum-label' > minimum pulse </td> \r\n"
        "               <td id='G_Mot1Min' class='datum'  width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "                   <input id='G_Mot1Min_Ovd' type='number' onkeypress='onOvd()' class='override' > \r\n"
        "                   </input> \r\n"
        "               </td> \r\n"));
    client.print(F(
        "               <td class='datum-label' > minimum pulse </td> \r\n"
        "               <td id='G_Mot2Min' class='datum'  width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "                   <input id='G_Mot2Min_Ovd' type='number' onkeypress='onOvd()' class='override' > \r\n"
        "                   </input> \r\n"
        "               </td> \r\n"
        "           </tr> \r\n"
        "           <tr class='even-row' > \r\n"
        "               <td class='datum-label' > maximum pulse </td> \r\n"
        "               <td id='G_Mot1Max' class='datum'  width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "                   <input id='G_Mot1Max_Ovd' type='number' onkeypress='onOvd()' class='override' > \r\n"
        "                   </input> \r\n"
        "               </td> \r\n"
        "               <td class='datum-label' > maximum pulse </td> \r\n"
        "               <td id='G_Mot2Max' class='datum'  width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "                   <input id='G_Mot2Max_Ovd' type='number' onkeypress='onOvd()' class='override' > \r\n"
        "                   </input> \r\n"
        "               </td> \r\n"
        "           </tr> \r\n"
        "           <tr class='odd-row' > \r\n"
        "               <td> \r\n"
        "               </td> \r\n"
        "               <td class='datum-label' > az calibration, &deg;/&micro;s </td> \r\n"
        "               <td id='G_Mot1AzCal' class='datum'  width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "               </td> \r\n"
        "               <td class='datum-label' > az calibration, &deg;/&micro;s </td> \r\n"
        "               <td id='G_Mot2AzCal' class='datum'  width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "               </td> \r\n"
        "           </tr> \r\n"
        "           <tr class='even-row' > \r\n"
        "               <td> \r\n"
        "               </td> \r\n"
        "               <td class='datum-label' > el calibration, &deg;/&micro;s </td> \r\n"
        "               <td id='G_Mot1ElCal' class='datum'  width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "               </td> \r\n"
        "               <td class='datum-label' > el calibration, &deg;/&micro;s </td> \r\n"
        "               <td id='G_Mot2ElCal' class='datum'  width = 40 > ---- </td> \r\n"
        "               <td> \r\n"
        "               </td> \r\n"
        "           </tr> \r\n"
        " \r\n"
        "       </table> \r\n"
        "   </td> \r\n"
        " </tr> \r\n"
        " </table> \r\n"));
}

/*! @brief send the main page
*
* in turn it will send us commands using XMLHttpRequest
*
* @param client a reference to the calling WiFi client
*
 */
void Webpage::sendMainPage (WiFiClient client)
{
    client.print (F(
        "<!DOCTYPE html> \r\n"
        "<html> \r\n"
        "<head> \r\n"
        "    <meta http-equiv='Content-Type' content='text/html; charset=UTF-8' /> \r\n"
        " \r\n"
    ));
    printHTMLStyle(client);
    printHTMLScripts(client);
    client.print (F("</head> \r\n<body> \r\n"));
    printHTMLTopTable (client);
    printHTMLSensorTable (client);
    printHTMLGimbalTable (client);
    client.print (F("</body> \r\n</html> \r\n"));
}

/*! @brief send HTTP header for plain text content
*
* @param client a reference to the calling WiFi client
*
*/
void Webpage::sendPlainHeader (WiFiClient client)
{
	client.print (F(
	    "HTTP/1.1 200 OK \r\n"
	    "Content-Type: text/plain \r\n"
	    "Connection: close \r\n"
	    "\r\n"
	));
}

/*! @brief send HTTP header for html content
*
* @param client a reference to the calling WiFi client
*
*/
void Webpage::sendHTMLHeader (WiFiClient client)
{
	client.print (F(
	    "HTTP/1.1 200 OK \r\n"
	    "Content-Type: text/html \r\n"
	    "Connection: close \r\n"
	    "\r\n"
	));
}

/*! @brief send empty response
*
* @param client a reference to the calling WiFi client
*
*/
void Webpage::sendEmptyResponse (WiFiClient client)
{
	client.print (F(
	    "HTTP/1.1 200 OK \r\n"
	    "Content-Type: text/html \r\n"
	    "Connection: close \r\n"
	    "Content-Length: 0 \r\n"
	    "\r\n"
	));
}

/*! @brief send back error 404 when requested page not found.
*
*  N.B. important for chrome otherwise it keeps asking for favicon.ico
*
* @param client a reference to the calling WiFi client
*
*/
void Webpage::send404Page (WiFiClient client)
{
	client.print (F(
	    "HTTP/1.1 404 Not Found \r\n"
	    "Content-Type: text/html \r\n"
	    "Connection: close \r\n"
	    "\r\n"
	    "<html> \r\n"
	    "<body> \r\n"
	    "<h2>404: Not found</h2>\r\n \r\n"
	    "</body> \r\n"
	    "</html> \r\n"
	));
}

/*! @brief reboot the ESP32
 */
void Webpage::reboot()
{
	ESP.restart();
}