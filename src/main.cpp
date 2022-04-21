 /*
 ---------------------------------------
	WiFi Telemetry package for Hovercraft
 ---------------------------------------
	v2.9
		changed client->send() in events handler to 1second reconnect instead of 3secs - seems
		to fix exception issue after logging turned off!  Added click on chart to pause.
		
 	v2.8
		Change data sample rate from 20 to 50Hz
		added About button
		Chart clikc now pauses/unpauses
		Fixed bug in cam interpreter. Increased camera search timeout.
		Various tidy up stuff.  Now details lib and framewwork versions
		as new ones may not work!!!
 	
	v2.7
		Changed HTML thrust progress element to a <meter> element

	V2.6 -- release version -- 
		Now uses adc non-blocking functions from file in source directory
		thus making code compatible with ESP32 library versions higher than 1.0.4+

 	V2.5 -- release version -- 
		Used new MPU6050 accel/gyro library that reads more accurately and stable angle accuracy.
		Removed Pfod support code.
	
	V2.4 -- release version -- 
		Swapped to using tinyMPU lib for the 6050 accel.  Also now calibrate accel
  	and presure sensors using two seperate commands instead of one.

	V2.3
		Changed from using SPIFFS log file to uploading it using HTML chunked
		File download (SPIFFS large files downloads  (>150K or so) are very flaky with 
		the ESP libs - produces WDT resets at random.   
		
	V2.2
		Moved to Server Sent Event (SSE) driven web page to display data.
		Now saves log data to a file in ESP flash - it can be downloaded via web page
		Max filesize is 3MB.
	
	V2.1
		swapped I2C and HX711 interface pins to make wiring easier!
		Added web serer /data page that showws thrust and pressure readings continuously
	
	V2.0
		converted to WeMOS D1 Mini ESP32 module instead of using Arduino Mega256 and 2 x ESP8266!
		ESP32 handles both Station (connecting to camera) and AP modes (connnects to Android Pfod App) 
		Add voltage dividers for 5V to 3v3 sensor and GPS inputs!
	
	V1.5
	last update 16/10/2019
		changed gpsUpdate loop to "if" from "while"
 	 
	last update 30/9/2019
		changed HX711 library to add pull up to Dout to ensure noise doesn;'t lock up code.
	
	last update 5/9/2019
		changed EEPROM data block check to set pressures to default values if checksum invalid
		Rounds pressure values correctly now
	
	last update 5/9/2019
		added support for 2nd ESP8266 module (on Serial2) used to control a Xaomi Yi video camera recording
		Added command intepreter and Pfod Status info for camera
	 
 
 
 
Function
========
Reads pressure, accelerometer and GPS data (location and speed) and xmts
via WiFi to web page for data storage.  Controls Yi camera via WiFi
(turns camera video on and off when logging is started/stopped).

Access point 
------------  
  Acts as a WiFi access point named Hovercraft_Telemetry (NOTE - no password is required) and responds
  to a fixed IP address (192.168.1.99) on port 80
 


COMPILING
=========

Tools->Board: WEMOS D! MINI ESP£"
Tools->Partition Scheme: either "no OTA" (around 2mb prog space) or "Minimal SPIFFS" (3mb prog mem).

Install "ESP32 Exception Decoder" and  "ESP Sketch Data Upload" (used to format any SPIFFS partition) tools.

!!!!!!!!!!!!!
! IMPORTANT !
!!!!!!!!!!!!!

ESP32 Arduino board version (boards manager) MUST BE 1.0.4 - later versions do not support adcBusy, adcStart, etc.
non-blocking functions for reading adc presure values.
ESP32 uses adcAttachPin/Start/Busy/End functions
https://github.com/espressif/arduino-esp32/commit/eb46978a8d96253309148c71cc10707f2721be5e



HARDWARE 
==========

ESP32 D1 mini board with BT and WiFi

Power budget is 500Ma on 3V3 - NOTE - wrong regulator limits this to 150mA!!!!

Poor ESP32 flash programming
 
 see : flash_issues.txt in this directory
 also:  https://github.com/espressif/esptool/wiki/ESP32-Boot-Mode-Selection
 


ESP Notes
---------

PIN SELECTION:

ADC2 pins can't be used when WiFi is active - use ADC1 pins (GPIO32-39)
GPIO34-39 don't have pull up/down resistors on input (we need puul up for HX711)
I2C SDA/SCL are on GPIO21/22 by default - but can be on any pins
Rx is on pin IO16 (UART2 = serial2?) - limited range of pins available

 Rear pin view of ESP32 WeMos Min PCB:
  GND		Tx				RST		GND
  IO27	Rx				SVP		nc
  IO25	IO22-LED	IO26	SVN
  IO32	IO21			IO18	IO35
  TDI		IO17			IO19	IO33
  IO4		IO16			IO23	IO34
  IO0		GND				IO5		TMS
  IO2		Vcc				3v3		nc
  SDI		TDO				TCK		SD2
  CLK		SDO				SD3		CMD
   ^										 ^
   |						 				 |
	 |										 |
		---Pin rows we use--- 
 
Note- we need to pick up Vcc (5v) for pressure sensors and GPS module
Sensors and GPS Tx (to Rx pin) need votage divders as they are 5V signals!
 

WIFI:

WiFi device as AP and Station at same time:
https://siytek.com/esp8266-ap-and-station-mode/
There are some issues with this  mode:
 https://thecustomizewindows.com/2019/04/esp32-arduino-wi-fi-access-point-with-wi-fi-connection-station/
 https://techtutorialsx.com/2021/01/04/esp32-soft-ap-and-station-modes/#Working_as_Soft_AP_and_Station_simultaneously.

AP_IP is the IP we set for the softAP interface, the IP for the station interface is set by the Yi Camera when we connect
all servers are visible at both IP addresses so we need to determine which to be able to handle request from clients

Using AP_STA mode causes some issues if unable to connect, as a STA, to another network AP.  If a connection fails 
then AP gets disconnected (possiby a radio issue?).  Current solution is to reconnect the AP if the STA connection fails.

Use of atoi or atof C functions with a pointer seem to cause crash problems for the ESP32 core.

The ESP32 can only handle sending one TCP packet (of 1460 bytes of data) at a time.
 
 
MEMORY:
 
Most of STRINGS.INO could be moved to the file system rather than prog memory to save space


EXCEPTIONS

Use delay() or yield() to allow the OS to handle background tasks.  Unlike 8-bit Arduino
delay() allows background activity and interrupts.


!!! BAD CLONE ESP32 BOARDS !!
 
Some (most!) clone ESP32 Mini boards use lower spec components.   In particular, the 3.3V regulator is the low current 150mA
4B2x (XC6204/5) type instead of the reference design NCP/AMS1117 (1Amp) - this causes all sorts of sporadic WDT and brownout
issues when WiFi or BT is loaded.  Adding a 22uF cap to the 3v3 power helps but isn't a cure - large file downlaods are the main 
issue when WiFi is heavily loaded.  Repalacing the regulator with an AMS1117 is the solution!
 
Also the ESP32 reset transistor array is the wrong type (DDC143 instead of S8050) - it has base bias resistors that prevent it turning
off properly during reset (it sits at 0.5 x Pwr = 1.5v) - this makes it almost impossible to reliably reset the ESP32 into flash mode.



 
Devices:
--------
 GPS module @ 9600baud on serial port 2 (GPIO16)
 white wire is GND
 black is Vcc (5V - I = 40mA?)
 orange is Tx data (NOTE - needs res divider to interface with ESP32 3v3 UART !!!)
 
USB car power socket adapter and USB cable (micro USB)

HX711 load cell interface (built into load cell cable)
 Vcc = 3.3V I = 1.5mA
 wh/blk wire = DATA
 wh wire CLK
 Max sample rate is 10Hz (100mSec)
 Powered from 3V3 supply
 
XYZ accelerometer/Gyro MPU6050 (I2C)
 Wire to SDA/SCL and Vcc/Gnd
 Max sample rate is 1KHz (1mSec)
 3V3 power supply I = 3.9mA
	
3 x pressure sensors MXPV67002 series (analog output)
 Max sample rate is 1KHz (1mSec)
 Supply 5V I = 3 x 10mA = 30mA max.
 NOTE: needs voltage divider to 3v3 ESP32 inputs


WiFi used to control Xaomi Yi camera operating the video shutter


 
Input sensors:
--------------
 
 load cell (interfaced using an HX711 ADC near the load cell in the cable)
--------------------------------------------------------------------------------
 200Kg load cell detects pull thrust via HX711 ADC (in connecting cable)
 Red = pwr+
 blk = pwr-
 wh = sens+
 grn = sens-
 
 
Air pressure sensors (MXPV7002DP series)
----------------------------------------
Supply voltage: 5V!!
Voltage divider needed for ESP32 ADC - range is 3v3 max!!!!
Add 47K res in series before 33K to ground in parallel with a 470pf cap all close to ESP32.
 
Output current 0.1mA max. 0.5-(0.5 x Vcc) - (Vcc - 0.5) range (-/+ 2KPa)

The ESP32 reference is 3v3 and the module is 5V
 
Therefore the 0KPa voltage (after a divider to 3v3) is 2.5V * 0.66 = 1.65V
Voltage at ADC is 0.33V to 2.97V) (-2 to +2 KPa).  Usable range for us (0-+2KPA)
is 1.65V to 2.97V (1.32V range)

Pin voltage_value = (ADC_VALUE * 3.3 ) / (4095);
ADC reading for 1.65-2.97V are 2049-3689 (1640) giving a resolution of 1640 / 2000Pa = 1.2195 Pa per ADC count (0.82mV) 
ADC offset (to zero Pa) is 2048

To convert ADC count to Pa:
	ADC_COUNT - 2048) / 1.2195
 
ADC zero Pa offset = 2048 - however, we use calibration offset saved in EEPROM to determince the actual zero offset
 


Accelerometer/Gyro
------------------
MPU 6050 (3v3-5V)
Sample rate is 1KHz max (1mSec per sample)
Convert to degrees:
   ax = xyz[0];
   ay = xyz[1];
   az = xyz[2];
   double xAngle = atan( ax / (sqrt(square(ay) + square(az))));
   double yAngle = atan( ay / (sqrt(square(ax) + square(az))));
   double zAngle = atan( sqrt(square(ax) + square(ay)) / az);

   xAngle *= 180.00;   yAngle *= 180.00;   zAngle *= 180.00;
   xAngle /= 3.141592; yAngle /= 3.141592; zAngle /= 3.141592;




CAMERA
------

Camera acts as WiFi access point (see YiRemote.ino for details).  The ESP32 code connects to the camera using WiFi
then gets a command token from it to use to control functions in camera firmware.  It passes json commera command 
strings to the camera and returns responses.

Camera sends and receives JSON packets.  It sends  msg_id 7 (status) when something in the camera changes
 - containing various status "type" including battery state, command activity, current mode, etc.

{ "msg_id": 7, "type" =
		"adapter_status" ,"param":"1"} 			// 0 = on battery power, 1 = charging
		"start_usb_storage" }
		"wifi_will_shutdown" }
		"precise_capture_data_ready" }
		"photo_taken" ,"param":"/tmp/fuse_d/DCIM/100MEDIA/YDXJnnnn.jpg"}
		"battery" ,"param":"81"} 							// battery charge state
		"vf_start" }
		"vf_stop" }
		"video_record_complete" ,"param":"/tmp/fuse_d/DCIM/100MEDIA/YDXJ0015.mp4"}
		"switch_to_cap_mode" } 								// picture mode for button
		"switch_to_rec_mode" }  							// video mode for button
		"start_video_record" }
		"video_record_complete" ,"param":"/tmp/fuse_d/DCIM/100MEDIA/YDXJ0065.mp4"}<--
		"switch_to_cap_mode" }
		"start_photo_capture" ,"param":"precise quality;off"}<--
		"precise_capture_data_ready" }
		"photo_taken" ,"param":"/tmp/fuse_d/DCIM/100MEDIA/YDXJ0066.jpg"}<--
		"switch_to_rec_mode" }								// user has disabled wifi on camera
		"wifi_will_shutdown" }


Commands sent to camera are all acknowledged
"rval" is returned for all comands, EXCEPT 7, sent to indicate command has been accepted/actioned
 -4 = invalid token
 -3 = invalid command param
 0  = command executed OK
  
 

Get token:      			{"msg_id":257,"token":x} (executed by the ESP32)
		response:
								{ "rval": 0, "msg_id": 257, "param": 1 } // param = requested token


Take photo:          {"msg_id":769,"token":x} -
		response -->
								{"rval:0,"msg_id":769} 
async status
								{ "msg_id": 7, "type": "start_photo_capture" ,"param":"precise quality;off"}
								{ "msg_id": 7, "type": "precise_capture_data_ready" }
								{ "msg_id": 7, "type": "photo_taken" ,"param":"/tmp/fuse_d/DCIM/100MEDIA/YDXJnnnn.jpg"}
								{ "msg_id": 7, "type": "battery" ,"param":"81"}
								{ "msg_id": 7, "type": "vf_start" }
								{"rval":0,"msg_id":769}


 Stop record video:  {"msg_id":514,"token":x}
								{"rval":0,"msg_id":514}
								{ "msg_id": 7, "type": "start_video_record" }
								{"rval":0,"msg_id":514}
								

								 
 Stop record video:   {"msg_id":514,"token":x}
								// stop recodring:
								{ "msg_id": 7, "type": "video_record_complete" ,"param":"/tmp/fuse_d/DCIM/100MEDIA/YDXJ0015.mp4"}
								{ "msg_id": 7, "type": "vf_start" }
								{"rval":0,"msg_id":514}


 Get live stream:     {"msg_id":259,"token":x,"param":"none_force"}
 Get bat stat:        {"msg_id":13,"token":x} -> {"rval":0,"msg_id":13,"type":"adapter","param":"100"}
 Get all settings:    {"msg_id":3,"token":x}
 Get setting choices: {"msg_id":9,"param":"video_resolution","token":x}
 Get single setting:  {"msg_id":1,"type":"video_resolution","token":x}
 Get SDCARD space:    {"msg_id":5,"type":"free","token":x}  							// param returned is space in memory
 Set single setting:  {"msg_id":2,"type":"video_resolution","param":"1920x1080 60P 16:9","token":x}
 Activate log:        {"msg_id":2,"type":"save_log","param":"on","token":x} -> READ telnet x.x.x.x + tail -f /tmp/fuse_a/firmware.avtive.log
 VLC stream: rtsp://192.168.42.1:7878/live

  
  
*/

// arduino libs
#include <Arduino.h>
#include <WiFi.h> // https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html
#include <WiFiClient.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>

// special libs
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <MPU6050_tockn.h> // better library than the others!

// modified local libs in src dir
#include "HX711_ADC_jr.h" // use modified local version (has pull up enabled on Dout)!

// HTML string data
#include "strings.cpp"

// set DEBUG to 1 to send xtra debug messages 
bool debug = false; // true send deug info from serila


//=============================
// constants
const long DEFAULT_BAUD_RATE = 115200;  //serial baud rate for arduino monitor

// https://thecustomizewindows.com/2019/04/esp32-arduino-wi-fi-access-point-with-wi-fi-connection-station/
// https://techtutorialsx.com/2021/01/04/esp32-soft-ap-and-station-modes/#Working_as_Soft_AP_and_Station_simultaneously
// AP_IP is the IP we set for the softAP interface, the IP for the station interface is set by the Yi Camera when we connect
// all servers are visible at both IP addresses so we need to determine which to be able to handle request from clients
const IPAddress AP_IP(10,1,1,1); // server and AP IP address - MUST be different netID from camera!
const IPAddress AP_subnet(255,255,0,0);
#define AP_SSID "Hovercraft_Telemetry" // max length 32 chrs!!!

//--------------------------
//hardware pins
/*
 Rear pin view of ESP32 WeMos Min PCB:
  GND		Tx					RST		GND
  IO27	Rx					SVP		nc
  IO25	IO22 (LED)	IO26	SVN
  IO32	IO21				IO18	IO35
  TDI		IO17				IO19	IO33
  IO4		IO16				IO23	IO34
  IO0		GND					IO5		TMS
  IO2		Vcc					3v3		nc
  SDI		TDO					TCK		SD2
  CLK		SDO					SD3		CMD
   ^									 ^
   |						 			 |
	 |									 |
			Pin rows we use 
*/
const int pin_pressBag = 33; // pressure sensors ADC1 (ADC2 can't be used when WiFi active)
const int pin_pressBow = 34;
const int pin_pressCushion = 35;
// HX711 hardware interface to load cell
const int pin_cellD = 32; // wh/blk wire
const int pin_cellCK = 2; // wh wire
// MPU6050 accelerometer uses Wire library
const int pin_SDA = 25;
const int pin_SCL = 27; // GPIO2 must float during boot so set as an output!
// GPS module
const int pin_GPS_Rx = 4; // GPS on UART2 (UART1 is for IDE comms)


//=============================
// objects

// main loop update timer
//const unsigned long sampleTime = 50; // log data sample time period in millisec (20Hz)
const unsigned long sampleTime = 20; // log data sample time period in millisec (50Hz)
// based on the theoretical maximum transfer rate through WiFi of 19200 baud. We 
// send appr. 80chrs per sample period = 24 samples per period = 42mSecs min).
unsigned long sampleTimer = millis();

// buffer to store up to one secs worth of samples for the chunked handler
// NOTE - don't include any % chrs in this string  as we use snprintf to copy this to server buffer
// csvHeader includes 2nd header line with default values for use with the Plotter.php code!
const String csvHeader = 
		"sampletime, Bag press, Bow, Cushion, Pitch (deg), Roll, Yaw, latitude, longitude, Speed (mph), Time (HHMMSS)\r\n"		"type=X,xcol=sampletime&type=Y&units=Pa&filter=8,xcol=sampletime&type=Y&units=Pa&filter=8,xcol=sampletime&type=Y&units=Pa&filter=8,xcol=sampletime&type=Y&units=deg&filter=8,xcol=sampletime&type=Y&units=deg&filter=8,xcol=sampletime&type=Y&units=deg&filter=8,,,type=Y&units=mph&filter=4,type=H&xcol=sampletime&factor=[d1$d2$_d3$d4$_d5$d6$]";
// byte count:			10				4			4			4			5		5		5		10		10		4		6		2			
// = 11 commas plus 69 = 80 chars string length - NOTE csvHeader is 480 bytes long!!!
const unsigned int maxSampleLen = 80;  // NOTE - maximum length of a sample text line!!!
static uint8_t sampleBuffer[maxSampleLen * (1000 / sampleTime)]; // needs to be large enough to take one seconds worth of sample data!!!
const unsigned int maxBufLen = sizeof(sampleBuffer);
volatile int readPtr = 0;
volatile int writePtr = 0;
volatile int bufCounter = 0; // # of bytes in buffer

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
#ifndef SSE_MAX_QUEUED_MESSAGES
	#define SSE_MAX_QUEUED_MESSAGES 32 // for older libs that don't define this value!
#endif
AsyncEventSource events("/events"); // Create an Event Source on /events
//static const size_t EVENTS_BUFFER_SIZE = 1460; //WIFICLIENT_MAX_PACKET_SIZE; // Max data size for standard TCP/IP packet
static const size_t EVENTS_BUFFER_SIZE = (maxSampleLen * 2); //twice the mac length of the string we see!
char eventsBuffer[EVENTS_BUFFER_SIZE]; // bufferred events data
//size_t eventsBufferPtr = 0; // current index

const unsigned long restartTimeOut = 1000; // delay before reseting the ESP32 after getting a command to do so!
unsigned long restartTimer = 0; // timer, zero = disabled!
 
bool serLogMode = false;
bool webLogMode = false;
bool camState = false;
const byte modeToggle = 3;
const byte modeSet = 1;
const byte modeClear = 0;
unsigned long webLogStartTime = millis();
unsigned long serLogStartTime = millis();

bool PcalMode = false; // pressure calibration 
bool AcalMode = false; // accel calibration
unsigned long calDelayTimer = millis(); // delay timer before starting calibration to allow for a menu update to show calibration is in progress
const int preCalDelay = 1000; // delay after setting calMode until we actually do the calilbration

String sensorText;


// acceleromter sensor MPU6050
MPU6050 mpu(Wire, 0.01, 0.99);  // coefficients for acc + gyro used to calibrate sensors (default is 0.02 & 0.98


// ANALOG INPUTS
// =============
// pressure sensors
int current_sensor_pin = 0; // sensor currently being read

const float pressMult = 1.2195F; // number of Pascals per ADC count of 1
volatile int press_bag, press_bow, press_cushion; 
const int maxPressure = 2000;
const int minPressure = -500; // clamp values for pressures
const int press_offset = 4096 / 2; // 0K Pa default ADC max value is 4096
const int max_offset = 4095;
const int min_offset = 0;

// eeprom calibration data
const int eepromStartAddress = 0;

struct eeprom_block {
	int offsetBag = press_offset;
	int offsetBow = press_offset;
	int offsetCushion = press_offset;
	float offsetX;
	float offsetY;
	float offsetZ;
	unsigned int checksum;
} eeprom_data;

// note that integers are 32bit on ESP32 - not 16 bit!!!
const int eeprom_storageSize = sizeof(eeprom_data);


// DIGITAL INPUTS
// ==============

// Load Cell
// ---------
HX711_ADC load_cell(pin_cellD, pin_cellCK); // thrust load cell
// thrust in lbf
int thrust;
/* note: checked against InB's digital scale 21/6/17
	indicated that 5007 was reading approx 1.1% low so
	value changed to 4952 to compensate */

float const thrust_scale = 9904;	// set to scale thrust to lb units //	#define thrust_scale 4952	// set to scale thrust to 0.5 lb units
bool loadCellDetected = false;
unsigned long thrustDelayTimer = millis();
const int thrustDelay = 333; 	// only return new thrust value every so often to avoid screen flicker
															// also used to return serial log data continuously so make port can keep up!

//--------------
//const int MPU_addr = 0x68;  // I2C address of the MPU-6050
// accel analog can be +ve or -ve so integer)
float accel_Y, accel_X, accel_Z;
const float maxDeg = 90;  // clamp angles
const float minDeg = -90;

// ============================
// Yi camera stuff
WiFiClient camClient; // camera client
const char* YI_SSID = "YDXJ_4319057";
const char* password = "1234567890";
const IPAddress CamIP(192,168,42,1);
const int jsonPort = 7878;

// camera command codes
const unsigned int camIdle = 0; // no command active
const unsigned int camCmdToken = 257;
const unsigned int camCmdRecOn = 513; 
const unsigned int camCmdRecOff = 514;
const unsigned int camCmdGetSD = 5; // memory free space
const unsigned int camCmdGetBatt = 13; // battery state %
const unsigned int camStatus = 7; // asyn status values returned
 
float camMem = 0; // free memory on camera SD card
String camBat = ""; // battery percentage charge 0-100% - must be empty at startup!!

unsigned long camFileNameTimer = 0; // delay after stopping rec to wait for filename tofrom camera
const unsigned int camFileNamePeriod = 3000; // wait 3secs for status response that includes last filename used for rec.
bool waitForCamFileName = false;

unsigned int camCmd = camIdle; // currently active camera command msg_ID 
bool chkSD = true; // flag to check SD card space - true to get done at startup, therafter every time record is used
unsigned int token = 0; // command token from camera
String vidFileName; // name of saved video file added to end of CSV

unsigned int json_index = 0; // current position in json string being received
const int max_json_length = 255; // max buffer size!
char json_string[max_json_length]; // not sure exactly how long response data might be - longest seen is 128bytes!
StaticJsonDocument<255> json_doc; // json deserialzation result here

// ============================
// GPS vars 
// white wire is GND
// black is Vcc
// orange is Tx data
TinyGPSPlus GPS; // GPS data interpreter
const long GPSbaud = 9600; // GPS receiver transmit rate
bool gotGPS = false; // true if GPS locked




// *******************************************
// *******************************************
// 				Functions
// *******************************************

// declarations
void noblockDelay(long delay);



void save_calData()
{
	byte * byteStorage = (byte *)&eeprom_data;
  eeprom_data.checksum = 0; // zero checksum
	for (unsigned int i = 0; i < eeprom_storageSize; i++)  // write data - including checksum!
	{
		if (i < (eeprom_storageSize - sizeof(eeprom_data.checksum))) // don't checksum the checksum int!
			eeprom_data.checksum += (unsigned int)byteStorage[i]; // update checksum
		EEPROM.write(eepromStartAddress + i, byteStorage[i]);
	}
	EEPROM.commit(); // do the write!
}

// read calibration data from EEPROM - return false if checksum fails
// NOTE  that integers are 32bit on ESP32 - not 16 bit so we use sizeof() to work with any CPU!!!
bool get_calData() 
{
 	byte * byteStorageRead = (byte *)&eeprom_data;  // pointer to data
  unsigned int checksum = 0;
  for (unsigned int i = 0; i < eeprom_storageSize; i++) 
  {
		byteStorageRead[i] = EEPROM.read(eepromStartAddress + i);
 		if (i < (eeprom_storageSize - sizeof(eeprom_data.checksum))) // checksum is int so don't checksum it!
			checksum += (unsigned int)byteStorageRead[i]; // don't checksum the checksum!
	}

	if (eeprom_data.checksum != checksum) // checksum good?
	{ // no, so reset to default values
		eeprom_data.offsetBag = press_offset;
		eeprom_data.offsetBow = press_offset;
		eeprom_data.offsetCushion = press_offset;
		eeprom_data.offsetX = eeprom_data.offsetY = eeprom_data.offsetZ = 0;
		save_calData(); // write valid data back to eeprom
		return false;
	}
	return true; // data is good!
}

void printCalib()
{
	Serial.print(F("Bag offset = "));
	Serial.println(eeprom_data.offsetBag);
	Serial.print(F("Bow offset = "));
	Serial.println(eeprom_data.offsetBow);
	Serial.print(F("Cushion offset = "));
	Serial.println(eeprom_data.offsetCushion);
	Serial.print("X offset = ");
	Serial.println(eeprom_data.offsetX);
	Serial.print("Y offset = ");
	Serial.println(eeprom_data.offsetY);
	Serial.print("Z offset = ");
	Serial.println(eeprom_data.offsetZ);
	Serial.print("checksum = ");
	Serial.println(eeprom_data.checksum);
}

// write string into sampleBuffer
// if not enough space then discard it & ret false
unsigned int sampleBufferWrite(String sText)
{
	sText = sText + "\r\n"; // add EOL
	if ((maxBufLen - bufCounter) >= (sText.length())) // buffer has space for another line?
	{
		for (int i=0; i < sText.length(); i++)
		{
			sampleBuffer[writePtr++] = sText[i];
			writePtr %= maxBufLen; // wrap
			bufCounter++;
		}
		return bufCounter;
	}
	return false;
}

// gets called with a line of data to send to Web client using server-side-event
// if networks isn't too busy, send buffer content regardless of size
// NOTE - 	problem with String.c_str passed to Async lib - causes exception
//  				we use separate char array with String.toCharArray instead
// https://www.arduino.cc/reference/en/language/variables/data-types/string/functions/c_str/
void eventsBufferWrite(String data)
{
 	if (data.length() == 0 || events.count() == 0  || events.avgPacketsWaiting() >= SSE_MAX_QUEUED_MESSAGES) // got data, connected client & space?
	 	return; // no data or no active SSevent connected
	// send each message immediately!
	data += "|"; // add eol chr
	data.toCharArray(eventsBuffer, data.length() + 1); // include the null terminator!
	events.send(eventsBuffer, "data", millis()); // send buffer - async server so won't block!
}

// feed GPS interpreter
// if GPS data is valid and up-to-date sets gotGPS true else false
void gpsUpdate() 
{
	if (Serial2.available()) // feed the GPS interpreter whilst serial data is available
/*
		if (debug)
		{
			byte c = Serial2.read();
			Serial.print(char(c));
			GPS.encode(c); // TinyGPS -> decode sentence
		}
		else
*/
		GPS.encode(Serial2.read()); // TinyGPS -> decode sentence

	// is GPS data valid, up to date & have we got enough satellite fixes?
	gotGPS = (		
					GPS.speed.isValid() 
			&& 	(GPS.satellites.value() > 3)  // need at least 4 satellites, 9-12 is best, 5 is poor 
			&& 	GPS.speed.age() < 5000
		 ); // up to date? - we've lost fix if not!
	
}

// update acceleromter readings
void XYZupdate()
{
	mpu.update(); // reading clamped to +/-45 degrees - we shoudln't see anything like that!
	accel_X = min(max(mpu.getAngleX() - eeprom_data.offsetX, minDeg), maxDeg);
	accel_Y = min(max(mpu.getAngleY() - eeprom_data.offsetY, minDeg), maxDeg);
	accel_Z = mpu.getAngleZ() - eeprom_data.offsetZ; // no clip for yaw!
}

// updates pressure value current_sensor
// returns true if updated, false if ADC not ready yet
void pressureUpdate()
{
	press_bag = (min(max((int)round((float)(analogRead(pin_pressBag) - eeprom_data.offsetBag) * pressMult), minPressure), maxPressure));
	noblockDelay(1);
	press_bow = (min(max((int)round((float)(analogRead(pin_pressBow) - eeprom_data.offsetBow) * pressMult), minPressure), maxPressure));
	noblockDelay(1);
	press_cushion = (min(max((int)round((float)(analogRead(pin_pressCushion) - eeprom_data.offsetCushion) * pressMult), minPressure), maxPressure));
	noblockDelay(1);
	
/*
 	if (adcBusy(current_sensor_pin))  // is a conversion finished?
		return false; // not ready yet
	else
	{
		int reading = adcEnd(current_sensor_pin); // clamped reading
		switch (current_sensor_pin)
		{
			case pin_pressBag:
				press_bag = (min(max((int)round((float)(reading - eeprom_data.offsetBag) * pressMult), minPressure), maxPressure));
				current_sensor_pin = pin_pressBow;
				break;
			case pin_pressBow:
				press_bow = (min(max((int)round((float)(reading - eeprom_data.offsetBow) * pressMult), minPressure), maxPressure));
				current_sensor_pin = pin_pressCushion;
				break;
			case pin_pressCushion:
				press_cushion = (min(max((int)round((float)(reading - eeprom_data.offsetCushion) * pressMult), minPressure), maxPressure));
				current_sensor_pin = pin_pressBag;
				break;
			default: // current sensor is wrong so reset!
				current_sensor_pin = pin_pressBag;
		}
		adcAttachPin(current_sensor_pin); // start next ADC reading
		adcStart(current_sensor_pin);
		return true; // got new reading
	}
*/
}

int swap01(int in) {
  return (in==0)?1:0;
}

//interrupt routine for load cell HX711:
void cellReadyISR() {
  load_cell.update();  // get new reading from hx711
}

void setCalMode()
{
	calDelayTimer = millis(); // restart timer
	serLogMode = webLogMode = waitForCamFileName = false; // make sure we arean't logging!
}

void setPcalMode()
{
	PcalMode = true;
	setCalMode();
}

void setAcalMode()
{
	AcalMode = true;
	setCalMode();
}

// send json command to camera
// including token
// command isn't sent unless a token is set OR it's a getToken cmd
void sendCamCmd(const int msg_id, const char type[] = "")
{
	if (camClient.connected()) // only if cam available!
	{
		if ((token != 0) || msg_id == camCmdToken)  // ignore all cmd requests until we've got a token
		{
			camCmd = msg_id; // set currently active cmd (cleared when response received)
			camClient.printf("{\"msg_id\":%i, \"type\": \"%s\", \"token\":%i }", msg_id, type, token);
		}
	}
}

// check for camera json data and handle response
void camUpdate()
{
	// CAMERA
	if (camClient.connected())
	{
		// check if we should be waiting for the video filename
		waitForCamFileName = (vidFileName == "") && ((millis() - camFileNameTimer) < camFileNamePeriod); 
	
		// get token if we don't have one
		// if cam state mode has changed turn on/off cam record as required 
		if (camCmd == camIdle) // no active cmd?
		{
			if (token == 0) // not got token yet?
				sendCamCmd(camCmdToken); // ask for a command token first at startup!
			else if (camState != (serLogMode || webLogMode)) // log mode changed?
			{
				camState = serLogMode || webLogMode; // update mode
				if (camState)
				{
					vidFileName = ""; // delete last filename used
					sendCamCmd(camCmdRecOn); // start recording
				}
				else
				{
					sendCamCmd(camCmdRecOff);  // stop recording video
					chkSD = true; // set flag to check memory after a record session!
				}
			}
			else if (!(serLogMode || webLogMode)) // all inactive?
			{
				if (camBat == "") // not got battery state yet?
					sendCamCmd(camCmdGetBatt); // check batt. once only at startup - thereafter the cam sends status every time the batt charge changes
				else if (chkSD) // check memory space after a record cmd OR at startup!
				{
					sendCamCmd(camCmdGetSD, "free"); // get memory state after we got a token (switch on)
					chkSD = false;
				}
			}
		}

		// check for command response from camera - note it sends responses even if a command hasn't been issued!
		// if a reponse has a non-zero "rval" (return value) then there was an error with the command in msg_id

		// all user commands are acknowledged by rval - we just extract whatever param data they return
		// async responses are used to update status data

		// do NOT use while for this loop - it can lockup if garbage continuallay received from camera
		if (camClient.available()) // status from camera WiFi
		{
			char c = camClient.read(); // get the char from the camera
		
			if (json_index > 0)	// are we reading a json string?
			{
				json_string[json_index] = c; // save in buffer
				json_index++;
				if (json_index >= max_json_length) // buffer overflow?
					json_index = 0; // clear buffer!
				else
				{
					if (c == '}') // end of json string found?
					{
						json_string[json_index] = 0; // terminate string
						json_index = 0; // got all of json string so reset ptr
						DeserializationError error = deserializeJson(json_doc, json_string, strlen(json_string));

						if (error == 0)
						{
							const int msg_id = json_doc["msg_id"];
							const int rval = json_doc["rval"]; // error return value from command
							const char* type = json_doc["type"]; // eg "start_photo_capture", etc
							// param could be numeric or string so we read as both!
							// https://forum.arduino.cc/index.php?topic=643838.0
							const char* paramstr = json_doc["param"];
							const unsigned int paramint = json_doc["param"];

							if (msg_id == camStatus) // status parameters returned asynchronously
							{
								if (strcmp(type, "battery") == 0)
									camBat = paramstr; // get battery status - is sent async at any time by camera

								else if (strcmp(type, "video_record_complete") == 0)
								{
									vidFileName = paramstr; // get video filename
									if (vidFileName.length() > 0)
									{
										// NOTE -we can't send this to web page client as logmode has already been turned off some time ago!
										vidFileName = vidFileName.substring(vidFileName.lastIndexOf("/")); // get end filename only
										String text = String("Video," + vidFileName + ",,,,,,,,,");  // last line in CSV file
										Serial.println(text); // and send to serial port!
										sampleBufferWrite(text); // write filename to web log
									}
								}
							}
							else
							{
								// must be a cmd acknowledgement
								if (rval == 0) // no errors
								{
									switch (msg_id) {
										case camCmdToken:
											token = paramint; // save new token
											break;
										case camCmdGetBatt:
											camBat = paramstr; // save new battery charge level 
											break;
										case camCmdGetSD:
 											camMem = float(paramint) / 1048576; // free mem in GB
											break;
										case camCmdRecOn:
											vidFileName = ""; // clear old video filename
											break;
										case camCmdRecOff:
											break;
									}
									if (msg_id == camCmd) camCmd = camIdle; // got an Ack for current command?
								}
								else // command rval Ack error
								{
									if (debug) Serial.printf("%i camcmd error - %s\r\n", camCmd, json_string);
									sendCamCmd(camCmd); // retry command again!
								}
							}
						}
						else
							if (debug) Serial.printf("%s - json decode err - %s\r\n", (char *)error.f_str(), json_string);
					}
				}
			}
			else if (c == '{')	// start of json string?
			{
				json_string[0] = c; // save in buffer
				json_index = 1; // point to buffer start
			} // ignore any other data ...
		}
	}
}

void toggleDebug()
{
	debug = !debug; // toggle
	Serial.setDebugOutput(debug); // ESP32 debug info
	Serial.printf("DEBUG OUTPUT %s\r\n", debug ? "ON" : "OFF");
}

void serCheck()
{
	// check for serial port commands through USB
	if (Serial.available())
	{
		char c = Serial.read();
		switch (c)
		{
			case '?': // help screen
				Serial.println(help);
				break;			
			case ' ': // data values
				Serial.printf("\r\nTmr: %lu\r\nbag: %i,\r\nbow: %i\r\ncush: %i\r\nPitch: %.1f\r\nRoll: %.1f\r\nYaw: %.1f\r\nLat: %.6f\r\nLng: %.6f\r\nmph: %.1f\r\nTme: %u \r\n",
				(millis() - serLogStartTime), press_bag, press_bow, press_cushion, accel_X, accel_Y, accel_Z, GPS.location.lat(), GPS.location.lng(), GPS.speed.mph(), (GPS.time.value() / 100)
				);
				if (loadCellDetected) Serial.printf("Thrust : %i lbf\r\n", thrust);
				if (camClient.connected()) Serial.printf("Camera batt:  %s%%, free space: %.2fGB\r\n", camBat.c_str(), camMem);
				break;				
			case 'D': // debug mode
			case 'd':
				toggleDebug();
				break;
			case 'P': // calibrate press command
			case 'p':
				setPcalMode();
				break;
			case 'A': // calibrate accel command
			case 'a':
				setAcalMode();
				break;
			case 'L': // log mode on/off
			case 'l':
				serLogMode = !serLogMode; // toggle log mode
				serLogStartTime = millis();
				if (debug)
					Serial.printf("Serial Log %s\r\n", (serLogMode ? "ON" : "OFF"));
				if (serLogMode)
					Serial.println(csvHeader.substring(0, csvHeader.indexOf("\r"))); // send 1st line only - no plotter paramters)!
				break;
			case 'R': // log mode on/off
			case 'r':
				restartTimer = millis();  //start timer
				break;
		}
	}
}

// delay while still handling gps data 
void noblockDelay(long delay)
{
	long timer = millis();
	while ((millis()-timer) < delay)
	{
		gpsUpdate(); // need to do this every loop to ensure we don't miss GPS sentence data
		camUpdate();
		yield();
	}
}


// read static values from ADC and accel sensors, average and store in EEPROM as offsets
// calMode is the sensor we are calibrating (1-n)
//step we are at (to prevent blocking awaiting sensor input)
void Pcalibrate()
{	
	if (!PcalMode || webLogMode || serLogMode) // abort - can't calibrate if still logging!
		return;

	Serial.println(F("\r\n... Calibrating Press. Sensors ...\r\n"));

//	while (adcBusy(current_sensor_pin))  // wait until any current ADC sampling is complete!
//		noblockDelay(1);

	// average 16 ADC samples from each sensor limed by int size (64K / 16 = 4K )
	eeprom_data.offsetBag = eeprom_data.offsetBow = eeprom_data.offsetCushion = 0;

	for (int i=0; i<16; i++) // read sensor 16 times and average it
	{
		noblockDelay(100);
		eeprom_data.offsetBag += analogRead(pin_pressBag); // pick up reading
		noblockDelay(100);
		eeprom_data.offsetBow += analogRead(pin_pressBow); // pick up reading
		noblockDelay(100);
		eeprom_data.offsetCushion += analogRead(pin_pressCushion); // pick up reading
	}
	eeprom_data.offsetBag = eeprom_data.offsetBag >> 4; // div by 16
	eeprom_data.offsetBow = eeprom_data.offsetBow >> 4; // div by 16
	eeprom_data.offsetCushion = eeprom_data.offsetCushion >> 4; // div by 16
	
/*
 		adcAttachPin(pin_pressBag); // start another conversion
		adcStart(pin_pressBag); // start another conversion
		while (adcBusy(pin_pressBag))
			noblockDelay(1); // wait for it
		eeprom_data.offsetBag += adcEnd(pin_pressBag); // pick up reading
	}
	eeprom_data.offsetBag = eeprom_data.offsetBag >> 4; // div by 16

	for (int i=0; i<16; i++) // read sensor 16 times and average it
	{
		adcAttachPin(pin_pressBow); // start another conversion
		adcStart(pin_pressBow); // start another conversion
		while (adcBusy(pin_pressBow))
			noblockDelay(1); // wait for it
		eeprom_data.offsetBow += adcEnd(pin_pressBow); // pick up reading
	}

	eeprom_data.offsetBow = eeprom_data.offsetBow >> 4; // div by 16
	
	for (int i=0; i<16; i++) // read sensor 8 times and average it
	{
		adcAttachPin(pin_pressCushion); // start another conversion
		adcStart(pin_pressCushion); // start another conversion
		while (adcBusy(pin_pressCushion))
			noblockDelay(1); // wait for it
		eeprom_data.offsetCushion += adcEnd(pin_pressCushion); // pick up reading
	}
	eeprom_data.offsetCushion = eeprom_data.offsetCushion >> 4; // div by 16
*/	
//	adcAttachPin(current_sensor_pin); // re start normal ADC 
//	adcStart(current_sensor_pin);

	save_calData(); // write to eeprom
	
	Serial.println(F("\r\nPressure sensor calibration complete ...\r\n"));
	printCalib();
	
	PcalMode = false;
}


void Acalibrate()
{
	if (!AcalMode || webLogMode || serLogMode) // abort - can't calibrate if logging!
		return;

	Serial.println(F("\r\n... Calibrating Accelerometer ...\r\n"));
	
  mpu.calcGyroOffsets(false);

	double offsetX = 0, offsetY = 0, offsetZ = 0; // need floats cos accel values would overflow integers!
	const int sampleCount = 128; // number of samples
	for (int i=0; i<sampleCount; i++) // read sensor count times and average it
	{
		mpu.update();
		noblockDelay(50);
		offsetX += mpu.getAngleX();
		offsetY += mpu.getAngleY();
		offsetZ += mpu.getAngleZ();
	}
	eeprom_data.offsetX = offsetX / (float)sampleCount; //scale result
	eeprom_data.offsetY = offsetY / (float)sampleCount;
	eeprom_data.offsetZ = offsetZ /  (float)sampleCount;
	save_calData(); // write to eeprom
	
	Serial.println(F("\r\nAccel/Gyro calibration complete ...\r\n"));
	printCalib();

	AcalMode = false;
}

// if sampletimer has expired, read sensors and send data
void updateSensors()
{
	if (!PcalMode && !AcalMode) // not when calibrating!
	{
		// timed update of sensor readings every 50mS or so
		// also sends result at this rate to SSE web page
		if ((millis() - sampleTimer) >= sampleTime)
		{
			sampleTimer = millis(); // restart timer
			
			pressureUpdate(); // get pressure values
			
			XYZupdate(); /// acceleromter positions

			sensorText = 
				String(millis() - webLogStartTime) + ","
				+	String(press_bag) + ',' 
				+	String(press_bow) + ',' 
				+	String(press_cushion) + ',' 
				+ String(accel_X,1) + ',' 
				+ String(accel_Y,1) + ',' 
				+ String(accel_Z,1)  + ','
				+ String(GPS.location.lat(), 6) + ','  // position to 1m approx.
				+ String(GPS.location.lng(), 6) + ','
				+ String(GPS.speed.mph(), 1) + ','
				+ String(GPS.time.value() / 100) // time in HHMMSS
				; 

			if (webLogMode)
				// copy sample into buffer for chunked file download 
				// if no space in buffer then ignore this sample
				sampleBufferWrite(sensorText);
			else 
			// if (events.count() > 0 && events.avgPacketsWaiting() < SSE_MAX_QUEUED_MESSAGES) // events connected to client and space in queue?
				// note - some tablets are pretty slow at chart update so we buffer data into 1462 packet sizes before sending
				// buffer events data until enough for a packet then send it!
				eventsBufferWrite( String(
													(debug ? "128," : "0,") 
												+ (token != 0 ? (String(camBat) + "," 
												+ String(camMem)) : ",") + ","
												+ String((gotGPS ? "1" : "0")) + ","
												+ (loadCellDetected ? String(thrust) : "") + "," 
												+ vidFileName + ","
												+ sensorText));
		}
	}

	// slower polled stuff
	// return thrust and sensor values to serial monitor every 0.5s or so while not logging
	// we also use this to update to handle longer delay stuff
	if ((millis() - thrustDelayTimer) >= thrustDelay)
	{
		thrustDelayTimer = millis();
		if (loadCellDetected) thrust = min(max(int(load_cell.getData()), 0), 440); // only read if a load cell has been found!

		// send slow-speed data to web page if connected and logging or calibrating
		if (PcalMode)
			eventsBufferWrite(debug ? "129," : "1,");
		else if (AcalMode)
			eventsBufferWrite(debug ? "130," : "2,"); 
		else if (webLogMode) // send cal, log modes, cambat, cammem, gpsStat, logtime - no sensor data
			eventsBufferWrite(String(
													(debug ? "131," : "3,") 
												+ (token!=0 ? (String(camBat) + "," + String(camMem)) : ",") + ","
												+ String((gotGPS ? "1" : "0")) + ","
												+ String(millis() - webLogStartTime) + ","
											));
		if (serLogMode) 
			Serial.println(sensorText);
//			Serial.printf("%lu,%i,%i,%i,%.1f,%.1f,%.1f,%.6f,%.6f,%.1f,%u \r\n",
//				(millis() - serLogStartTime), press_bag, press_bow, press_cushion, accel_X, accel_Y, accel_Z, GPS.location.lat(), GPS.location.lng(), GPS.speed.mph(), (GPS.time.value() / 100)
//				);
		else if (!webLogMode && loadCellDetected) // only send thrust if NOT logging!
			Serial.printf("Thrust : %i lbf\r\n", thrust);
	}
}

//=============================
// 					SETUP
//=============================
void setup ( void ) {

  Serial.begin(DEFAULT_BAUD_RATE);

	delay(1000);  // skip the ESP8266 debug output stuff

	Serial.println(help);

  Serial.println(F("- INITIALISING -"));

  if (!EEPROM.begin(eeprom_storageSize))
  {
    Serial.println(F("Fatal error - failed to initialise EEPROM"));
    while(1); // die!
  }

	Serial.println(F("\r\nReading saved calibration data:"));

	if (get_calData()) // get calbration data
	{
		printCalib();
		Serial.println(F("Saved calibration data is valid\r\n"));
	}
	else
		Serial.println(F("Calibration date checksum error - has been reset to default values.\r\n"));

	Serial.println(F("Initialising Gyros.\r\n"));
  Wire.begin(pin_SDA, pin_SCL, 100000u);
  mpu.begin();
  mpu.calcGyroOffsets(false); // set true to display on serial!
	
	Serial2.begin(9600, SERIAL_8N1, pin_GPS_Rx);  // GPS serial port = GPIO16 = Rx, don't need Tx

  load_cell.begin();
	const long stabilisingtime = 2000; // tare precision can be improved by adding a few seconds of stabilising time
	load_cell.start(stabilisingtime);
  delay(400);
	Serial.print(F("Thrust load cell "));
  if (load_cell.getTareTimeoutFlag()) 
  {
		loadCellDetected = false;
		Serial.println(F("not found, check connections.\r\n"));
  }
  else
  {
    load_cell.setCalFactor(thrust_scale); // set calibration value (float)
		attachInterrupt(digitalPinToInterrupt(pin_cellD), cellReadyISR, FALLING);
		loadCellDetected = true;
    Serial.println(F("started OK.\r\n"));
  }

	// now setup the two WiFi networks
	// an access point for web page serving and a station to conect to camera
  Serial.println(F("- WiFi SETUP -"));
	WiFi.disconnect();
	WiFi.mode(WIFI_OFF);
	delay(1000);
	WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
	delay(2000);  						// MUST HAVE DELAY - bug:https://github.com/espressif/arduino-esp32/issues/3906 

  // this line can cause exception errors in core !!!!!!!!!!!!!!!!!!!!!!!11
  WiFi.softAPConfig(AP_IP, AP_IP, AP_subnet);
	delay(500);  						// MUST HAVE DELAY - bug:https://github.com/espressif/arduino-esp32/issues/3906 
	WiFi.softAP(AP_SSID);
	delay(500);  						// MUST HAVE DELAY - bug:https://github.com/espressif/arduino-esp32/issues/3906 
 
  Serial.print(F("Access point setup at "));
  Serial.println(AP_SSID);

  // now connect to camera wifi network
	Serial.print(F("Connecting to Yi camera WiFi network "));
  Serial.print(YI_SSID);
	WiFi.begin(YI_SSID, password);
	int retry = 20;
	while (WiFi.status() != WL_CONNECTED && retry > 0)
  {
    delay(500);
    Serial.print(".");
    retry--;
  }

  if (WiFi.status() == WL_CONNECTED)
	{
  	if (camClient.connect(CamIP, jsonPort))
		{
			Serial.print(F("\r\n... connected to "));
			Serial.print(CamIP);
			Serial.print(":");
			Serial.print(jsonPort);
			Serial.print(F(" - our IP address on cam network: "));
			Serial.println(WiFi.localIP());
		}
		else
			Serial.println(F("\r\n - port connection failed!"));
	}
	else
	  Serial.println(F(" - camera not found."));


	// no connection to camera?
	if (!camClient.connected())
	{
		Serial.println(F("Restarting Access point"));
		// restart the softAP as the wifi scan seems to disconnect it if it doesn't find the camera network!
		WiFi.disconnect();
		WiFi.mode(WIFI_OFF);
		delay(1000);
		WiFi.persistent(false);

		// Begin Access Point
		WiFi.mode(WIFI_AP);
		delay(2000);  						// MUST HAVE DELAY - bug:https://github.com/espressif/arduino-esp32/issues/3906 
		// this lne cause exception erros in core !!!!!!!!!!!!!!!!!!!!!!!11
		WiFi.softAPConfig(AP_IP, AP_IP, AP_subnet);
		delay(200);  						// MUST HAVE DELAY - bug:https://github.com/espressif/arduino-esp32/issues/3906 
		WiFi.softAP(AP_SSID);
		delay(200);  						// MUST HAVE DELAY - bug:https://github.com/espressif/arduino-esp32/issues/3906 
	}
	
  // Connected to WiFi
  Serial.print(F("IP address for Access Point network is "));
  Serial.println(WiFi.softAPIP());


  // now setup webservers - appears at AP and STA IPs!
	// https://github.com/me-no-dev/ESPAsyncWebServer#rewrites-and-how-do-they-work cdocd for lib!
	// Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
		if (request->hasParam("debug"))
			toggleDebug();

		AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html);
		response->addHeader("Cache-Control", "no-cache, max-age=0, must-revalidate, no-store"); // to make sure page gets reloaded after a download!
		request->send(response);
//		request->send_P(200, "text/html", index_html);
	});

	// log mode is set by GET call for log file and cleared by POST call!
  server.on("/logon", HTTP_GET, [](AsyncWebServerRequest *request){
	
		/* older Chrome (Android 8) sends two GET requests on a download - second one to close connection after download
			or maybe to change encoding?
		HEADER[Host]: 10.1.1.1
		HEADER[Connection]: keep-alive
		HEADER[User-Agent]: Mozilla/5.0 (Linux; Android 8.1; 8227L_demo Build/MRA58K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/50.0.2661.89 Safari/537.36
		HEADER[Accept-Encoding]: gzip, deflate, sdch
		HEADER[Accept-Language]: en-US,en;q=0.8
		Web Log ON

		Thrust : 0 lbf
		HEADER[User-Agent]: Mozilla/5.0 (Linux; Android 8.1; 8227L_demo Build/MRA58K) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/50.0.2661.89 Safari/537.36
		HEADER[Cookie]: 
		HEADER[Accept-Encoding]: identity
		HEADER[Connection]: close
		HEADER[Host]: 10.1.1.1
		Web Log ON

		Thrust : 0 lbf

		Safari downloads the file OK but stops scripts running on the page 9you can;t see the download has started or stop it unless page is refreshed).
		A javascript bodge in Strings.ino fixes this.
		
		*/
		webLogMode = true; // set log mode
		if (debug)
		{
			Serial.println("Web Log ON\r\n");
			int headers = request->headers();
			int i;
			for(i=0;i<headers;i++){
				AsyncWebHeader* h = request->getHeader(i);
				Serial.printf("HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
			}
		}			

		webLogStartTime = millis(); // reset start timer
		PcalMode = AcalMode = false; // make sure calibration is off!

		bufCounter = writePtr = readPtr = 0; // empty chunk buffer
		sampleBufferWrite(csvHeader);  // copy csv header to buffer to get sent 1st time!

		// CHUNKED transfer version:
		AsyncWebServerResponse *response = request->beginChunkedResponse
		(
			"text/plain", 
			[](uint8_t *buffer, size_t maxLen, size_t index) -> size_t 
			{	// callback function to retrieve another chunk of data
				// Write up to "maxLen" bytes into "buffer" and return the amount written.
				// index equals the amount of bytes that have been already sent
				
				// NOTE - if this callback returns response_try_again it then won't
				// get called again by the lib for 0.5secs.  We need to buffer the sample
				// data to avoid losing any.
				
				// keep sending data until our buffer is empty AND logmode is turned off
				// to avoid terminating before last sample line has been sent..
				
				if (bufCounter > 0) // data avail.?
				{
					int count = min((int)bufCounter, (int)maxLen);	// only send up to maxLen bytes
					for (int i=0; i < count; i++) 
					{
						buffer[i] = sampleBuffer[readPtr++];
						readPtr %= maxBufLen; //wrap read ptr
						bufCounter--;
					}
					return count;
				}
				
				else if (webLogMode || waitForCamFileName) // buffer empty - have we stopped logging?
					return RESPONSE_TRY_AGAIN; // no, no data available OR waiting for vidfilename
				else	
					return 0; // log finished & no data so return 0 to close download at client
			}
		);

		String fname = "attachment; filename=\"log_" + String(gotGPS ? (GPS.time.value() / 100) : millis()) + ".csv\"";
		response->addHeader("Content-Disposition", fname.c_str());
		request->send(response);
  });

	// change server state via a POST
	server.on("/", HTTP_POST, [](AsyncWebServerRequest *request){
		if (request->hasParam("pcal", true))
		{
			if (!PcalMode)
				setPcalMode();
		}
		else if (request->hasParam("acal", true))
		{
			if (!AcalMode)
				setAcalMode();
		}
		else if (request->hasParam("rst", true))
		{
			restartTimer = millis();  //start timer
			request->send_P(200, "text/html", reset_html);
			return;
		}
		else if (request->hasParam("about", true))
		{
			request->send_P(200, "text/html", about_html);
			return;
		}
		else if (request->hasParam("logoff", true))
		{
			if (webLogMode && camClient.connected() && vidFileName == "") // was logging & got camera and no filename at end of record op?
				camFileNameTimer = millis(); // wait until camera supplies vidfilename
			webLogMode = false; // stop log mode
			if (debug)
				Serial.println(F("Web Log OFF\r\n"));
		}
		request->redirect("http://10.1.1.1"); // always redirect to a GET after a post!
  });
  
	// files supplied as part of web page
	// PNG images
	server.on("/hover_rear.png", HTTP_GET, [](AsyncWebServerRequest *request){
		AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", hover_rear, sizeof_hover_rear);
		request->send(response);
  });
	server.on("/hover_side.png", HTTP_GET, [](AsyncWebServerRequest *request){
		AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", hover_side, sizeof_hover_side);
		request->send(response);
  });
	server.on("/hover_top.png", HTTP_GET, [](AsyncWebServerRequest *request){
		AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", hover_top, sizeof_hover_top);
		request->send(response);
  });


  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client)
		{
			if (debug)
				Serial.println(F("Client connected!"));
			if(client->lastId()){
				Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
			}
			// send event with message "hello!", id current millis
			// and set reconnect delay to 1 second
			client->send("hello!", NULL, millis(), 1000);
			// Ptr = 0; // empty buffer
		}
	);

  server.addHandler(&events); // handler above is for /events reqs
  
  server.begin();

  Serial.println(F("HTTP server started on port 80"));
 
  Serial.println(F("\r\n Listening on WiFi for connections ...\r\n"));

	current_sensor_pin = pin_pressBag; // start first  conversion
//	adcAttachPin(pin_pressBag); 
//	adcStart(pin_pressBag); // start conversion
	
	sampleTimer = millis(); // start sampling data

}


//=============================
//				MAIN LOOP
//=============================

// the loop routine runs over and over again forever:
void loop() {

	if (restartTimer != 0) // restart active!
		if ((millis() - restartTimer) > restartTimeOut)
			ESP.restart();

	gpsUpdate(); // need to do this every loop to ensure we don't miss GPS sentence data

	camUpdate();

	serCheck();

	updateSensors();

 	// calibrate if requested AND after a delay (to allow for proper menu update!)
	if ((PcalMode || AcalMode) && ((millis()-calDelayTimer) > preCalDelay))
	{
		if (PcalMode)
			Pcalibrate();
		else 
			Acalibrate();
	}

}
