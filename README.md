# Realtime Telemetry Data using WiFi
## Description
Code (running on an Espressif ESP32 platform) supplies realtime data via WiFi from various sensors: pressure (3 x MXPV67002 +/-2KPa sensors), XYZ accelerometer & gyro (MPU6050) and a GPS module.  Log data can be viwed (and saved) via a web page OR via a USB serial interface (@ 115200baud).  Data sampling rate is currentlyset at 50Hz (0.3Hz on the serial interface).  An optional load cell can also be connected to read thrust - it isn't logged but displayed in real time.  Either of the connected devices (web page, serial port) can stream data independently.

Clearly, this code, as is, will have very limited application.  However, the underlying functions (eg. data sampling using chunked file transfer) could be used as a base for other data sampling applications.

During sensor calibration, logging is disabled on all devices.  Make sure the unit is level, oriented correctly (front, l/r, etc.) and that the pressure sensors are only subject to ambient pressure!  Calibration values are saved between power cycles.

Connect your client browser/device to the WiFi Access Point **Hovercraft_Telemetry**.  A web page is then available at IP address 10.1.1.1:80.  This device also uses a TelNet port on 7878 to control an optional Xaomi Yi camera (video recording is turned on/off with Logging).  Note that the camera must be turned on and WiFi enabled (camera side button) BEFORE this device is powered up or reset (so it can connect to the camera).
			
### Data is CSV formattted
*Sample time (mSec)*

*Bow pressure (Pa)*

*Bag*

*Cushion*  

*Pitch (raw accelerometer value)*  

*Roll*  

*Yaw*

*Latitude*

*Longitude*

*Speed (100ths of a knot)*

*Time (HHMMSS)*

Log data files can be uploaded and viewed interactively [HERE](http://cirtech.co.uk/viewer/plotter.php">http://cirtech.co.uk/viewer/plotter.php)


## Features:
Allows unlimited data collection in real time using chunked file download supported by HTTP1.1.  A "normal" file download is initiated and the code streams data in "chunks" until instructed to stop.  Tested Web browsers will keep the download open indefinetly until either they time out (no data for > 60secs) OR the download is termniated by the ESP32 via a web page.  Data sampling rate has been tested at 50Hz (multiple samples per packet) - ultimately, the WiFi maximum transfer rate will limit the sampling rate.

File downloaded is simple CSV format ready to be viewed/manipulated by any Spreadsheet pApp.

When not saving data to a file, it is streamed continuously in the background for display on the web page using Server Sent Events.  


## System Requirements 
Esp32 board.


## Dependencies

ESPAsyncWebServer
ArduinoJson
TinyGPS++
MPU6050_tockn

 
