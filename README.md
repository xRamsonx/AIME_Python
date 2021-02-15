# AIME_Python
Python documents for the AIME Project
## Requirements:
- numpy
- matplotlib
- opencv-python
- Adafruit_AMG88xx
- mavsdk
- mavsdk-server
(optional)
- gstream1.0 (for streaming the data to QGC)

## Usage:
If necessary calibrate the IR Sensor with calibrate_ir.py. For that you will need 2 candles and a ruler with cm. The script will guide you through the process.\
Otherwise run readIR.py or readIRdata.py to stream the IR image to an location set inside the script.
Before using trackNumberOfPeople.py make sure to set up the MAVSDK server. If it's your first time, take a look here: \ 
https://www.hackster.io/mdobrea/c-and-python-interface-management-application-for-fmuk66-6dd935\
trackNumberOfPeople.py will use the telemetry data of your drone to calculate the number of people in an area and their relative position. The data will be streamed to an location set within the script.
Have Fun!
 
