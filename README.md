# AIME_Python
Python documents for the AIME Project
## About:
This project aims to build a passive system to anonymously track the number of people in a given area using IR-camera.
## Requirements:
- numpy
- matplotlib
- opencv-python
- Adafruit_AMG88xx
- mavsdk
- mavsdk-server
(optional)
- gstream1.0 (for streaming the data to QGC)

## Inspiration:
The image recognition functions were greatly inspired by Landon Haugh (https://nxp.gitbook.io/8mmnavq/navq-developer-guide/software-support/opencv). \
The mavsdk-python usage was inspired by Dobrea Dan Marius (https://www.hackster.io/mdobrea)

## Usage:
If necessary calibrate the IR Sensor with calibrate_ir.py. For that you will need 2 candles and a ruler with cm. The script will guide you through the process. \
Otherwise run readIR.py or readIRdata.py to stream the IR image to an location set inside the script.
Before using trackNumberOfPeople.py make sure to set up the MAVSDK server. If it's your first time, take a look here: \
https://www.hackster.io/mdobrea/c-and-python-interface-management-application-for-fmuk66-6dd935 \
trackNumberOfPeople.py will use the telemetry data of your drone to calculate the number of people in an area and their relative position. The data will be streamed to a location set within the script. \

## Troubleshooting:
Using Adafruit_AMG88xx library on NavQ will raise the error "no standard i2c bus found" to fix this I did sudo nano to the stated file and added a 'return 1' in the get_Bus function in front of the error message. \ 
This implies you are using NavQ's i2c bus 1. To check this u can use i2cdetect from the i2c-tools ubuntu package. 

# Have Fun!
 
