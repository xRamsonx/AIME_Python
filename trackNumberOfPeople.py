#!/usr/bin/env python3

import asyncio
from mavsdk import System
import numpy as np
import cv2
import time
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__),'..')) # this is done for the AMG88xx folder (you may have to rewrite this to include the path of your AMG file)
from Adafruit_AMG88xx import Adafruit_AMG88xx
import matplotlib.pyplot as plt

def init():
    #Get callibration data
    global H0
    global a
    a,H0=np.loadtxt('data/calIR.csv')
    
    
    #initalise IR Sensor
    global sensor
    sensor = Adafruit_AMG88xx(1)
    # wait for AMG to boot
    time.sleep(0.1)

    # preallocating variables for IR sensor
    global norm_pix
    norm_pix = []
    global cal_vec
    cal_vec = []
    global kk
    kk = 0
    global cal_size
    cal_size = 10 # size of calibration
    global cal_pix
    cal_pix = []
    plt.ion()
    
    #define dataoutput
    global out 
    img = cv2.imread("data/IR.png")
    height, width, layers = img.shape
    size = (width,height)
    ##Stream to avi file
    out=cv2.VideoWriter('data/IR.avi',cv2.VideoWriter_fourcc(*'XVID'), 15, size)
    ##Stream to gstream
    #gstreamdata="appsrc ! videoconvert! vpuenc_h264 bitrate=500 ! rtph264pay ! udpsink host=192.168.0.173 port=5600 sync=false"
    #out=cv2.VideoWriter(gstreamdata,cv2.CAP_GSTREAMER,0, 20, size, True)
    return

async def run():
    # Init the drone
    drone = System()
    print("connecting to drone")
    await drone.connect(system_address="serial://dev/ttymxc2:921600")
    print("connected")
    # Start the tasks
    asyncio.ensure_future(get_position(drone))
    asyncio.ensure_future(get_center_of_people())
    asyncio.ensure_future(streamdata())
    asyncio.ensure_future(get_IR_data())

async def get_position(drone):
    global d_pos
    global height
    height=H0
    global has_pos
    has_pos=False
    async for position in drone.telemetry.position():
        print(position)
        d_pos=position
        height=position.relative_altitude_m()
        has_pos=True
        
async def print_center_of_people():
    while(1):
        time.sleep(1)
        if(has_pos):
            if(has_heatsignal):
                for i in range(IRpos):
                    print("Found target at:",pxtocm(IRpos[i]),"cm")
            else:
                    print("No Target found")
        else:	
            print("GPS signal not sufficient to get real position")
            if(has_pos):
                print("Found target at:",IRpos[i],"px")
            else:    
                print("No target found")
async def get_IR_data():
        while(1):
            global kk
            time.sleep(0.1)
            # calibration procedure #
            if kk==0: 
                print("Sensor should have clear path to calibrate against environment")
                graph = plt.imshow(np.reshape(np.repeat(0,64),(8,8)),cmap=plt.cm.hot,interpolation='lanczos')
                #plt.colorbar()
                plt.axis('off')
                plt.clim(1,8) # can set these limits to desired range or min/max of current sensor reading
            norm_pix = sensor.readPixels() # read pixels
            if kk<cal_size+1:
                kk+=1
            if kk==1:
                cal_vec = norm_pix
                continue
            elif kk<=cal_size:
                for xx in range(0,len(norm_pix)):
                    cal_vec[xx]+=norm_pix[xx]
                    if kk==cal_size:
                        cal_vec[xx] = cal_vec[xx]/cal_size
                continue
            else:
                [cal_pix.append(norm_pix[x]-cal_vec[x]) for x in range(0,len(norm_pix))]
                if min(cal_pix)<0:
                    for y in range(0,len(cal_pix)):
                        cal_pix[y]+=abs(min(cal_pix))
            # Save IR picture as png
            graph.set_data(np.reshape(cal_pix,(8,8))) # updates heat map in 'real-time'
            plt.draw() # plots updated heat map
            global frame
            plt.savefig("data/IR.png", bbox_inches='tight')

async def get_center_of_people():
    while(1):
        time.sleep(0.1)
        global frame
        frame = cv2.imread('data/IR.png')#load current IR picture
        #resize to make processing faster
        frame = cv2.resize(frame, (600,600), interpolation = cv2.INTER_AREA)
        #Blur image to make contours easier to find
        radius = 10
        ksize = int(2 * round(radius) + 1)
        image = cv2.blur(frame, (ksize, ksize))

        # Convert to HSV color for filtering
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out all colors except red
        lower_red = np.array([0,87,211])
        upper_red = np.array([36,255,255])
        #Create binary mask to detect objects/contours
        mask = cv2.inRange(hsv, lower_red, upper_red)


        # Find contours and sort using contour area
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_poly = [None]*len(cnts)
        boundRect = [None]*len(cnts)
        centers = [None]*len(cnts)
        radius = [None]*len(cnts)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
        global has_heatsignal
        if(len(cnts)==0):
            has_heatsignal=False
            continue
        else: has_heatsignal=True
        i=0
        for i, c in enumerate(cnts):
            contours_poly[i] = cv2.approxPolyDP(c, 3, True) 
            boundRect[i] = cv2.boundingRect(contours_poly[i])
            centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
        global area
        area=np.array([])
        for i, c in enumerate(cnts):
            #calculate the area of each contour
            np.append(area,[cv2.contourArea(c)])	    
        global IRpos
        IRpos=np.array([])

        for i, c in enumerate(cnts):
            # Once we hit smaller contours, stop the loop
            if(cv2.contourArea(c) < 100):
                break
            x,y,w,h = cv2.boundingRect(c)
            color = (0,255,0)
            cv2.circle(frame, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), color, 2)
            np.append(IRpos,[int(centers[i][0]),int(centers[i][1])])
            cv2.minEnclosingCircle( contours_poly[i], centers[i], radius[i] )
            font = cv2.FONT_HERSHEY_SIMPLEX
            numberofpeople=get_numberofpeope(area(i))
            cv2.putText(frame,f'{numberofpeople} people', (x,y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

async def streamdata():
    while(1):
        if(has_heatsignal):
            cv2.imwrite('data/IRdata.png', frame)
            out.write(frame)

def getAreaincm2(pxarea):
    if(not has_pos): #assume height=H0  
        out_area=pxarea*a**2  
    else: #use callibration to calculate the area  
        out_area=pxarea*(a*H0/height)**2
    return out_area

def get_numberofpeope(pxarea):    
    return int(getAreaincm2(pxarea)/250)

def pxtocm(pxdata):
    while(not has_pos):#wait untill position is available
        time.sleep(0.5)
    return pxdata*a*H0/height

if __name__ == "__main__":
    try:
        init()
        # Start the main function
        asyncio.ensure_future(run())

        # Runs the event loop until the program is canceled with e.g. CTRL-C
        asyncio.get_event_loop().run_forever()
    
    except KeyboardInterrupt:
        print("CTRL-C: Program Stopping via Keyboard Interrupt...")

    finally:
        out.close()
        print("Exiting Loop")       

