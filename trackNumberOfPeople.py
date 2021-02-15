#!/usr/bin/env python3
import asyncio
from mavsdk import System
import numpy as np
import cv2 #image 
import time
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__),'..')) # this is done for the AMG88xx folder (you may have to rewrite this to include the path of your AMG file)
from Adafruit_AMG88xx import Adafruit_AMG88xx #to read out the IR-sensor
import matplotlib.pyplot as plt #to create heatmap from IR-Sensordata 

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
    asyncio.ensure_future(print_center_of_people())


###define async-functions
##subscribe to drone position
async def get_position(drone):
    global d_pos
    global height
    height=H0
    global has_pos
    has_pos=False #to not run into errors while no position was found e.g. indoor
    async for position in drone.telemetry.position():
        print(position)
        d_pos=position
        height=position.relative_altitude_m()
        has_pos=True

##define output
async def print_center_of_people():
    while(1):
        time.sleep(1)#slows the output
        if(has_pos):
            if(has_heatsignal):
                for i in range(IRpos):
                    print("Found target at:",pxtocm(IRpos[i]),"cm")
            else:
                    print("No Target found")
        else:	
            print("GPS signal not sufficient to get real position")
            if(has_pos):
                for i in range(IRpos):
                    print("Found target at:",IRpos[i],"px")
            else:    
                print("No target found")
   
##get IR-data continously             
async def get_IR_data():
    global kk
    while(1):
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
        plt.savefig("data/IR.png", bbox_inches='tight')
        
##define image recognition function
async def get_center_of_people():
    global has_heatsignal
    global IRpos
    global area
    while(1):
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
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
        
        if(len(cnts)==0):
            has_heatsignal=False
            continue
        else: has_heatsignal=True
        i=0
        area=np.array([])       
        IRpos=np.array([])
        for i, c in enumerate(cnts):
            # Once we hit smaller contours, stop the loop
            if(cv2.contourArea(c) < 100):
                break
            np.append(area,[cv2.contourArea(c)])
            x,y,w,h = cv2.boundingRect(c)
            color = (0,255,0)
            contours_poly = cv2.approxPolyDP(c, 3, True) 
            centers, radius = cv2.minEnclosingCircle(contours_poly)
            cv2.circle(frame, (int(centers[0]), int(centers[1])), int(radius), color, 2) #draw circular contour
            np.append(IRpos,[int(centers[0]),int(centers[1])])
            font = cv2.FONT_HERSHEY_SIMPLEX
            numberofpeople=get_numberofpeople(cv2.contourArea(c))
            cv2.putText(frame,f'{numberofpeople} people', (x,y), font, 1, (0, 255, 0), 2, cv2.LINE_AA) #add text to circle
        cv2.imwrite('data/IRdata.png', frame)
        
#continously stream data to videosink
async def streamdata():
    while(1):
        if(has_heatsignal):
            img= cv2.imread('data/IRdata.png')             
            out.write(img)



###some important functions:
#convert pixel area to real area
def get_Areaincm2(pxarea):
    if(not has_pos): #assume height=H0  
        out_area=pxarea*a**2  
    else: #use callibration to calculate the area  
        out_area=pxarea*(a*height/H0)**2
    return out_area

#estimate number of people with a given headsice
def get_numberofpeople(pxarea):    
    headsize=250 #in cm
    return np.round(get_Areaincm2(pxarea)/headsize,1)

#conver pixel length to cm
def pxtocm(pxdata):
    while(not has_pos):#wait untill position is available
        time.sleep(0.5)
    return pxdata*a*height/H0 #intercept theorem R0/R=H0/H



###main loop
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
        out.release()
        print("Exiting Loop")       
