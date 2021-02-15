#!usr/bin/python
import sys
import os.path
import cv2
sys.path.append(os.path.join(os.path.dirname(__file__),'..')) # this is done for the AMG88xx folder (you may have to rewrite this to include the path of your AMG file)
from Adafruit_AMG88xx import Adafruit_AMG88xx
from time import sleep
import time
#import matplotlib as mpl
#mpl.use('TKAgg')
#import matplotlib as mpl
#mpl.use('tkagg') # to enable real-time plotting in Raspberry Pi
import matplotlib.pyplot as plt
import numpy as np

def center_of_people():
    frame = cv2.imread('data/calIR.png')
    # Resize to make processing faster
    frame = cv2.resize(frame, (600,600), interpolation = cv2.INTER_AREA)
    # Blur image to make contours easier to find
    radius = 10
    ksize = int(2 * round(radius) + 1)
    image = cv2.blur(frame, (ksize, ksize))

    # Convert to HSV color for filtering
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Filter out all colors except red
    lower_red = np.array([0,87,211])
    upper_red = np.array([36,255,255])
    # Create binary mask to detect objects/contours
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours and sort using contour area
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_poly = [None]*len(cnts)
    boundRect = [None]*len(cnts)
    centers = [None]*len(cnts)
    radius = [None]*len(cnts)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    i=0
    for i, c in enumerate(cnts):
	    contours_poly[i] = cv2.approxPolyDP(c, 3, True) 
	    boundRect[i] = cv2.boundingRect(contours_poly[i])
	    centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
    IRpos=[]
    for i, c in enumerate(cnts):
 	   # Once we hit smaller contours, stop the loop
        if(cv2.contourArea(c) < 100):
            break
	   # Draw bounding box around contours and write "Candles" text
        color = (0,255,0)
        cv2.circle(frame, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), color, 2)
        IRpos.append([int(centers[i][0]),int(centers[i][1])])
        cv2.minEnclosingCircle( contours_poly[i], centers[i], radius[i] )
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame,'Candle', (x,y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    # Write images to disk for debugging
    #cv2.imwrite('data/thresh.png', mask)
    cv2.imwrite('data/IRCalib.png', frame)
    return IRpos
        
sensor = Adafruit_AMG88xx(1)
# wait for AMG to boot
sleep(0.1)

# preallocating variables
norm_pix = []
cal_vec = []
kk = 0
cal_size = 10 # size of calibration
cal_pix = []
time_prev = time.time() # time for analyzing time between plot updates
Calibration=True
plt.ion()

try:
    print("For initial calibration use two candles and put one directly under the IR camera and one in a distance d away from it. Then measure the distance d as well as the height of the camera. Don't light the candles yet!")
    print("When done press Enter to continue")
    input()
    while(1):
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
        # Moving Pixel Plot #
        #print(np.reshape(cal_pix,(8,8))) # this helps view the output to ensure the plot is correct
        graph.set_data(np.reshape(cal_pix,(8,8))) # updates heat map in 'real-time'
        plt.draw() # plots updated heat map
        plt.savefig("data/calIR.png", bbox_inches='tight')
        cal_pix = [] # off-load variable for next reading
        print(time.time()-time_prev) # prints out time between plot updates
        time_prev = time.time()
        if(Calibration):
            print("Now light the candles")
            print("When done press Enter to continue")
            input()
            Calibration=False
        else:
            break
    time.sleep(0.2)
    dv2=np.array(center_of_people())
    print("What is the height? (in cm) e.g 50 for 50cm")
    H=float(input())
    print("What is the distance? (in cm) e.g 10 for 10cm")
    d=float(input())
    print("Calculating the calibration data")

    d2=np.linalg.norm(dv2[1]-dv2[0])
    a= d/d2 #1px=a cm at an height of H
    np.savetxt("data/calIR.csv", [a,H], delimiter=",")
    
    
except KeyboardInterrupt:
        print("CTRL-C: Program Stopping via Keyboard Interrupt...")

finally:
        print("Exiting Loop")       
