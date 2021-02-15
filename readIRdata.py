#!usr/bin/python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__),'..')) # this is done for the AMG88xx folder (you may have to rewrite this to include the path of your AMG file)
from Adafruit_AMG88xx import Adafruit_AMG88xx
from time import sleep
import time
import matplotlib.pyplot as plt
import numpy as np	
import cv2

###initialise
#Load calibration data
global H0
global a
a,H0=np.loadtxt('data/calIR.csv')
#load sensor
sensor = Adafruit_AMG88xx(1)
# wait for AMG to boot
sleep(0.1)
# preallocating variables
norm_pix = [] #load IR data
cal_vec = [] #interpolated IR data
kk = 0 #counter for th calibration
cal_size = 10 # size of calibration
cal_pix = []
time_prev = time.time() # time for analyzing time between plot updates
plt.ion() #initialise matplotlib
img_array = [] 

#Setup videosink
global out
img = cv2.imread("data/IRdata.png")
height, width, layers = img.shape
size = (width,height)
##Stream to avi file
out=cv2.VideoWriter('data/IRdata.avi',cv2.VideoWriter_fourcc(*'DIVX'), 2, size)
##Stream to gstream
#gstreamdata="appsrc ! videoconvert! vpuenc_h264 bitrate=500 ! rtph264pay ! udpsink host=192.168.0.173 port=5600 sync=false"
#out=cv2.VideoWriter(gstreamdata,cv2.CAP_GSTREAMER,0, 20, size, True)



###define image recognition function
def get_center_of_people():
    frame = cv2.imread('data/IR.png')
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
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    
    for i, c in enumerate(cnts):
 	   # Once we hit smaller contours, stop the loop
        if(cv2.contourArea(c) < 100):
            break
        print('contact!')
	   # Draw bounding box around contours and write "Candles" text
        x,y,w,h = cv2.boundingRect(c) #get counter data
        color = (0,255,0)
        #estimate circular dimension
        contours_poly = cv2.approxPolyDP(c, 3, True)
        centers, radius = cv2.minEnclosingCircle(contours_poly)
        cv2.circle(frame, (int(centers[0]), int(centers[1])), int(radius), color, 2) #draw circular contour
        font = cv2.FONT_HERSHEY_SIMPLEX 
        headsize=120 #in cm**2
        numberofpeople=np.round(cv2.contourArea(c)*a**2/headsize,1) #estimate number of object with a given headsize 
        cv2.putText(frame,f'{numberofpeople} candles', (x,y), font, 1, (0, 255, 0), 2, cv2.LINE_AA) #draw text
    cv2.imwrite('data/IRdata.png', frame) #save altered image
    return 


###main loop
try:
    while(1):
        # calibration procedure #
        if kk==0:
            print("Sensor should have clear path to calibrate against environment")
            graph = plt.imshow(np.reshape(np.repeat(0,64),(8,8)),cmap=plt.cm.hot,interpolation='lanczos')
            plt.axis('off')
            plt.clim(1,8) # can set these limits to desired range or min/max of current sensor reading
            plt.draw()
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
                    
        graph.set_data(np.reshape(cal_pix,(8,8)))#update heatmap
        plt.draw() # plots updated heat map
        plt.savefig("data/IR.png", bbox_inches='tight') # save heatmap
        cal_pix = [] # off-load variable for next reading
        get_center_of_people() #manipulate saved image
        img = cv2.imread("data/IRdata.png") #load it again
        out.write(img) #send img to videosink

                
except KeyboardInterrupt:
        print("CTRL-C: Program Stopping via Keyboard Interrupt...")

finally:
        out.release() #to make sure nothing breaks down
        print("Exiting Loop") 
