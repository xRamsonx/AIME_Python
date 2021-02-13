#!usr/bin/python
import sys
import os.path
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
i=0
plt.ion()

try:
        while(1):
                time.sleep(1)
                i+=1
                if(i>10): 
                        i=0
                # calibration procedure #
                if kk==0:
                        print("Sensor should have clear path to calibrate against environment")
                        graph = plt.imshow(np.reshape(np.repeat(0,64),(8,8)),cmap=plt.cm.hot,interpolation='lanczos')
                        #plt.colorbar()
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
                # Moving Pixel Plot #
                #print(np.reshape(cal_pix,(8,8))) # this helps view the output to ensure the plot is correct
                #np.savetxt("data/IR.csv", np.reshape(cal_pix,(8,8)), delimiter=",")
                graph.set_data(np.reshape(cal_pix,(8,8))) # updates heat map in 'real-time'
                
                plt.draw() # plots updated heat map
                plt.savefig("data/IR.png", bbox_inches='tight')
                cal_pix = [] # off-load variable for next reading
                #print(time.time()-time_prev) # prints out time between plot updates
                #time_prev = time.time()
                
except KeyboardInterrupt:
        print("CTRL-C: Program Stopping via Keyboard Interrupt...")

finally:
        print("Exiting Loop")       
