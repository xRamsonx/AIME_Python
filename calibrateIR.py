# -*- coding: utf-8 -*-
from ir_image_processing import IRImage, IRSensor

###main function


if __name__ == "__main__":
    try:
       sensor = IRSensor()
       sensor.callibrate()
        
    except KeyboardInterrupt:
        print("CTRL-C: Program Stopping via Keyboard Interrupt...")

    finally:
        print("Exiting Loop") 

