# -*- coding: utf-8 -*-
#!usr/bin/python
import sys
import os.path
from time import sleep

import cv2
from Adafruit_AMG88xx import Adafruit_AMG88xx
import matplotlib.pyplot as plt
import numpy as np

# this is done for the AMG88xx folder
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))


class IRImage:
    """
    A class for image recognition tasks on an image.

    Attributes
    ----------
    frame : numpy.ndarray
        The image represented as a NumPy array.

    Methods
    -------
    center_of_people()
        This function detects objects/contours of red color in the frame,
        estimates the circular dimension,
        and returns the positions of centers of the contours.

    write_to_disk(picture_name)
        This function writes the current frame to disk as a PNG image
        with the specified filename in the "data" directory.

    Parameters
    ----------
    picture_name : str
        The name of the picture file to be loaded, without the file extension.
    """
    def __init__(self, picture_name):
        frame = cv2.imread("data/" + picture_name + ".png")
        # Resize to make processing faster
        self.frame = cv2.resize(frame, (600, 600), interpolation=cv2.INTER_AREA)

    ###define image recognition function
    def center_of_people(self):
        """
        This function detects objects/contours of red color in the frame,
        estimates the circular dimension, and
        returns the positions of centers of the contours.
        -------

        Returns
        -------
            pos
                A list of positions of centers of the detected contours
        """
        radius = 10
        ksize = int(2 * round(radius) + 1)
        # Blur image to make contours easier to find
        image = cv2.blur(self.frame, (ksize, ksize))

        # Convert to HSV color for filtering
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Filter out all colors except red
        lower_red = np.array([0, 87, 211])
        upper_red = np.array([36, 255, 255])
        # Create binary mask to detect objects/contours
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours and sort using contour area
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
        pos = []
        for contour in cnts:
            # Once we hit smaller contours, stop the loop
            if cv2.contourArea(contour) < 100:
                break

            x_pos, y_pos, width, height = cv2.boundingRect(contour)  # get counter data
            color = (0, 255, 0)
            # estimate circular dimension
            contours_poly = cv2.approxPolyDP(contour, 3, True)
            centers, radius = cv2.minEnclosingCircle(contours_poly)
            cv2.circle(
                self.frame, (int(centers[0]), int(centers[1])), int(radius), color, 2
            )  # draw circular contour
            pos.append([int(centers[0]), int(centers[1])])
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                self.frame,
                "Candle",
                (x_pos, y_pos),
                font,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )  # draw text
        return pos

    def write_to_disk(self, picture_name):
        """This function writes the current frame to disk as a PNG image
        with the specified filename in the "data" directory.
        ----------
        picture_name (str):
            the desired name of the picture file to be saved to disk,
            without the file extension

        Returns
        -------
            None
        """
        # Write image to disk
        cv2.imwrite("data/" + picture_name + ".png", self.frame)


class IRSensor:
    """
    A class representing an infrared sensor for measuring temperature and distance.
    It uses Adafruit_AMG88xx sensor to read the temperature
    of objects in its field of view
    and allows calibration for distance measurements.

    Attributes
    ----------
    sensor : Adafruit_AMG88xx
        An instance of the Adafruit_AMG88xx sensor class.
    number_callib_pictures : int
        The number of pictures taken during calibration to average for a reference background image.

    Methods
    -------
    take_picture(picture_name):
        Takes a picture with the AMG88xx Sensor and saves it to disk.
    callibrate():
        Calibrates the IR sensor for distance measurements.
    """

    def __init__(self):
        self.sensor = Adafruit_AMG88xx(1)
        # wait for AMG to boot
        sleep(0.1)

        self.number_callib_pictures = 10

        plt.ion()  # initialise matplotlib

    def take_picture(self, picture_name):
        """
        Takes a picture with the AMG88xx Sensor and writes it to disk
        ----------
        picture_name : string
            Name of the picture
        Returns
        -------
        None
        """
        pic = np.array(self.sensor.readPixels())
        plt.imshow(np.reshape(pic, (8, 8)), cmap="hot", interpolation="lanczos")
        plt.draw()  # plots updated heat map
        plt.savefig("data/" + picture_name + ".png", bbox_inches="tight")

    def callibrate(self):
        """Calibrates the IR sensor for distance measurements
        by taking a reference image of the environment without "people" and
        then measuring the distance between two lit candles.

        Instructions:
        1. Use two candles, one placed directly under the IR camera and the
        other placed at a distance d away from it. Do not light the candles yet!
        2. Measure the distance d as well as the height of the camera.
        3. When ready, light the candles and press Enter to continue.
        4. The function takes a number of calibration pictures
        and averages them to get a background reference image.
        5. The function then takes another picture with the lit candles,
        subtracts the background image, and normalizes the result.
        6. The function saves the resulting image to data/calIR.png
        and the calibration data to data/calIR.csv.
        7. Finally, the function prompts the user to input the height and distance values
        and calculates the pixel-to-cm conversion factor.

        Returns
        -------
        None
        """
        print(
            "For initial calibration use two candles and put one directly under the IR camera and one in a distance d away from it. Then measure the distance d as well as the height of the camera. Do not light the candles yet!"
        )
        print("When done press Enter to continue")
        input()
        print("Sensor should have clear path to calibrate against environment")

        background = np.array(self.sensor.readPixels())

        for i in range(0, self.number_callib_pictures):
            sleep(0.1)
            background += np.array(self.sensor.readPixels())
        background = background / self.number_callib_pictures

        cal_pix = []  # off-load variable for next reading

        print("Now light the candles")
        print("When done press Enter to continue")
        input()

        cal_pix = np.array(self.sensor.readPixels())
        cal_pix -= background
        cal_pix += abs(min(cal_pix))

        plt.imshow(np.reshape(cal_pix, (8, 8)), cmap="hot", interpolation="lanczos")
        plt.axis("off")
        plt.clim(
            1, 8
        )  # can set these limits to desired range or min/max of current sensor reading
        plt.savefig("data/calIR.png", bbox_inches="tight")

        ir_image = IRImage("calIR")
        sleep(0.2)
        pixel_pos = np.array(ir_image.center_of_people())
        print("What is the height? (in cm) e.g 50 for 50cm")
        height = float(input())
        print("What is the distance? (in cm) e.g 10 for 10cm")
        distance = float(input())
        print("Calculating the calibration data")

        pixel_distance = np.linalg.norm(pixel_pos[1] - pixel_pos[0])
        pixel_to_cm = distance / pixel_distance  # 1px=a cm at an height of H
        np.savetxt("data/calIR.csv", [pixel_to_cm, height], delimiter=",")
