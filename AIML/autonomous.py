# Python 2/3 compatibility
from __future__ import print_function

import time

import cv2 as cv

from .obstacle_avoidance import ObstacleAvoidance

CROP_WIDTH = 544
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360

CAMERA_LEFT = 0
CAMERA_RIGHT = 1
print("Camera Index", CAMERA_LEFT, CAMERA_RIGHT)


class Autonomous(object):
    def __init__(self):
        self.is_active = True

        self.vsL = cv.VideoCapture(CAMERA_LEFT)
        self.vsR = cv.VideoCapture(CAMERA_RIGHT)

        # Set the resolution
        self.vsL.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.vsL.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.vsR.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.vsR.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        # Use MJPEG to avoid overloading the USB 2.0 bus at this resolution
        self.vsL.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
        self.vsR.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))

    # Discard the edges
    def cropHorizontal(self, image):
        return image[:,
               int((CAMERA_WIDTH - CROP_WIDTH) / 2):
               int(CROP_WIDTH + (CAMERA_WIDTH - CROP_WIDTH) / 2)]

    # Start Autonomous Mode
    def start(self):
        obstacle_avoidance = ObstacleAvoidance(CROP_WIDTH, CAMERA_HEIGHT)

        time.sleep(2.0)

        while self.is_active:
            # time.sleep(10)

            start = time.time()

            retL, frameL = self.vsL.read()
            retR, frameR = self.vsR.read()

            # distortion in the left and right edges prevents a good calibration, so
            imgL = self.cropHorizontal(frameL)
            imgR = self.cropHorizontal(frameR)

            # Find available targets
            available_targets = obstacle_avoidance.find_targets(imgL, imgR)
            print("Available Targets", len(available_targets))

            end = time.time()
            print("Total time: {}".format(end - start))

            key = cv.waitKey(1) & 0xFF

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

        cv.destroyAllWindows()

    # Stop Autonomous Mode
    def stop(self):
        self.is_active = False
