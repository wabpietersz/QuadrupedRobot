#serial libs
import serial
import time
#sonar libs
import RPi.GPIO as GPIO
import time

#establishing serial caommunication with the arduino
ser = serial.Serial('/dev/ttyUSB0', 9600)

##<--sonar sensor outs-->

    #sonar sensor code
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

# lf= distance to front
# lr= distance to right
# ll= distance to left

##<--picam outs-->

    #picam object detection code
# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

from picamera.array import PiRGBArray
from picamera import PiCamera

FRAME_WIDTH = 600

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
pts = deque(maxlen=64)

# current position of the green object
targetPosition = -1

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.vflip=True
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
for f in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    frame = f.array

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=FRAME_WIDTH)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # decide the current position of the green object
        targetPosition = int(center[0] / (FRAME_WIDTH / 5))
        cv2.putText(frame, str(targetPosition), (center[0] + 10, center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 255), thickness=2, lineType=cv2.LINE_AA)
    else:
        targetPosition = -1

    # update the points queue
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    #   #   #   #   #   #
    # 0 # 1 # 2 # 3 # 4 #
    #   #   #   #   #   #

    ############## Call your robot code here. This part is called continuously ##############

    # turnlimit
    turnLimit = 0

        # calculating the distance
    threshhold = 10  #threshhold distance
    distancef = distance()  # forward distance
    if (targetPosition>=0):                              #object is available
            if (targetPosition == 2):                    #object in the center
                    if (threshhold < distancef):         #going to a threshhold distance
                        ser.write(str.encode('1'))       #forward
                        print ("move forward")
                    #else:
                            #stop
            else:
                    if (targetPosition == 1 or 0):      #object in the right
                        ser.write(str.encode('4'))
                        print ("turn right")
                    if (targetPosition == 3 or 4):      #object in the left
                        ser.write(str.encode('3'))
                        print ("turn left")


                    # if (objArea == 1):
                    #     ser.write(4)
                    # if (objArea == 1):
                    #     ser.write(4)
                    #
                    # elif (d==d1):
                    #         #sidehalfturn
    #else:
            #if (turnLimit<4):      # turning and searching for the object around
                    #turnleft
                    #turnLimit++
            #else:
                    #pathing

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# close all windows
cv2.destroyAllWindows()
