# USAGE
# python object_movement.py --video object_tracking_example.mp4
# python object_movement.py
# TO TAKE PICTURE IN CONSOLE - fswebcam imageName.jpg

# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import serial
import math
from time import sleep

def sendIndvBytes(byteArray):
    for i in byteArray:

        charValue = chr(i)

        ser.write(charValue.encode())


def distance_to_camera(knownWidth, focalLength, perWidth):
        return (knownWidth * focalLength)/perWidth


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=32,
    help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "orange"
# ball in the HSV color space
orangeLower = (10, 100, 20)
orangeUpper = (25, 255, 255)

# define the lower and upper boundaries of the "red"
redLower = (0,50,50)
redUpper = (10,255,255)

redLower1 = (170,50,50)
redUpper1 = (180,255,255)

# define the lower and upper boundaries of "black"
# rectangle in the HSV color space
blueLower = (100,150,0)
blueUpper = (140,255,255)

greenLower = (65, 60, 60)
greenUpper = (80, 255, 255)

# initialize the list of tracked points, the frame counter,
# and the coordinate deltas
pts = deque(maxlen=args["buffer"])
counter = 0

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    vs = VideoStream(src=0).start()

# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

def find_dropOff(image):

        dBlurred = cv2.GaussianBlur(image, (5,5), 0)
        dHsv = cv2.cvtColor(dBlurred, cv2.COLOR_BGR2HSV)
        #dMask0 = cv2.inRange(dHsv, redLower, redUpper)
        #dMask1 = cv2.inRange(dHsv, redLower1, redUpper1)
        #xMask = dMask0 + dMask1
        xMask = cv2.inRange(dHsv, greenLower, greenUpper)
        xMask = cv2.erode(xMask, None, iterations=2)
        xMask = cv2.dilate(xMask, None, iterations=2)
        cnts = cv2.findContours(xMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        c = max(cnts, key = cv2.contourArea)

        return cv2.minAreaRect(c)


def find_marker(image):

    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    c = max(cnts, key=cv2.contourArea)

    return cv2.minEnclosingCircle(c)


KNOWN_DISTANCE = 9
KNOWN_WIDTH = 1.5

KNOWN_DISTANCE_DROPOFF = 13.2
KNOWN_WIDTH_DROPOFF = 1.5

# Ping Pong Ball Distance
image = cv2.imread("uartImages/uartImage1.jpg")
((x,y), radius) = find_marker(image)
focalLength = ((radius*2) * KNOWN_DISTANCE) / KNOWN_WIDTH

# Drop Off Location Box Distance
dImage = cv2.imread("uartImages/uartImage2.jpg")
dMarker = find_dropOff(dImage)
dFocalLength = (dMarker[1][0] * KNOWN_DISTANCE_DROPOFF) / KNOWN_WIDTH_DROPOFF

print("Start UART Communication")

ser = serial.Serial('/dev/serial0', baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                    )

ser.flushOutput()

START_BYTE = 0x41
COMMAND_BYTE = 0x42
X_CORD1 = 0x00
X_CORD0 = 0x00
Y_CORD1 = 0x00
Y_CORD0 = 0x00
DISTANCE = 0x00
check_sum = 0x00

packet = bytearray()

packet.append(START_BYTE)
packet.append(COMMAND_BYTE)
packet.append(X_CORD1)
packet.append(X_CORD0)
packet.append(Y_CORD1)
packet.append(Y_CORD0)
packet.append(DISTANCE)
packet.append(check_sum)


# keep looping
while True:
    # grab the current frame
    frame = vs.read()

    # handle the frame from VideoCapture or VideoStream
    frame = frame[1] if args.get("video", False) else frame

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break

    """

        # Ball Search
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "orange", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    xcordinate = None
    ycordinate = None
        """

        #Drop Off Zone Search

    frame = imutils.resize(frame, width=600)

    dBlurred = cv2.GaussianBlur(frame, (5,5), 0)
    dHsv = cv2.cvtColor(dBlurred, cv2.COLOR_BGR2HSV)

    #dMask0 = cv2.inRange(dHsv, redLower, redUpper)
    #dMask1 = cv2.inRange(dHsv, redLower1, redUpper1)
    #xMask = dMask0 + dMask1
    xMask = cv2.inRange(dHsv, greenLower, greenUpper)
    xMask = cv2.erode(xMask, None, iterations=2)
    xMask = cv2.dilate(xMask, None, iterations=2)


    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray, (11,11), 0)
    #edged = cv2.Canny(gray, 35, 125)
    #cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    cnts = cv2.findContours(xMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)



    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)

        # PROCESS DROP OFF CHARACTERISTICS
        dropOff = cv2.minAreaRect(c)
        inches = distance_to_camera(KNOWN_WIDTH_DROPOFF, dFocalLength, dropOff[1][0])
        M = cv2.moments(c)
        xcoordinate = int(M["m10"] / M["m00"])
        ycoordinate = int(M["m01"] / M["m00"])


        '''
                # PROCESS BALL CHARACTERISTICS
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        inches = distance_to_camera(KNOWN_WIDTH, focalLength, radius*2)

        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        xcoordinate = int(M["m10"] / M["m00"])
        ycoordinate = int(M["m01"] / M["m00"])
                '''

        xCord1 = (xcoordinate >> 7) & 0xFF
        yCord1 = (ycoordinate >> 7) & 0xFF

        xCord0 = xcoordinate & 0x7F
        yCord0 = ycoordinate & 0x7F

        print("X-Coordinate: ", xcoordinate)
        print("Y-Coordinate: ", ycoordinate)

        if inches < 0:
            distanceInch = 0
        elif inches > 255:
            distanceInch = 255
        else:
            distanceInch = round(inches)
        print("DistanceInch = ", inches)


        packet[2] = xCord1
        packet[3] = xCord0
        packet[4] = yCord1
        packet[5] = yCord0
        packet[6] = distanceInch

        check_sum = packet[0] + packet[1] + packet[2] + packet[3] + packet[4] + packet[5] + packet[6]

        checkSumByte = check_sum & 0x7F

        packet[7] = checkSumByte

        sendIndvBytes(packet)

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(dropOff)
        box = np.int0(box)
        cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)

        """
                # USED TO VIEW CONTOUR ON THE VIDEO FEED
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (255, 0, 0), 8)
            print("Distance (inches): ", inches)
            pts.appendleft(center)
        """


    # show the frame to our screen and increment the frame counter
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    counter += 1

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.stop()

# otherwise, release the camera
else:
    vs.release()

# close all windows
cv2.destroyAllWindows()
