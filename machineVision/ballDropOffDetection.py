# USAGE
# Ping Pong Ball detection
# Drop off zone detection
# UART

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

# define the lower and upper boundaries of "green"
# rectangle in the HSV color space
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


KNOWN_DISTANCE = 6
KNOWN_WIDTH = 1.5

KNOWN_DISTANCE_DROPOFF = 13.2
KNOWN_WIDTH_DROPOFF = 1.5

# Ping Pong Ball Distance
image = cv2.imread("uartImages/uartImage1.jpg")
image = cv2.resize(image, (160,120))
((x,y), radius) = find_marker(image)
focalLength = ((radius*2) * KNOWN_DISTANCE) / KNOWN_WIDTH

# Drop Off Location Distance
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
COMMAND_BYTE = 0x00

X_CORD1 = 0x00
X_CORD0 = 0x00
Y_CORD1 = 0x00
Y_CORD0 = 0x00

dX_CORD1 = 0x00
dX_CORD0 = 0x00

DISTANCE = 0x00
dDISTANCE = 0x00

check_sum = 0x00

packet = bytearray()

packet.append(START_BYTE)
packet.append(COMMAND_BYTE)

packet.append(X_CORD1)
packet.append(X_CORD0)
packet.append(Y_CORD1)
packet.append(Y_CORD0)
packet.append(DISTANCE)

packet.append(dX_CORD1)
packet.append(dX_CORD0)
packet.append(dDISTANCE)

packet.append(check_sum)

count = 0

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

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = cv2.resize(frame, (160, 120))

    blurred = cv2.GaussianBlur(frame, (5, 5), 0) #(11, 11)
    dBlurred = cv2.GaussianBlur(frame, (5,5), 0)

    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    dHsv = cv2.cvtColor(dBlurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "orange", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=1) #2, 2
    mask = cv2.dilate(mask, None, iterations=1)

    dMask = cv2.inRange(dHsv, greenLower, greenUpper)
    dMask = cv2.erode(dMask, None, iterations=2)
    dMask = cv2.dilate(dMask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    dCnts = cv2.findContours(dMask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    dCnts = imutils.grab_contours(dCnts)


    # only proceed if at least one contour was found
    # BALL DETECTION
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((xcoordinate, ycoordinate), radius) = cv2.minEnclosingCircle(c)

        inches = distance_to_camera(KNOWN_WIDTH, focalLength, radius*2)

        xCord1 = (int(xcoordinate) >> 7) & 0xFF
        yCord1 = (int(ycoordinate) >> 7) & 0xFF

        xCord0 = int(xcoordinate) & 0x7F
        yCord0 = int(ycoordinate) & 0x7F

        print("Ball X-Coordinate: ", xcoordinate)

        if inches < 0:
            distanceInch = 0
        elif inches > 255:
            distanceInch = 255
        else:
            distanceInch = int(round(inches))

        print("Ball Distance: ", distanceInch)

        packet[1] = 0x43
        packet[2] = xCord1
        packet[3] = xCord0
        packet[4] = yCord1
        packet[5] = yCord0
        packet[6] = distanceInch

        #check_sum = packet[0] + packet[1] + packet[2] + packet[3] + packet[4] + packet[5] + packet[6]

        #checkSumByte = check_sum & 0x7F

        #packet[7] = checkSumByte

        #sendIndvBytes(packet)

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xcoordinate), int(ycoordinate)), int(radius),
                (255, 0, 0), 8)
            #print("Distance (inches): ", inches)
            pts.appendleft(center)


    if len(dCnts) > 0:

        dC = max(dCnts, key=cv2.contourArea)

        dropOff = cv2.minAreaRect(dC)

        dInches = distance_to_camera(KNOWN_WIDTH_DROPOFF, dFocalLength, dropOff[1][0])

        dM = cv2.moments(dC)
        dXCoordinate = int(dM["m10"] / dM["m00"])

        dXCord1 = (dXCoordinate >> 7) & 0xFF
        dXCord0 = dXCoordinate & 0x7F

        print("Drop Off X-Coord: ", dXCoordinate)

        if dInches < 0:
            dDistanceInch = 0
        elif dInches > 255:
            dDistanceInch = 255
        else:
            dDistanceInch = round(dInches)

        print("Drop Off Distance: ", dInches)

        packet[1] = 0x45
        packet[7] = dXCord1
        packet[8] = dXCord0
        packet[9] = dDistanceInch

    check_sum = packet[0] + packet[1] + packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7] + packet[8] + packet[9]

    checkSumByte = check_sum & 0x7F

    packet[10] = checkSumByte

    sendIndvBytes(packet)

    count += 1

    print("Count: ", count)

    # show the frame to our screen and increment the frame counter
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    #counter += 1

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
