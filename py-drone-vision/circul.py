from picamera.array import PiRGBArray
from picamera import PiCamera
import time

import argparse
import numpy as np
import glob
import cv2
import sys
import math

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

def nothing(*arg):
    pass

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    status = "No Targets"

    image = frame.array
 
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 7, 12, 12)

    v = np.median(gray)
    sigma = 50
    low = int(max(0, (1.0 - sigma/100) * v))
    up = int(min(255, (1.0 + sigma/100) * v))

    edged = cv2.Canny(gray, low, up)
 
    img, cnts, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.0001 * peri, True)
        k = cv2.isContourConvex(c)
        if (k == False):
            M = cv2.moments(c)
            area = cv2.contourArea(approx)

            (x,y),radius = cv2.minEnclosingCircle(c)
            S = np.pi * radius * radius
            sol = area/float(S)

            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = area/(hull_area+1)

            if sol >= 0.8 and sol <= 1 and solidity >= 0.95:

		m = x
		n = y

                center = (int(x),int(y))
                radius = float(radius)
                R = np.sqrt(area/np.pi)
                
                L = (15*400)/(R)
                F=((15*15*np.pi*400)/(area) - (15*15/(radius*radius)) * 400)/2
                L = round(L)
                F = round(F)
                area = round(area)
                status = "Target acquired, length = " + str(L) + "mm"

    		print("X= " + str(m) + "; Y= " + str(n))
 
		
    cv2.imshow("Frame", image)

    rawCapture.truncate(0)

    key =     ch = cv2.waitKey(5) & 0xFF
    if ch == 27:
        break

 
camera.release()
cv2.destroyAllWindows()


