import cv2
import numpy as np
import os
import RPi.GPIO as gpio
import time

###################################################################################################

gpio.setmode(gpio.BCM)

pin_x_l = 12
pin_x_r = 16
pin_y_up = 20
pin_y_d = 21

freq_x = 50
freq_y = 50

gpio.setup(pin_x_l, gpio.OUT)
gpio.setup(pin_x_r, gpio.OUT)

pwmObject_x_l = gpio.PWM(pin_x_l, freq_x)
pwmObject_x_r = gpio.PWM(pin_x_r, freq_x)

pwmObject_x_l.start(0) 
pwmObject_x_r.start(0) 

gpio.setup(pin_y_up, gpio.OUT)
gpio.setup(pin_y_d, gpio.OUT)

pwmObject_y_up = gpio.PWM(pin_y_up, freq_y)
pwmObject_y_d = gpio.PWM(pin_y_d, freq_y)

pwmObject_y_up.start(0)
pwmObject_y_d.start(0)

###################################################################################################

def moved_x(AK_x,Turn_x_l,Turn_x_r):
	
	if AK_x == 1:
		DC_x_l = int(Turn_x_l)
		DC_x_r = int(0)
		pwmObject_x_r.ChangeDutyCycle(DC_x_r)
		pwmObject_x_l.ChangeDutyCycle(DC_x_l)
		
	else:   
		DC_x_r = int(Turn_x_r)
                DC_x_l = int(0)
		pwmObject_x_l.ChangeDutyCycle(DC_x_l)
		pwmObject_x_r.ChangeDutyCycle(DC_x_r)

	
def moved_y(AK_y,Turn_y_up,Turn_y_d):

        if AK_y == 1:
		DC_y_d = int(Turn_y_d)
                DC_y_up = int(0)
                pwmObject_y_up.ChangeDutyCycle(DC_y_up)
                pwmObject_y_d.ChangeDutyCycle(DC_y_d)

        else:
		DC_y_up = int(Turn_y_up)
                DC_y_d = int(0)
                pwmObject_y_d.ChangeDutyCycle(DC_y_d)
                pwmObject_y_up.ChangeDutyCycle(DC_y_up)



###################################################################################################
def main():
    DELAY = 0.001
    diap_x = int(20)
    diap_y = int(10)
    AK_x = 1
    AK_y = 1

    Angle_x = 1
    Koeff_x = 1

    Angle_y = 1
    Koeff_y = 1

###################################################################################################

    capWebcam = cv2.VideoCapture(0)

    print("default resolution = " + str(capWebcam.get(cv2.CAP_PROP_FRAME_WIDTH)) + "x" + str(capWebcam.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    capWebcam.set(cv2.CAP_PROP_FRAME_WIDTH, 320.0)
    capWebcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240.0)

    intXFrameCenter = int(float(capWebcam.get(cv2.CAP_PROP_FRAME_WIDTH)) / 2.0)
    intYFrameCenter = int(120)

    Position_x = 0
    Position_y = 0
    x=160
    y=120

    if capWebcam.isOpened() == False:
        print("error: capWebcam not accessed successfully\n\n")
        os.system("pause")
        return
    # end if

###################################################################################################

    while cv2.waitKey(1) != 27 and capWebcam.isOpened():
        blnFrameReadSuccessfully, frame = capWebcam.read()

        if not blnFrameReadSuccessfully or frame is None:
            print("error: frame not read from webcam\n")
            os.system("pause")
            break
        # end if

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edged = cv2.Canny(blurred, 50, 150)
 
    # find contours in the edge map
        img, cnts, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # loop over the contours
        for c in cnts:
	    # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01 * peri, True)
 
	# ensure that the approximated contour is "roughly" rectangular
            if len(approx) >= 4 and len(approx) <= 6:
            # compute the bounding box of the approximated contour and
	    # use the bounding box to compute the aspect ratio
                (a, b, w, h) = cv2.boundingRect(approx)
                aspectRatio = w / float(h)
 
	    # compute the solidity of the original contour
                area = cv2.contourArea(c)
                hullArea = cv2.contourArea(cv2.convexHull(c))
                solidity = area / float(hullArea)
 
            # compute whether or not the width and height, solidity, and
            # aspect ratio of the contour falls within appropriate bounds
                keepDims = w > 25 and h > 25
                keepSolidity = solidity > 0.9
                keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2
 
            # ensure that the contour passes all our tests
                if keepDims and keepSolidity and keepAspectRatio:
        	# draw an outline around the target and update the status
        	# text
                    cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)
                    status = "Target(s) Acquired"
 
		# compute the center of the contour region and draw the
		# crosshairs
                    M = cv2.moments(approx)
                    (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    (startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
                    (startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
                    cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 255), 3)
                    cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 255), 3)
		    x = cX
		    y = cY
###################################################################################################

            if x > intXFrameCenter:
                Angle_x = 1
                Turn_x_r = int(0)
                if x < intXFrameCenter + diap_x:
                    Turn_x_l = int((x-160)*0.2)

                else:
                    Turn_x_l = int(0.3 * (x-160))


            else:
                Angle_x = -1
                Turn_x_l = int(0)
                if intXFrameCenter - diap_x < x:
                    Turn_x_r = int(-(x-160)*0.2)

                else:
                    Turn_x_r = int(0.3 * (-(x-160)))

###################################################################################################

            if y > intYFrameCenter:
                Angle_y = 1
                Turn_y_up = int(0)
                if y < intYFrameCenter + diap_y:
                    Turn_y_d = int((y-120)*0.25)

                else:
                    Turn_y_d = int((y-120)*0.35)


            else:
                Angle_y = -1
                Turn_y_d = int(0)
                if intYFrameCenter - diap_y < y:
                    Turn_y_up = int(-(y-120)*0.25)

                else:
                    Turn_y_up = int(0.35*(-(y-120)))


###################################################################################################

	AK_x = Angle_x*Koeff_x
	AK_y = Angle_y*Koeff_y

	moved_x(AK_x,Turn_x_l,Turn_x_r)
	moved_y(AK_y,Turn_y_up,Turn_y_d)

###################################################################################################
    # draw the status text on the frame
        cv2.putText(frame, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
 
    # show the frame and record if a key is pressed
        cv2.imshow("Frame", frame)
 
    # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break
 
# cleanup the camera and close any open windows
    camera.release()
    cv2.destroyAllWindows()

        
    return

###################################################################################################
if __name__ == "__main__":
    main()

