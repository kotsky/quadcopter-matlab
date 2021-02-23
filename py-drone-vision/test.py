
import cv2
import numpy as np
import os
import RPi.GPIO as gpio
import time


def moved(Angle):

        if Angle == 1:
                DC_x = int(50)
                pwmObject_x_r.ChangeDutyCycle(0)
                pwmObject_x_l.ChangeDutyCycle(DC_x)

        else:
                DC_x = int(50)
                pwmObject_x_l.ChangeDutyCycle(0)
                pwmObject_x_r.ChangeDutyCycle(DC_x)



gpio.setmode(gpio.BCM)

pin_x_l = 20
pin_x_r = 21

freq_x_l = 50
freq_x_r = 50

gpio.setup(pin_x_l, gpio.OUT)
gpio.setup(pin_x_r, gpio.OUT)

pwmObject_x_l = gpio.PWM(pin_x_l, freq_x_l)
pwmObject_x_r = gpio.PWM(pin_x_r, freq_x_r)

pwmObject_x_l.start(10)
pwmObject_x_r.start(10)


while True:
        strAngle = raw_input("enter desired angle (0 to 180): ")
        intAngle = int(strAngle)
        Angle = int(intAngle)
	moved(Angle)


