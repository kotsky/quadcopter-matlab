
import cv2
import numpy as np
import os
import RPi.GPIO as gpio
import time


def moved(Angle):

        if Angle <101:
               # DC_x = int(50)
                pwmObject_x_r.ChangeDutyCycle(0)
                pwmObject_x_l.ChangeDutyCycle(Angle)

        else:
                #DC_x = int(50)
                pwmObject_x_l.ChangeDutyCycle(0)
                pwmObject_x_r.ChangeDutyCycle(Angle-100)



gpio.setmode(gpio.BCM)

pin_x_l = 20
pin_x_r = 21

freq_x_l = 50
freq_x_r = 50

gpio.setup(pin_x_l, gpio.OUT)
gpio.setup(pin_x_r, gpio.OUT)

pwmObject_x_l = gpio.PWM(pin_x_l, freq_x_l)
pwmObject_x_r = gpio.PWM(pin_x_r, freq_x_r)

pwmObject_x_l.start(0)
pwmObject_x_r.start(0)


while True:
        strAngle = raw_input("enter DC of turn into left(101-200) or into right (0-100): ")
        intAngle = int(strAngle)
        Angle = int(intAngle)
	moved(Angle)
