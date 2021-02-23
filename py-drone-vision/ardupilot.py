import cv2
import numpy as np
import os
import RPi.GPIO as gpio
import time

import pigpio

pi = pigpio.pi()

gpio.setmode(gpio.BCM)

pin_x_l = 12
pin_x_r = 16
pin_y_up = 20
pin_y_d = 21

freq_y = 50

gpio.setup(pin_x_l, gpio.IN)
gpio.setup(pin_x_r, gpio.IN)

gpio.setup(pin_y_up, gpio.OUT)
gpio.setup(pin_y_d, gpio.OUT)

pwmObject_y_up = gpio.PWM(pin_y_up, freq_y)
pwmObject_y_d = gpio.PWM(pin_y_d, freq_y)

pwmObject_y_up.start(0)
pwmObject_y_d.start(0)

while True:
	cb1 = pi.callback(pin_x_l, pigpio.RISING_EDGE)
	CT = time.time()
	pi.wait_for_edge(pin_x_l, pigpio.FALLING_EDGE)
	CT_HIGH = time.time() - CT

#	pi.wait_for_edge(pin_x_l, pigpio.RISING_EDGE)
#	CT_LOW = time.time() - CT_HIGH - CT
#	time.sleep(CT_HIGT)
	#cb1 = pi.callback(pin_x_l, pigpio.RISING_EDGE)

#	FULL = CT_HIGH + CT_LOW
#	
#	DC = (CT_HIGH/FULL)*100
	
	


	print("DC=" + str(CT_HIGH))
