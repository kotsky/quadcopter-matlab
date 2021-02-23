import numpy as np
import os
import RPi.GPIO as gpio
from evdev import InputDevice, categorize, ecodes

###########################################################
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

Turn_x_l = 0
Turn_x_r = 0
Turn_y_up = 0
Turn_y_d = 0

##############################################################

def moved_x(Turn_x_l,Turn_x_r):
	
	if Turn_x_r == 0:
		DC_x_l = int(Turn_x_l)
		DC_x_r = int(0)
		pwmObject_x_r.ChangeDutyCycle(DC_x_r)
		pwmObject_x_l.ChangeDutyCycle(DC_x_l)
		
	else:   
		DC_x_r = int(Turn_x_r)
                DC_x_l = int(0)
		pwmObject_x_l.ChangeDutyCycle(DC_x_l)
		pwmObject_x_r.ChangeDutyCycle(DC_x_r)

	
def moved_y(Turn_y_up,Turn_y_d):

        if Turn_y_up == 0:
		DC_y_d = int(Turn_y_d)
                DC_y_up = int(0)
                pwmObject_y_up.ChangeDutyCycle(DC_y_up)
                pwmObject_y_d.ChangeDutyCycle(DC_y_d)

        else:
		DC_y_up = int(Turn_y_up)
                DC_y_d = int(0)
                pwmObject_y_d.ChangeDutyCycle(DC_y_d)
                pwmObject_y_up.ChangeDutyCycle(DC_y_up)



###########################################################

dev = InputDevice('/dev/input/event0')
for event in dev.read_loop():
        if event.type == 03:
                if event.code == 01:
                        if event.value == 127:
                                Turn_y_d = 0
                                Turn_y_up = 0
                        elif event.value > 127:
                                Turn_y_d = 0
                                Turn_y_up = event.value*0.7874 - 100
                        elif event.value < 127:
                                Turn_y_up = 0
                                Turn_y_d = 100 - 0.7874 * event.value
                        else:
                                continue
                elif event.code == 00:
                        if event.value == 127:
                                Turn_x_l = 0
                                Turn_x_r = 0
                        elif event.value > 127:
                                Turn_x_l = 0
                                Turn_x_r = event.value*0.7874 - 100
                        elif event.value < 127:
                                Turn_x_r = 0
                                Turn_x_l = 100 - 0.7874 * event.value
                        else:
                                continue
                else:
                        continue
        else:
                continue
	moved_x(Turn_x_l, Turn_x_r)
	moved_y(Turn_y_up, Turn_y_d)        
	print(event)
