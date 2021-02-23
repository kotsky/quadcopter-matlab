import cv2
import numpy as np
import os
import RPi.GPIO as gpio
import time



gpio.setmode(gpio.BCM)
gpio.setup(23, gpio.OUT)
gpio.setup(24, gpio.OUT)




gpio.output(23, False)
gpio.output(24, False)

