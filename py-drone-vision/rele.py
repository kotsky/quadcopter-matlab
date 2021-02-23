import RPi.GPIO as GPIO
import time

###################################################################################################
def main():
    GPIO.setmode(GPIO.BCM)      # use GPIO pin numbering, not physical pin numbering

    rele1 = 26
    rele2 = 19
    rele3 = 13

    GPIO.setup(rele1, GPIO.OUT)
    GPIO.setup(rele2, GPIO.OUT)
    GPIO.setup(rele3, GPIO.OUT)



    GPIO.output(rele2, GPIO.LOW)
    GPIO.output(rele3, GPIO.LOW)

    # end while

    return
# end main

###################################################################################################
if __name__ == "__main__":
    main()
