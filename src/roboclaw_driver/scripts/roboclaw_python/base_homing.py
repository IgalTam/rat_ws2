import RPi.GPIO as gpio
gpio.setmode(gpio.BCM)


hallpin = 2

gpio.setup(hallpin, gpio.IN)

while (True)
    if(gpio.input(hallpin) == True):
        print("magnet detected")
    else:
        print(" ")
