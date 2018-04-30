import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

motorpin = 'P9_14'
inputpin = 'P9_40'
ADC.setup()

max_freq = 100

PWM.start(motorpin,50)

while True:
    normin = ADC.read(inputpin)
    print(str(normin) + '\r')
    PWM.set_frequency(motorpin,1+normin*max_freq)
