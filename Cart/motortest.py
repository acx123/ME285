import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

m1,m2 = ('P9_14','P9_16')
inputpin = 'P9_40'
ADC.setup()

max_DS = 50

PWM.start(motorpin,0,10000)

while True:
    normin = ADC.read(inputpin)
    PWM.set_duty_cycle(motorpin,normin*max_DS)
    PWM.set_set_duty_cycle(motorpin,normin*max_DS)
