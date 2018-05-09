import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import time
m1,m2 = ('P9_14','P9_16')

max_DS = 50

PWM.start(m1,30,10000)
PWM.start(m2,30,10000)

time.sleep(10)

PWM.set_duty_cycle(m1,0)
PWM.set_duty_cycle(m2,0)
