import Adafruit_GPIO.I2C as I2C
import subprocess as sub

lcd_write = sub.Popen(['lcd_write'],stdin=sub.PIPE)
lcd_write.communicate('Line1 \n Line2')
