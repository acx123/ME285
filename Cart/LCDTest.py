import Adafruit_GPIO.I2C as I2C

def send(device,value)
    device.write8(value & 0xF0)
    device.write8((value<<4) & 0xF0)
