import Adafruit_GPIO.I2C as I2C

ADXL_DEV = 0x53
ADXL_PWR = 0x2d
ADXL_FMT = 0x31
ADXL_DAT = 0x32
ADXL_TO_READ = 6
ADXL_SENS = 0.004 #mG/LSB
ADXL_OFFSETS = (-4,-3,0)
ADXL_OFFREG = (0x1E,0x1F,0x20)

adxl = I2C.get_i2c_device(ADXL_DEV,2)
adxl.write16(ADXL_PWR,0x08)
adxl.write16(ADXL_FMT,0x08)
for reg,offset in zip(ADXL_OFFREG,ADXL_OFFSETS):
    adxl.write16(reg,offset)


while True:
    raw_data = adxl.readList(ADXL_DAT,ADXL_TO_READ)
    data = (int(raw_data[1] << 8 | raw_data[0]),int(raw_data[3] << 8 | raw_data[2]),int(raw_data[5] << 8 | raw_data[4]))
    data = (data[0] - (data[0] >> 15) * 65536,data[1] - (data[1] >> 15) * 65536,data[2] - (data[2] >> 15) * 65536)
    print(data)
