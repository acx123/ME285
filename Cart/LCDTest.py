import Adafruit_GPIO.I2C as I2C
import time

class LCD:
    # commands
    _LCD_CLEARDISPLAY = 0x01
    _LCD_RETURNHOME = 0x02
    _LCD_ENTRYMODESET = 0x04
    _LCD_DISPLAYCONTROL = 0x08
    _LCD_CURSORSHIFT = 0x10
    _LCD_FUNCTIONSET = 0x20
    _LCD_SETCGRAMADDR = 0x40
    _LCD_SETDDRAMADDR = 0x80

    # flags for display entry mode
    _LCD_ENTRYRIGHT = 0x00
    _LCD_ENTRYLEFT = 0x02
    _LCD_ENTRYSHIFTINCREMENT = 0x01
    _LCD_ENTRYSHIFTDECREMENT = 0x00

    # flags for display on/off control
    _LCD_DISPLAYON = 0x04
    _LCD_DISPLAYOFF = 0x00
    _LCD_CURSORON = 0x02
    _LCD_CURSOROFF = 0x00
    _LCD_BLINKON = 0x01
    _LCD_BLINKOFF = 0x00

    # flags for display/cursor shift
    _LCD_DISPLAYMOVE = 0x08
    _LCD_CURSORMOVE = 0x00
    _LCD_MOVERIGHT = 0x04
    _LCD_MOVELEFT = 0x00

    # flags for function set
    _LCD_8BITMODE = 0x10
    _LCD_4BITMODE = 0x00
    _LCD_2LINE = 0x08
    _LCD_1LINE = 0x00
    _LCD_5x10DOTS = 0x04
    _LCD_5x8DOTS = 0x00

    # flags for backlight control
    _LCD_BACKLIGHT = 0x08
    _LCD_NOBACKLIGHT = 0x00

    _En = 0x04  # Enable bit
    _Rw = 0x02  # Read/Write bit
    _Rs = 0x01  # Register select bit

    def __init__(address=0x27,busnum=2,col=16,row=2):
        self.device = I2C.get_i2c_device(_addr,_bus)
        self.col = col
        self.row = row
        time.sleep(0.045)
        self._backlight = self._LCD_BACKLIGHT
        for _delay in [0.004500, 0.004500, 0.000150]:
            self.write4(0x03 << 4)
            time.sleep(_delay)

        self.write4(0x02 << 4)

        self.write4(self._LCD_FUNCTIONSET | self._LCD_4BITMODE | self._LCD_2LINE | self._LCD_5x8DOTS)

        self.write4(self._LCD_DISPLAYCONTROL | self._LCD_DISPLAYON | self._LCD_CURSOROFF | self._LCD_BLINKOFF)

        self.write4(self._LCD_ENTRYMODESET | self._LCD_ENTRYLEFT | self._LCD_ENTRYSHIFTDECREMENT)

    def write4(self,data,mode=0):
        dat_hl = ((data & 0xF0) | mode , ((data << 4) & 0xF0)| mode)
        for dat in dat_hl:
            self.device.write8(dat | self._backlight)
            self.device.write8((dat | _En) | self._backlight)
            time.sleep(0.000001)
            self.device.write8((dat | ~_En) | self._backlight)
            time.sleep(0.000050)

    def move_cursor(self,col,row):
        rows = [0x00,0x40]
        self.write4(self._LCD_SETDDRAMADDR | (rows[row] + col))

    def set_message(self,string):
        self.move_cursor(0,0)
        for char in string:
            if char is '\n':
                self.move_cursor(0,1)
            self.write4(ord(char),mode=0x01)
