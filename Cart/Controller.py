from gps3 import agps3threaded as gps
from Queue import Queue,Empty,Full
from collections import namedtuple
from GPSPath import GPSPath as Gpath
from GPSPath import Vector
import os
import socket
import pickle
import time
import threading
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
import Adafruit_GPIO.I2C as I2C

class Controller(object):
    def __init__(self,GPSPath):
        self.running = True
        self.INTERRUPTS = Queue()
        self.lcd_buffer = Queue(2)

        self.server = threading.Thread(target=self._network,args=[self.INTERRUPTS])
        self.server.daemon = True
        self.server.start()
        self.GPSPath = GPSPath
        self.MODES = {'MANUAL':self.manualStep,'ASSIST':self.assistStep,'STOP':self.stop}
        self.INTR_TYPES = {'CM':self.changeMode,'NEW_PATH':self.changePath,'NEW_PATH_R':self.changePathR,'EXIT':self.quit}
        self.mode = 'MANUAL'
        self.motorpins = ('P9_14','P9_16')
        self.accel_pin = ('AIN5')
        self.fwd_rev = ('P9_27')
        self.frswitch = 'P9_23'
        os.execv('/bin/bash',('gpsd -n /dev/ttyUSB0'))
        GPIO.setup(self.fwd_rev,GPIO.OUT)
        GPIO.setup(self.frswitch,GPIO.IN)
        GPIO.add_event_detect(self.frswitch,GPIO.BOTH)
        GPIO.setup('P9_25',GPIO.OUT)
        GPIO.output('P9_25',GPIO.HIGH)
        self.duty_cycle_range = 49
        ADC.setup()

#        self.adxl = ADXL()
        self.lcd = LCD(self.lcd_buffer)
        self.lcd.daemon = True
        self.lcd.start()

        self.lcd_buffer.put(('MANUAL MODE','    MANUAL MODE'))

        PWM.start(self.motorpins[0],0,10000)
        PWM.start(self.motorpins[1],0,10000)

        self.GPS = gps.AGPS3mechanism()
        self.GPS.stream_data()
        self.GPS.run_thread()
        self.t0 = 0

        self.pot_offset = (0.10,0.50)
        self.prev_pos = None

        self.conv_const = 364320 #to convert deg of lat to feet
        self.max_speed = 12 #max speed in ft/sec
        #self.us = Ultrasound(self.INTERRUPTS,self.fwd_rev)
        #self.us.start()
        self._numcalls = 0

    def readPot(self):
        val = (self.pot_offset[1]-ADC.read('AIN5'))/(self.pot_offset[1]-self.pot_offset[0])
        return max([0,min([1,val])])

    def manualStep(self,deltaTime):
        userRequested = self.readPot()
        self.setMotorSpeed(userRequested,userRequested)

    def assistStep(self,deltaTime):
        cur_pos = (self.GPS.data_stream.lat,self.GPS.data_stream.lon)
        try:
            cur_pos = (float(cur_pos[0]),float(cur_pos[1]))
        except:
            return
        if self.prev_pos == None:
            self.prev_pos = cur_pos
        userRequested = self.readPot()
#        cur_acc = self.adxl.accelData()
        self.GPSPath.updatePosition(cur_pos,self.t0)
        deviation = self.GPSPath.pathDeviation(cur_pos)
        dev_dist,dev_angle = self.GPSPath.pathDeviation(cur_pos)
        pos_vec = Vector(self.prev_pos,cur_pos)
        heading = pos_vec.absAngle()
        rel_bear = self.GPSPath.getRelBearing(heading)
        dev_bear = dev_angle - heading
        if abs(dev_bear) > 180:
            dev_bear = 180 + dev_bear * (1 - 2 * (dev_bear > 0))
        gps_vel = abs(pos_vec)/deltaTime
        adj_max_speed = self.GPSPath.speedReq(time)/(1 + dev_bear*dev_dist*20000 + rel_bear/max(dev_dist,0.3))
        adj_max_ds = adj_max_speed*self.conv_const/self.max_speed
        min_ds = min([userRequested,adj_max_ds])
        self.setMotorSpeed(min_ds, min_ds)
        turn_angle = dev_bear if dev_dist*self.conv_const > 5 else rel_bear
        line2 = ('Close' if dev_dist*self.conv_const < 5 else 'Far' ,'Right' if dev_bear < 0 else 'Left')
        try:
            self.lcd_buffer.put_nowait(('DEV:{:3} |BEA:{:3}'.format(dev_dist,abs(turn_angle)),'{:7}|{:7}'.format(line2)))
        except Full:
            pass

    def PIDStep(self,error,Kp=0.1,Ki=0.1,Kd=0.1):
        pass

    def setMotorSpeed(self,m1,m2):
        PWM.set_duty_cycle(self.motorpins[0], self.duty_cycle_range * m1)
        PWM.set_duty_cycle(self.motorpins[1], self.duty_cycle_range * m2)

    def quit(self,args):
        self.running = False
        self.GPS.stop()
        self.server.stop()
        self.lcd.stop()
        setMotorSpeed(0,0)

    def stop(self,deltaTime):
        setMotorSpeed(0,0)

    def handleInterrupt(self,interrupt):
        cmd,args = interrupt
        try:
            self.INTR_TYPES[cmd](args)
        except KeyError:
            print(cmd)

    def changeMode(self,args):
        self.mode = args
        if self.mode == 'MANUAL':
            self.lcd_buffer.put(('MANUAL MODE','   MANUAL MODE'))
        elif self.mode == 'STOP':
            self.lcd_buffer.put(('STOPPED!','Check for obstructions'))
        self.prev_pos = None

    def changePath(self,args):
        self.GPSPath = args
        self.t0 = 0

    def changePathR(self,args):
        cur_pos = None
        while cur_pos == None
            try:
                cur_pos = (float(self.GPS.data_stream.lat),float(self.GPS.data_stream.lon))
            except:
                continue
        self.GPSPath = GPSPath(args,offset=cur_pos)
        self.t0 = 0

    def run(self):
        self.t0 = time.time()
        while self.running:
#       if GPIO.event_detected(frswitch):
#        GPIO.output(fwd_rev,GPIO.input(frswitch))
            try:
                inter = self.INTERRUPTS.get_nowait()
                self.handleInterrupt(inter)
                self.INTERRUPTS.task_done()
            except Empty:
                pass
            except TypeError:
                print(inter)
            t1 = time.time()
            self.MODES[self.mode](time.time() - self.t0)
            self.t0 = t1
            self._numcalls = self._numcalls + 1

    def _network(self,inter):
        serv = socket.socket()
        serv.bind(('',4440))
        serv.listen(1)
        cli,cli_addr = serv.accept()
        serv.listen(0)
        while True:
            try:
                pickled_data = cli.recv(1024)
                dat = pickle.loads(pickled_data)
                self.lcd_buffer.put(dat)
            except EOFError:
                serv.listen(1)
                cli,cli_addr = serv.accept()
                serv.listen(0)
                break
            inter.put(pickle.loads(pickled_data))


class ADXL(object):
    def __init__(self):
        self.ADXL_DEV = 0x53
        self.ADXL_PWR = 0x2d
        self.ADXL_FMT = 0x31
        self.ADXL_DAT = 0x32
        self.ADXL_TO_READ = 6
        self.ADXL_SENS = 0.004 #mG/LSB
        self.ADXL_OFFSETS = (-4,-3,0)
        self.ADXL_OFFREG = (0x1E,0x1F,0x20)
        self.adxl = I2C.get_i2c_device(self.ADXL_DEV,2)
        self.adxl.write16(self.ADXL_PWR,0x08)
        self.adxl.write16(self.ADXL_FMT,0x08)
        for reg,offset in zip(self.ADXL_OFFREG,self.ADXL_OFFSETS):
            self.adxl.write16(reg,offset)

    def calibrateAngles(self):
        pass

    def accelData(self):
        raw_data = self.adxl.readList(self.ADXL_DAT,self.ADXL_TO_READ)
        data = (int(raw_data[1] << 8 | raw_data[0]),int(raw_data[3] << 8 | raw_data[2]),int(raw_data[5] << 8 | raw_data[4]))
        data = ((data[0] - (data[0] >> 15) * 65536) *self.ADXL_SENS * 32.2,(data[1] - (data[1] >> 15) * 65536) * self.ADXL_SENS * 32.2,(data[2] - (data[2] >> 15) * 65536) * self.ADXL_SENS * 32.2)
        return data

class LCD(threading.Thread):
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

    def __init__(self,queue,address=0x27,busnum=2,col=16,row=2):
        self.device = I2C.get_i2c_device(address,busnum)
        self._addr = address
        self._bus = busnum
        self.col = col
        self.row = row
        time.sleep(0.045)
        self._backlight = self._LCD_BACKLIGHT
        self._buff = queue
        for _delay in [0.004500, 0.004500, 0.000150]:
            self.write4(0x03)
            time.sleep(_delay)

        self.write4(0x02)

        self.write4(self._LCD_FUNCTIONSET | self._LCD_4BITMODE | self._LCD_2LINE | self._LCD_5x8DOTS)

        self.write4(self._LCD_DISPLAYCONTROL | self._LCD_DISPLAYON | self._LCD_CURSOROFF | self._LCD_BLINKOFF)

        self.write4(self._LCD_ENTRYMODESET | self._LCD_ENTRYLEFT | self._LCD_ENTRYSHIFTDECREMENT)
        super(LCD,self).__init__()

    def run(self):
        while True:
            text = self._buff.get()
            self.set_message(text)
            self._buff.task_done()
            time.sleep(0.75)

    def write4(self,data,mode=0):
        dat_hl = ((data & 0xF0) | mode , ((data << 4) & 0xF0)| mode)
        try:
            for dat in dat_hl:
                self.device.writeRaw8(dat | self._backlight)
                self.device.writeRaw8((dat | self._En) | self._backlight)
                time.sleep(0.000001)
                self.device.writeRaw8((dat | ~self._En) | self._backlight)
                time.sleep(0.000050)
        except IOError:
            if IOError.errno == 11:
                self.write4(data,mode)
            elif IOError.errno == 121:
                self.device = I2C.get_i2c_device(self._addr,self._bus)

    def move_cursor(self,col,row):
        rows = [0x00,0x40]
        self.write4(self._LCD_SETDDRAMADDR | (rows[row] + col))
        time.sleep(0.00001)

    def set_message(self,string):
        self.move_cursor(0,0)
        lines = (string[0].ljust(16),string[1].ljust(16))
        for line in lines:
            for char in line:
                time.sleep(0.000001)
                self.write4(ord(char),mode=0x01)
            self.move_cursor(0,1)
        return lines
