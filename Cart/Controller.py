from gps3 import agps3threaded as gps
from collections import deque
from collections import namedtuple
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
        self.server = threading.Thread(target=self._network)
        self.server.start()
        self.GPSPath = GPSPath
        self.MODES = {'MANUAL':self.manualStep,'ASSIST':self.assistStep}
        self.INTR_TYPES = {'CM':self.changeMode,'NEW_PATH':self.changePath,'NEW_PATH_R':self.changePathR,'STOP':self.quit}
        self.mode = 'MANUAL'
        self.motorpins = ('P9_14','P9_16')
        self.accel_pin = ('AIN5')
        self.duty_cycle_range = 50
        ADC.setup()
#        self.adxl = ADXL()
        PWM.start(self.motorpins[0],0,10000)
        PWM.start(self.motorpins[1],0,10000)
        self.GPS = gps.AGPS3mechanism()
        self.GPS.stream_data()
        self.GPS.run_thread()
        self.t0 = 0
        self.INTERRUPTS = deque(list())
        self.conv_const = 364320 #to convert deg of lat to feet
        self.max_speed = 12 #max speed in ft/sec
        #self.cg_dist = constant distance to center of gravity


    def run(self):
        self.t0 = time.time()
        while self.running:
            if len(self.INTERRUPTS) > 0:
                handleInterrupt(self.INTERRUPTS.popleft())
            t1 = time.time()
            self.MODES[self.mode](time.time() - self.t0)
            self.t0 = t1

    def manualStep(self,deltaTime):
        userRequested = ADC.read(self.accel_pin)
        self.setMotorSpeed(userRequested,userRequested)

    def assistStep(self,deltaTime):
        cur_pos = (GPS.data_stream.lat,GPS.data_stream.lon)
        userRequested = ADC.read(accel_pin)
        cur_acc = adxl.accelData()
        self.GPSPath.updatePosition(cur_pos,t0)
        prev_pos = (pos,time)
        dev_dist,dev_angle = self.GPSPath.pathDeviation(cur_pos)
        pos_vec = Vector(prev_pos,cur_pos)
        gps_vel = abs(pos_vec)/deltaTime
        duty_cyc_conv = self.GPSPath.speedReq(time)*self.conv_const/self.max_speed
        adj_max_speed = self.GPSPath.speedReq(time)/(1 + dev_angle*dev_dist + pos_vec/min(dev_dist,0.3))

    def PIDStep(error,Kp=0.1,Ki=0.1,Kd=0.1):
        pass

    def setMotorSpeed(self,m1,m2):
        PWM.set_duty_cycle(self.motorpins[0], self.duty_cycle_range * m1)
        PWM.set_duty_cycle(self.motorpins[1], self.duty_cycle_range * m2)

    def quit(args):
        running = False
        self.GPS.stop()
        self.server.stop()

    def handleInterrupt(interrupt):
        cmd,args = interrupt
        INTR_TYPES[cmd](args)

    def changeMode(args):
        mode = MODES[args]

    def changePath(args):
        self.GPSPath = args
        self.t0 = 0

    def changePathR(args):
        cur_pos = (GPS.data_stream.lat,GPS.data_stream.lon)
        self.GPSPath = GPSPath(args,offset=cur_pos)
        self.t0 = 0

    def _network(self):
        serv = socket.socket()
        serv.bind(('',4440))
        serv.listen(1)
        cli,cli_addr = serv.accept()
        serv.listen(0)
        while True:
            try:
                pickled_data = cli.recv(1024)
                dat = pickle.loads(pickled_data)
            except EOFError:
                serv.listen(1)
                cli,cli_addr = serv.accept()
                serv.listen(0)
                continue
            self.INTERRUPTS.append(pickle.loads(pickled_data))


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
        adxl = I2C.get_i2c_device(self.ADXL_DEV,2)
        adxl.write16(self.ADXL_PWR,0x08)
        adxl.write16(self.ADXL_FMT,0x08)
        for reg,offset in zip(self.ADXL_OFFREG,self.ADXL_OFFSETS):
            adxl.write16(reg,offset)

    def accelData(self):
        raw_data = adxl.readList(self.ADXL_DAT,self.ADXL_TO_READ)
        data = (int(raw_data[1] << 8 | raw_data[0]),int(raw_data[3] << 8 | raw_data[2]),int(raw_data[5] << 8 | raw_data[4]))
        data = ((data[0] - (data[0] >> 15) * 65536) * ADXL_SENS * 32.2,(data[1] - (data[1] >> 15) * 65536) * ADXL_SENS * 32.2,(data[2] - (data[2] >> 15) * 65536) * ADXL_SENS * 32.2)
        return data
