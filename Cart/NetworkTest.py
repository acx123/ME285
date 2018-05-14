import socket
import pickle
import threading
from GPSPath import GPSPath
from gps3 import agps3threaded as gps
from queue import Queue

def changeMode(arg):
    mode = arg

def quit(arg):
    running=False

def manualStep(arg):
    pass

def assistStep(arg):
    pass

def stop(arg):
    pass

def changePathR(self,args):
    cur_pos = (float(GPS.data_stream.lat),float(GPS.data_stream.lon))
    GPSPath = GPSPath(args,offset=cur_pos)

INTERRUPTS = Queue()
MODES = {'MANUAL':manualStep,'ASSIST':assistStep,'STOP':stop}
INTR_TYPES ={'CM':changeMode,'NEW_PATH_R':changePathR,'EXIT':quit}
mode = 'MANUAL'
running = True
handled = list()
def run():
    t0 = time.time()
    while running:
        try:
            inter = INTERRUPTS.get_nowait()
            print(inter)
            handleInterrupt(inter)
            handled.append(inter)
            INTERRUPTS.task_done()
        except Empty:
            pass
        except TypeError:
            print(inter)
        t1 = time.time()
        print(mode)
        time.sleep(1)
        t0 = t1

def _network():
    serv = socket.socket()
    serv.bind(('',4440))
    serv.listen(1)
    cli,cli_addr = serv.accept()
    serv.listen(0)
    msgcnt = 0
    while True:
        try:
            pickled_data = cli.recv(1024)
            msgcnt = msgcnt + 1
            dat = pickle.loads(pickled_data)
        except EOFError:
            serv.listen(1)
            cli,cli_addr = serv.accept()
            serv.listen(0)
            continue
        INTERRUPTS.put(dat)

def handleInterrupt(inter):
    try:
        INTR_TYPES[inter[0]](inter[1])
    except Exception:
        print(Exception)

if __name__ == '__main__':
    GPS = gps.AGPS3mechanism()
    GPS.stream_data()
    GPS.run_thread()
    thread = threading.Thread(target=_network())
    thread.setDaemon(True)
    thread.start()
    print('Thread Started')
    run()
