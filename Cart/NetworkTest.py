import socket
import pickle
import threading
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

INTERRUPTS = Queue()
MODES = {'MANUAL':manualStep,'ASSIST':assistStep,'STOP':stop}
INTR_TYPES = {'CM':changeMode,'EXIT':quit}
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
            cli.sendall(pickle.dumps(((msgcnt,len(handled)),protocol=2))
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
    thread = threading.Thread(target=_network())
    thread.setDaemon(True)
    thread.start()
    print('Thread Started')
    run()
