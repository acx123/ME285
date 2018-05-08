import threading

class Ultrasound(threading.Thread):
    def __init__(self,interrupts,fwd_rev):
        self.interrupts = interrupts
        self.fwd_rev = fwd_rev
        self.stop = ('CM','STOP')
        self.cont = ('CM','ASSIST')
        pass

    def run(self):
        pass
