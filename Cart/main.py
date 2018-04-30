from GPSPath import GPSPath
from Controller import Controller

path = GPSPath([((0,0),0),((1,1),100)])
controller = Controller(path)

controller.run()
