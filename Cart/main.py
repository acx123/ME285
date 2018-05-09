from GPSPath import GPSPath
from Controller import Controller

path = GPSPath([((0,0),0),((0,0),10)])
controller = Controller(path)
controller.changePathR([((0,0),0),((0,0),10)])
controller.run()
