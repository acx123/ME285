import socket
import sys
import Controller
import pickle
import re
from __future__ import print_function

running = True
CMDS = {'quit':quit,'setmode':setmode:'setpath'}
pattern = '(\d+(?:,\s*\d+)*)'
ft2pt = 1/0.000003

def quit():
    running = False
    return ''

def setmode(command):
    try:
        Controller.MODES[command]
    except:
        raise Exception('Incorrect Argument')
        return None
    return ('CM',command)

def setpath(command):
    str_points = re.findall(pattern,command)
    waypoints = list()
    for waypoint in str_points:
        values = waypoint.split(sep=',')
        waypoints.append(((int)values[0]*ft2pt,(int)values[1]*ft2pt),(int)values[2])
    return('NEW_PATH_R',waypoints)

def quit(command):
    pass

    
if __name__ == '__main__':
    serv_addr,serv_port = sys.argv[1:3]
    serv = socket.create_connection((serv_addr,int(serv_port)))
    print('Connection Established')
    while running:
        print('>>> ',end='')
        cmd = input().split()
        try:
            msg = CMDS[cmd[1]](cmd[2::])
            ret = serv.sendall(pickle.dumps(msg,protocol=2))
        except ValueError:
            print('Command not found')
        except Exception:
            print('Incorrect Argument')
