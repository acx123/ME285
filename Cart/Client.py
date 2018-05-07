from __future__ import print_function
import socket
import sys
import pickle
import re


def setmode(command):
    try:
        MODES[command]
    except:
        raise Exception('Incorrect Argument: ',command)
        return None
    return ('CM',command)


def setpath(command):
    str_points = re.findall(pattern,command)
    print(str_points)
    waypoints = list()
    for waypoint in str_points:
        values = waypoint.split(sep=',')
        waypoints.append([((int)(values[0])*ft2pt,(int)(values[1])*ft2pt),(int)(values[2])])
    return('NEW_PATH_R',waypoints)

def quit():
    running = False
    return ''


running = True
CMDS = {'quit':quit,'setmode':setmode,'setpath':setpath}
pattern = '(\d+(?:,\s*\d+)*)'
ft2pt = 1/0.000003
MODES = {'MANUAL':1,'ASSIST':1}

def quit(command):
    pass


if __name__ == '__main__':
    serv_addr,serv_port = sys.argv[1:3]
    serv = socket.create_connection((serv_addr,int(serv_port)))
    print('Connection Established: ')
    while running:
        print('>>> ',end='')
        cmd = input().split()
        try:
            cmd_form = CMDS[cmd[0]](cmd[1])
            msg = pickle.dumps(cmd_form,protocol=2)
            ret = serv.sendall(msg)
        except ValueError:
            print('Command not found')
        except Exception as excp:
            print(excp)
