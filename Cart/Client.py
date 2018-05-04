import socket
import sys
import Controller
from __future__ import print_function

running = True
CMDS = {'quit':quit,'setmode':setmode:'setpath'}

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
        except ValueError:
            print('Command not found')
        except Exception:
            print('Incorrect arguments')
