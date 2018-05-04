import socket
import pickle
import threading

addr = ('192.268.7.2',4440)

serv = socket.socket.bind(address)

cli,cli_addr = serv.accept()

socket.socket.rec
print('Connection: ',cli_addr)

while True:
    data = cli.recv()
    if not data:
        break
    cli.sendall(data)

cli.close()
