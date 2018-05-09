import socket
import pickle
import threading

def _network():
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
        print(pickle.loads(pickled_data))



if __name__ == '__main__':
        _network()
