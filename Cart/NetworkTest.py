import socket
import pickle
import threading

def _network():
    serv = socket.socket()
    serv.bind(('',4442))
    serv.listen(1)
    cli,cli_addr = serv.accept()
    serv.listen(0)
    print('Connection:',cli_addr)
    while True:
        try:
            pickled_data = cli.recv(1024)
            print(pickled_data)
            dat = pickle.loads(pickled_data)
            print(dat)
        except EOFError:
            cli.close()
            serv.close()
            return


if __name__ == '__main__':
        _network()
