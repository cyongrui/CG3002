from Crypto.Cipher import AES
from Crypto import Random
import socket
import base64
import random

BLOCK_SIZE = 16
FILE = "/tmp/token"

class client:
    def __init__(self, ip_addr, port_num):

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (ip_addr, port_num)
        print('Connecting to %s port %s' % server_address)
        self.sock.connect((ip_addr, port_num))
        print('Connected')

    def send(self, message):
        encrypted = self.encrypt(message)
        self.sock.send(encrypted)

    def encrypt(self, message):
        message = self.pad(message)
        IV = Random.new().read(BLOCK_SIZE)
        cipher = AES.new(self.getKey(), AES.MODE_CBC, IV)
        return base64.b64encode(IV + cipher.encrypt(message))


    # Returns secret key if it exists
    # Otherwise create a new random secret key
    def getKey(self):
        # TODO
        # Encrypt secret key
        # Currently stored as bytes
        try:
            file = open(FILE, 'rb')
            key = file.read()
            if len(key) != 16:
                raise IOError
        except IOError:
            file = open(FILE, 'wb')
            key = Random.new().read(BLOCK_SIZE)
            file.write(key)
            key_str = base64.b64encode(key)
            print('New key is generated: %s' % key_str)
            print('key_str length: %s' % len(key_str))
        finally:
            file.close()
            return key

    # For debugging
    # Produce a dummy message
    # Message fields:
    #   action
    #   voltage
    #   current
    #   power
    #   cumpower
    #   goal
    def getDummyMessage(self):
        action = 'dummy'
        voltage = random.randint(350, 600) / 100
        current = random.randint(0, 11) / 4
        power = random.randint(10, 35)
        cumpower = random.randint(35, 60)
        message = '#' + action + '|' + str(voltage) + '|' + str(current) + '|' + str(power) + '|' + str(cumpower)
        return message.encode('utf8')

    def pad(self, str):
        return b"\0" * (AES.block_size - len(str) % AES.block_size) + str

    def close(self):
        self.sock.close()

if __name__ == '__main__':

    import sys
    import time
    if len(sys.argv) != 3:
        print('Invalid number of arguments')
        print('python server.py [IP address] [Port]')
        sys.exit()

    ip_addr = sys.argv[1]
    port_num = int(sys.argv[2])
    my_client = client(ip_addr, port_num)
    while True:
        message = my_client.getDummyMessage()
        encrypted = my_client.encrypt(message)
        my_client.sock.send(encrypted)
        time.sleep(3)
