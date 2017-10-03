# To be run on bootup
# Establish a 3 way handshake with Arduino Mega

import serial
import time

def readlineCR(port):
    rv=[]
    ch = port.read(3)
    rv.append(ch)
##    for i in range(3):
##        ch=port.read().encode('ascii')
##        rv.append(ch)
    return rv
        #if ch=='\r':
        #    print('eof found')
        #    return rv
        #elif ch=='':
        #    print('no text found')
        #    return rv
        #elif len(rv) == 6
        #    return rv

port=serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=3.0)

i=0
while(True):
    data = ['\x00', '\xff', '\x02', i]
##    print type(data[i])
    #data_chr = map(chr, data)
    port.write(data)
##    port.write(data)
##    print(i)
##    i = i + 1
##    rcv=readlineCR(port)
##    print(str(rcv))
##    print(rcv)
    #print('Done')
    #port.write('\r\nYou sent:' +rcv)
    time.sleep(1)
