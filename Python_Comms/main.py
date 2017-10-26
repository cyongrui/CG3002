import handshake as hs
import serial
import time
import packet as pkt
import numpy as np
import cPickle as pickle
from sklearn import preprocessing as pp
import client as clt
import sys
import csv

SERVER_EXIST = False

if __name__ == '__main__':
    port=serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=3.0)
    
    # Initialize client server communication
    if len(sys.argv) == 3:
        IP_ADDR = sys.argv[1]
        PORT_NUM = sys.argv[2]
        SERVER_EXIST = True
        my_client = clt(IP_ADDR, PORT_NUM)
        print("Established connection with server")

    hs.handshake(port)
    # Handshake done
    
    log_file = csv.writer(open ('data.csv', 'w'), delimiter=',', lineterminator='\n')
    
    data_buf = []
    power_buf = []

    # Load machine learning model
    clf = pickle.load(open('model40.pkl',"rb"))
    model_input_size = 40
    # Wait for 2 seconds for Arduino to collect data
    time.sleep(2)

    while True:
        print("Length of databuf: {databufsize}").format(databufsize=len(data_buf))
        print("Length of powerbuf: {powerbufsize}").format(powerbufsize=len(power_buf))

        # Reading sensor data
        id = 0
        req = pkt.generate_msg(pkt.REQUEST_DATA, 0)
        while True:
            port.flushInput()   # clear port to get fresh read
            port.write(req)     # Send request/ ACK
            data_msg = port.read(pkt.DATA_LEN)
            msg_field = pkt.read_data_msg(data_msg)
            if msg_field is None:
                print("Data Read: Issues with timeout or checksum")
                continue
            msg_type, msg_id, data = msg_field
            # Message type is wrong. Resend request
            if msg_type != pkt.SENSOR_DATA and msg_type != pkt.DONE:
                print("Data Read: Invalid message type. Resend Req/ ACK")
                continue
            if msg_id != id:
                print("Data Read: Previous ACK not received. Resend ACK.")
                continue
            if msg_type == pkt.SENSOR_DATA: # Message is correct
                data_buf.append(data)
                req = pkt.generate_msg(pkt.ACK, id)
                id += 1
                print("Data: {sensor_data}".format(sensor_data=data))
                log_file.writerow(data)
                continue
            if msg_type == pkt.DONE:    # Last message
                req = pkt.generate_msg(pkt.ACK, id)
                port.write(req)
                print("All data messages read")
                break

        # Machine learning algorithm
        # pass first n element into the algorithm
        if len(data_buf) >= model_input_size:
	    input = np.hstack(data_buf[-model_input_size:]).reshape(1, -1)
	    input = pp.normalize(input.astype(np.float64))
	    label = clf.predict(input)
            print(label)
	    data_buf = []

        # Reading power data
        id = 0
        req = pkt.generate_msg(pkt.REQUEST_POWER, 0)
        while True:
            port.flushInput()   # clear port to get fresh read
            port.write(req)     # Send request/ ACK        
            power_msg = port.read(pkt.POWER_LEN)
            msg_field = pkt.read_power_msg(power_msg)
            if msg_field is None:
                print("Power Read: Issues with timeout or checksum")
                continue
            msg_type, msg_id, power = msg_field
            if msg_type != pkt.POWER_DATA and msg_type != pkt.DONE:
                print("Power Read: Invalid message type. Resend Req/ ACK")
                continue
            if msg_id != id:
                print("Power Read: Previous ACK not received. Resend ACK.")
                continue
            if msg_type == pkt.POWER_DATA:  # Message is correct
                power_buf.append(power)
                req = pkt.generate_msg(pkt.ACK, id)
                id += 1
                print("Power: {power}".format(power=power))
                continue
            if msg_type == pkt.DONE:    # Last message
                req = pkt.generate_msg(pkt.ACK, id)
                port.write(req)
                print("All power messages read")
                break

        print("cycle done")
        # Done one iteration

        if SERVER_EXIST:
            message = my_client.formatMessage(power, label)
            encrypted = my_client.encrypt(message)
            my_client.sock.send(encrypted)

        time.sleep(1.0)

