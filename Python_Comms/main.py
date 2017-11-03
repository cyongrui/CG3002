
import handshake as hs
import serial
import time
import packet as pkt
import numpy as np
import cPickle as pickle
from sklearn import preprocessing as pp
from client import client
import sys
#import csv
from scipy.stats import mode

SERVER_EXIST = False
cumulative_power = 0.0
toPrint = True
label = 0

if __name__ == '__main__':
    port = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=3.0)

    hs.handshake(port)
    # Handshake done
 
    # Initialize client server communication
    if len(sys.argv) == 3:
        while not SERVER_EXIST:
            IP_ADDR = sys.argv[1]
            PORT_NUM = int(sys.argv[2])
            try:
                my_client = client(IP_ADDR, PORT_NUM)
            except:
                time.sleep(5)
                continue
            SERVER_EXIST = True
            print("Established connection with server")

    #log_file = csv.writer(open('0.csv', 'w'), delimiter=',', lineterminator='\n')

    data_buf = []
    power_buf = []

    # Load machine learning model
    clf, scaler = pickle.load(open('model40.pkl', "rb"))
    m_input_size = 40
    # Wait for 2 seconds for Arduino to collect data
    time.sleep(2)

    while True:
        # print("Length of databuf: {databufsize}").format(databufsize=len(data_buf))
        # print("Length of powerbuf: {powerbufsize}").format(powerbufsize=len(power_buf))

        # Reading sensor data
        id = 0
        req = pkt.generate_msg(pkt.REQUEST_DATA, 0)
        while True:
            port.flushInput()  # clear port to get fresh read
            port.write(req)  # Send request/ ACK
            data_msg = port.read(pkt.DATA_LEN)
            msg_field = pkt.read_data_msg(data_msg)
            if hs.unsuccesful_msg.counter >= pkt.MAX_FAILED_MSG:
                print("Resetting Arduino")
                hs.reset(port)
                hs.handshake(port)
                break
            if msg_field is None:
                print("Data Read: Issues with timeout or checksum")
                hs.unsuccesful_msg()
                continue
            msg_type, msg_id, data = msg_field
            # Message type is wrong. Resend request
            if msg_type != pkt.SENSOR_DATA and msg_type != pkt.DONE:
                print("Data Read: Invalid message type. Resend Req/ ACK")
                hs.unsuccesful_msg()
                continue
            if msg_id != id:
                print("Data Read: Previous ACK not received. Resend ACK.")
                hs.unsuccesful_msg()
                continue
            if msg_type == pkt.SENSOR_DATA:  # Message is correct
                data_buf.append(data)
                req = pkt.generate_msg(pkt.ACK, id)
                id += 1
                if toPrint:
                    print("Data: {sensor_data}".format(sensor_data=data))
                    toPrint = False
                #log_file.writerow(data)
                hs.unsuccesful_msg.counter = 0
                continue
            if msg_type == pkt.DONE:  # Last message
                req = pkt.generate_msg(pkt.ACK, id)
                port.write(req)
                print("All data messages read")
                toPrint = True
                break


            # Machine learning algorithm
        label = 0
        if len(data_buf) >= m_input_size + 11:
            data_buf = np.array(data_buf[-(m_input_size + 11):])
            # print 'length: {}'.format(len(data_buf))	# check to see if length is 51

            data_diff = []
            for i in range(len(data_buf)-1):		# Iterate from 0 to 50
                data_diff.append(data_buf[i+1] - data_buf[i])

            inputs = scaler.transform(data_diff)	# data_diff should have 50 data
            test = []
            for i in range(10):
                test.append(np.hstack(inputs[i:i+m_input_size]))

            label = mode(clf.predict(test))[0][0]	# take the mode of 10 predictions
            print(label)
            data_buf = []


        # Reading power data
        id = 0
        req = pkt.generate_msg(pkt.REQUEST_POWER, 0)
        while True:
            port.flushInput()  # clear port to get fresh read
            port.write(req)  # Send request/ ACK
            power_msg = port.read(pkt.POWER_LEN)
            msg_field = pkt.read_power_msg(power_msg)
            if hs.unsuccesful_msg.counter >= pkt.MAX_FAILED_MSG:
                print("Resetting!")
                hs.reset(port)
                hs.handshake(port)
                break
            if msg_field is None:
                print("Power Read: Issues with timeout or checksum")
                hs.unsuccesful_msg()
                continue
            msg_type, msg_id, power = msg_field
            if msg_type != pkt.POWER_DATA and msg_type != pkt.DONE:
                print("Power Read: Invalid message type. Resend Req/ ACK")
                hs.unsuccesful_msg()
                continue
            if msg_id != id:
                print("Power Read: Previous ACK not received. Resend ACK.")
                continue
            if msg_type == pkt.POWER_DATA:  # Message is correct
                power_buf.append(power)
                req = pkt.generate_msg(pkt.ACK, id)
                cumulative_power += float(power[2])
                id += 1
                print("Power: {power}".format(power=power))
                hs.unsuccesful_msg.counter = 0
                continue
            if msg_type == pkt.DONE:  # Last message
                req = pkt.generate_msg(pkt.ACK, id)
                port.write(req)
                print("All power messages read")
                break

        # print("cycle done")
        # Done one iteration

        if SERVER_EXIST and label != 0:
            print("Sending")
            message = my_client.formatMessage(power_buf[len(power_buf) - 1], label, cumulative_power)
            encrypted = my_client.encrypt(message)
            my_client.sock.send(encrypted)

        time.sleep(5.0)

