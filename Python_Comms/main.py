import handshake as hs
import serial
import time
import packet as pkt

if __name__ == '__main__':
    port=serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1.0)
    hs.handshake(port)
    # Handshake done

    data_buf = []
    power_buf = []

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
                #port.write(req)
                continue
            if msg_type == pkt.DONE:    # Last message
                req = pkt.generate_msg(pkt.ACK, id)
                port.write(req)
                print("All data messages read")
                break

        # Machine learning algorithm
        # pass first n element into the algorithm
        n = 5
        if len(data_buf) >= 5:
            del data_buf[:n]    # delete first n element


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
            if msg_type != pkt.POWER_DATA or msg_type != pkt.DONE:
                print("Power Read: Invalid message type. Resend Req/ ACK")
                continue
            if msg_id != id:
                print("Data Read: Previous ACK not received. Resend ACK.")
                continue
            if msg_type == pkt.POWER_DATA:  # Message is correct
                power_buf.append(power)
                req = pkt.generate_msg(pkt.ACK, id)
                id += 1
                print("Data: {power}".format(power=power))
                continue
            if msg_type == pkt.DONE:    # Last message
                req = pkt.generate_msg(pkt.ACK, id)
                port.write(req)
                print("All power messages read")
                break

        # Done one iteration
        time.sleep(1)


