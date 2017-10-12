import crc8
import struct

SENSOR_DATA = 1
HELLO = 2
HELLO_DONE = 3
ACK = 4
NAK = 5
DONE = 6
REQUEST_DATA = 7
REQUEST_POWER = 8
POWER_DATA = 9

REPLY_LEN = 3
POWER_LEN = 11 #5
DATA_LEN = 75 #63


def generate_msg(pkt_type, id):
    type_hex = format(pkt_type, '02x')
    id_hex = format(id, '02x')
    crc = crc8.calc_check_sum(type_hex + id_hex)
    return [chr(pkt_type), chr(id), chr(crc)]


def read_reply_msg(bytes_ls):
    if len(bytes_ls) < 3:
        print("Listen timeout")
        return None 
    elif not checkCRC(bytes_ls):
        print("CRC error")
        return None
    else:
        return [ord(bytes_ls[0]), ord(bytes_ls[1]), ord(bytes_ls[2])]


def read_data_msg(bytes_ls):
    if len(bytes_ls) < DATA_LEN:
        print("Read data message timeout")
        return None
    elif not checkCRC(bytes_ls):
        print("CRC error")
        return None
    else:
        msg_type = get_type(bytes_ls)
        msg_id = get_id(bytes_ls)
        data = get_data(bytes_ls)
        return msg_type, msg_id, data


def read_power_msg(bytes_ls):
    if len(bytes_ls) < POWER_LEN:
        print("Read power message timeout")
        return None
    elif not checkCRC(bytes_ls):
        print("CRC error")
        return None
    else:
        msg_type = get_type(bytes_ls)
        msg_id = get_id(bytes_ls)
        power = get_power(bytes_ls)
        return msg_type, msg_id, power


def get_type(bytes_ls):
    return ord(bytes_ls[0])


def get_id(bytes_ls):
    return ord(bytes_ls[1])


def get_data(bytes_ls):
    data = []
    for i in range(18):
        float_bytes = bytes_ls[(4*i+2):(4*i+6)]
        sensor_reading = struct.unpack('<f', float_bytes)[0]
        data.append("{0:.2f}".format(sensor_reading))
    return data


def get_power(bytes_ls):
    power = []
    for i in range(2):
        float_bytes = bytes_ls[(4 * i + 2):(4 * i + 6)]
        power_reading = struct.unpack('<f', float_bytes)[0]
        power.append("{0:.2f}".format(power_reading))
    return power
    #return [ord(bytes_ls[2]), ord(bytes_ls[3])]

def checkCRC(bytes_ls):
    hex_values = map(lambda x: format(ord(x), '02x'), bytes_ls[:-1])
    hex_string = "".join(hex_values)
    crc = crc8.calc_check_sum(hex_string)
    return crc == ord(bytes_ls[-1])
