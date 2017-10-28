import packet as pkt
import time

def unsuccesful_msg():
    unsuccesful_msg.counter += 1

unsuccesful_msg.counter = 0

def reset(port):
    reset = pkt.generate_msg(pkt.RESET, 0)
    port.flushInput()
    port.write(reset)
    time.sleep(3)


def handshake(port):
    hello = pkt.generate_msg(pkt.HELLO, 0)
    while (1):
        if(unsuccesful_msg.counter >= pkt.MAX_FAILED_MSG):
            print("Resetting Arduino")
            unsuccesful_msg.counter = 0
            reset(port)
        port.flushInput()   # clear port to get fresh read
        port.write(hello)   # Send hello
        res = port.read(3)  # Read response
        if len(res) < 3:
            unsuccesful_msg()
            print("Handshake: timeout!")
            continue

        res_message = pkt.read_reply_msg(res)
        if res_message is None:
            unsuccesful_msg()
            continue
        if res_message[0] != pkt.ACK:
            unsuccesful_msg()
            print(unsuccesful_msg.counter)
            print("Handshake: Invalid message!")
            continue

        # ACK received
        port.write(pkt.generate_msg(pkt.HELLO_DONE, 0))
        unsuccesful_msg.counter = 0
        break

    print("handshake done!")
    
