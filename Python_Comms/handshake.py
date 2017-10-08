import packet as pkt

def handshake(port):
    hello = pkt.generate_msg(pkt.HELLO, 0)
    while (1):
        port.flushInput()   # clear port to get fresh read
        port.write(hello)   # Send hello
        res = port.read(3)  # Read response
        if len(res) < 3:
            print("Handshake: timeout!")
            continue

        res_message = pkt.read_reply_msg(res)
        if res_message is None:
            continue
        if res_message[0] != pkt.ACK:
            print("Handshake: Invalid message!")
            continue

        # ACK received
        port.write(pkt.generate_msg(pkt.HELLO_DONE, 0))
        break

    print("handshake done!")
    
