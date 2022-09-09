import argparse
import os
import time
import socket
import sys
import struct

# regular frames
ID_NACK = 0
ID_ACK = 1
ID_HANDSHAKE = 2
ID_WIFI = 3
ID_SET_NODE_ID = 4
ID_LOGGING = 5
ID_GET_POSITION = 6


#beacon frames
ID_BEACON_SET_PURPOSE = 32
ID_GET_STATUS = 33
ID_RETURN_STATUS = 34
ID_BEACON_FIRMWARE = 42

#telemetry frames
ID_TELEMETRY_START_STREAMING = 32
ID_TELEMETRY_FIRMWARE = 42

PORT = 55671

def send(buf):
    totalsent = 0
    while totalsent < len(buf):
        sent = s.send(buf[totalsent:])
        if sent == 0:
            raise RuntimeError("socket connection broken")
        totalsent = totalsent + sent

def receive(msg_len):
    chunks = []
    bytes_recd = 0
    while bytes_recd < msg_len:
        chunk = s.recv(min(msg_len - bytes_recd, 2048))
        if chunk == b'':
            raise RuntimeError("socket connection broken")
        chunks.append(chunk)
        bytes_recd = bytes_recd + len(chunk)
        buf = b''.join(chunks)
        return buf

def wait_ack():
    buf = receive(1)
    if buf[0] == 1:
        return True
    else:
        return False

parser = argparse.ArgumentParser(description='generic configurator')
parser.add_argument('ip', type=str,
                    help='ip')

parser.add_argument('--id', type=int,
                    help='set the node id')

parser.add_argument('--wifi', nargs = 2, type=str,
                    help='--wifi [ssid] [password]')

parser.add_argument('--beacon-firmware', type=str,
                    help="upload a firmware file")

parser.add_argument('--beacon-verify', action='store_true',
                    help="verify and apply the new firmware")

parser.add_argument('--beacon-purpose', type=str, help = "repeater, origin, x, y or z")

args = parser.parse_args()

print("connecting...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((args.ip, PORT))
print("connected")

if args.wifi:
    print("updating wifi")
    buf = bytearray([ID_WIFI])
    buf += struct.pack("32s64s", bytes(args.wifi[0], "utf-8"), bytes(args.wifi[1], "utf-8"))
    send(buf)
    if not wait_ack():
        print("failed to set wifi")

if args.id:
    print("updating node id")
    buf = bytearray([ID_SET_NODE_ID, args.id])
    send(buf)
    if not wait_ack():
        print("failed to set node id")


success = True
if args.beacon_firmware:
    print("updating firmware")
    f = open(args.beacon_firmware, "rb")
    size = os.path.getsize(args.beacon_firmware)
    send(bytearray([ID_BEACON_FIRMWARE]))
    send(struct.pack("<Q", size))
    sent = 0
    while True:
        buf = f.read(255)
        if len(buf) == 0:
            break
        send(buf)
        sent += len(buf)
        print(f"sent ota packet. {round(sent/size * 100, 2)}% done")
    if not wait_ack():
        success = False
        print("failed to send update\naborting")

if success and args.beacon_verify:
    if args.beacon_firmware:
        print("upload done. attempting to verify")
        time.sleep(5)
        while True:
            try:
                s.close()
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                print("trying to connect...")
                s.connect((args.ip, PORT))
                s.settimeout(None)
                break
            except:
                pass
        print("connected")

    print("sending a zero size OTA packet to confirm a successful update")
    send(bytearray([ID_BEACON_FIRMWARE]))
    send(bytearray([0] * 8))
    if not wait_ack():
        print("could not confirm the update and it will probably be rolled back")
    else:
        print("updated!")

if args.beacon_purpose:
    val = 0
    if args.beacon_purpose == "origin":
        val = 0
    elif args.beacon_purpose == "x":
        val = 1
    elif args.beacon_purpose == "y":
        val = 2
    elif args.beacon_purpose == "z":
        val = 3
    elif args.beacon_purpose == "repeater":
        val = 4
    else:
        raise Exception("invalid purpose")

    send(bytearray([ID_BEACON_SET_PURPOSE]))
    send(bytearray([val]))

s.close()