#!/usr/bin/python2
import socket
import sys

#IP = '192.168.3.136'
IP = '10.37.201.187'
PORT = 50007

DATA = "".join(sys.argv[1:]).decode("hex")

#DATA = '\x10\x00\x00\x00\x10\x00\x00\x00\x10'*16
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
s.connect((IP, PORT))
s.send(DATA)
s.close()
