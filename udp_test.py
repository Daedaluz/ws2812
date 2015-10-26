#!/usr/bin/python2
import socket

IP = '192.168.9.119'
PORT = 50007

DATA = '\x10\x00\x00\x00\x10\x00\x00\x00\x10'*16
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
s.connect((IP, PORT))
s.send(DATA)
s.close()
