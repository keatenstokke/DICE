# Python script to test Ethernet throughput
# Authors: Atiyeh Panahi & Keaten Stokke

# Import all needed libraries
import os
import sys
import time
import socket
import struct
import fnmatch

# Print start of program
print("\nRunning client program... \n")

# Create a TCP/IP socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Specify the servers IP, port number, and buffer size
TCP_IP = '192.168.1.10'
TCP_PORT = 7
BUFFER_SIZE = 1460 # 146 numbers stored
receiving_time = 0
packaging_time = 0

# Connect the socket to the port where the server is listening
server_address = (TCP_IP, TCP_PORT)
print 'Connecting to IP: ' + TCP_IP
print 'Connecting to Port: ' + str(TCP_PORT)
s.connect(server_address)

# Open test throughput file (512 MB)
file = open("SpeedTest_512MB.dat", 'r')
temp = file.readline()
sendData = ''
while temp:
	sendData += temp
	temp = file.readline()

# Send a large piece of data
print '\nSending data'
start1 = time.time()
s.sendall(sendData)

amount_received = 0
amount_expected = len(sendData)

while amount_received < amount_expected:
	data = s.recv(BUFFER_SIZE)
	amount_received += len(data)
	#print data

	
end1 = time.time()
total = end1 - start1

print '\nTotal time'
print total
	
print '\nClosing the socket.'
s.close()
