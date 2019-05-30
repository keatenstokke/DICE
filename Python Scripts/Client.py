# Preproccesing and postprocessing code for DICE FPGA-based acceleration
# Authors: Atiyeh Panahi & Keaten Stokke

# Import all needed libraries
import os
import sys
import time
import struct
import socket
import fnmatch
import numpy as np
from PIL import Image
from codecs import decode
from bitstring import Bits

# Program Start
starttime = time.time()
#print '\nStart: Converting all .tif images to 32-bit single precision floating-point (IEEE 754)'
#print 'All converted images will be stored in imageName_#.txt\n'

# Counter for which image is currently being processed
imgNum = 0

# Working directory that contains the program and images ('pwd' on macOS Terminal shows the current directory)
pwd = 'C:\Users\knstokke\Documents\Python'

# Name of the file type that is to be processed; currently .tif
fileType = '.tif'

# Total number of images to process; reads directory to count total .tif images exist for processing
totalImg = len(fnmatch.filter(os.listdir(pwd), '*' + fileType))
#print 'Total number of files of type ' + fileType + ' to be processed: ' + str(totalImg)

# Counter for how many images have been processed
processedImg = 0

# Loop through all images that need to be processed
startprocesstime = time.time()
while processedImg < totalImg:

	# Open the image that needs to be processed
	imgName = 'Image_' + str(imgNum)
	img = Image.open(imgName + fileType)

	# Get all of the pixel values from the image
	allPixels = list(img.getdata(0))

	# Counter for keeping track of the number of pixels in each image
	pixelCount = 0

	# Create a file to write all of the pixel values to
	file = open(imgName + '.txt', 'w+')

	# Loop through each pixel in the image and convert them
	#print '\nProcessing ' + imgName + '...'
	for pixel in allPixels:
	
		# Normalizes the pixel 
		pixel_normalized = pixel / float(255)
		
		# Converts each pixel to its corresponding IEEE 754 representation
		pow = -1
		fraction_dec = 0
		fraction_bin = list("00000000000000000000000")
		n = pixel_normalized

		if pixel_normalized >= 0:
			sign = '0';
		else:
			sign  = '1';

		if pixel_normalized == 0:
			Output_bin = "00000000000000000000000"
		else:
			while abs(n) < 1 or abs(n) >= 2:
				#n = (Input_dec) / (2 ** pow)
				try:
					n = pixel_normalized / (2 ** pow)
				except ZeroDivisionError:
					n = 0
				pow = pow -1

			exp_int = pow + 1 + 127
			exp_bin_str = str(Bits(int = exp_int, length = 8).bin)
			fraction_dec = n - 1;

			for i in range(0, 23):
				temp = fraction_dec * 2
				if temp < 1:
					fraction_bin[i] = '0'
					fraction_dec = temp
				else:
					fraction_bin[i] = '1'
					fraction_dec = temp - 1

			final_fraction = "".join(fraction_bin);
			Output_bin = str(sign) + str(exp_bin_str) + str(final_fraction)

		# Converts each IEEE 754 binary pixel value to decimal for packet trasnfer (represents the IEEE in 32-bit binary)
		dec = int(Output_bin, 2)

		# Writes the pixels decimal value to the .txt file
		file.write(str(dec) + '\n')

		# Increments the counter to keep track of the number of pixels in each image
		pixelCount += 1

	# Close the image opened and the file that all the pixel values were written to for each image
	file.close()
	img.close()

	# Print the total number of pixels converted from the image
	#print 'Total pixel count is: ' + str(pixelCount)

	# Increment the counter of images processed and image being processed
	processedImg += 1
	imgNum += 1

# Time stamp for how long the iamge conversions took
endprocesstime = time.time()

# Start the client connection to the server (FPGA)
#print("\nRunning client program... \n")

# Create a TCP/IP socket (UDP to be explored in the future)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Define the servers IP, port number, and buffer size
TCP_IP = '192.168.1.10'
TCP_PORT = 7
BUFFER_SIZE = 1440 # 146 numbers stored per packet (1460 bytes per packet w/o header information)

# Connect the socket to the port where the server is listening
server_address = (TCP_IP, TCP_PORT)

s.connect(server_address)

print 'Connected to IP: ' + TCP_IP
print 'Connected to Port: ' + str(TCP_PORT)

#print 'Default Time_Out: ' + str(s.gettimeout())
s.settimeout(None)
#print 'Time_Out: ' + str(s.gettimeout())

# Begin of transferring files over Ethernet to FPGA-based server
fileNum = 0
fileType = '.txt'
transferstarttime = time.time()

# Total number of images to process; reads directory to count total .tif images exist for processing
totalFiles = len(fnmatch.filter(os.listdir(pwd), '*' + fileType))
print 'Total number of files of type ' + fileType + ' to be processed: ' + str(totalFiles)

# Loops through each text file of converted pixel data
while fileNum < totalFiles:
	# Opening a .txt file with the image data stored in it
	imgName = 'Image_' + str(fileNum)
	print 'Sending: ' + str(imgName)
	file = open(imgName + fileType, 'r')
	pixel = file.readline()
	bigString = ''
	
	# Loops through each line in a file to concat the data to send as packets
	while pixel:
		# Send data as a packet through the socket (rstrip() gets rid of the trailing \n)
		pixel = pixel.rstrip()
		
		# Verfies that each number is of a predetermined length of 10 digits so they can be broken up on the server side
		if(len(pixel) == 1):
			pixel = '0000000000'
		elif(len(pixel) == 9):
			pixel = '0' + pixel
		
		bigString += pixel
		pixel = file.readline()

	# Transmits the data
	s.sendall(bigString)
	amount_received = 0
	amount_expected = len(bigString)

	# For receiving data via echo server 
	while amount_received < amount_expected:
		data = s.recv(BUFFER_SIZE)
		amount_received += len(data)

	# Close the file and move to the next one 
	file.close()
	fileNum += 1

print 'Results '
var = '1234567890'
amount_received2 = 0
amount_expected2 = len(var)
BUFFER_SIZE2 = len(var)
bigString_outputs = ""

# Dummy data value so the last sent data gets echoed back

s.sendall(var)

while amount_received2 < amount_expected2:
	data = s.recv(BUFFER_SIZE2)
	bigString_outputs += data
	amount_received2 += len(data)
	print 'Received: ' + data

# Timestamp to mark how long data transfer took
transferendtime = time.time()

# Timestamp to mark how long the output processing takes
outputstarttime = time.time()
outputs = bigString_outputs.split(",")

# Create a file to store the results
file = open('DICE_Solutions.txt', 'w+')
line = ""
# Write the header to the output file and given parameters (Parameters will be updated as more functionality is added to the core DICe Verilog-based IPs)
file.write('***')
file.write('\n*** Digital Image Correlation Engine (DICe), (git sha1: v1.0-beta.10-9-gd0eb4e) Copyright 2015 National Technology & Engineering Solutions of Sandia, LLC (NTESS)')
file.write('\n***')
file.write('\n*** Reference image: *TBD*')
file.write('\n*** Deformed image: *TBD*')
file.write('\n*** DIC method : Remoate *TBD*')
file.write('\n*** Correlation method: *TBD*')
file.write('\n*** Interpolation method: *TBD*')
file.write('\n*** Image gradient method: *TBD*')
file.write('\n*** Optimization method: *TBD*')
file.write('\n*** Projection method: *TBD*')
file.write('\n*** Guess initialization method: *TBD*')
file.write('\n*** Seed location: N/A')
file.write('\n*** Shape functions: *TBD*')
file.write('\n*** Incremental correlation: *TBD*')
file.write('\n*** Subset size: *TBD*')
file.write('\n*** Step size: *TBD*')
file.write('\n*** Strain window: N/A')
file.write('\n*** Coordinates given with (0,0) as upper left corner of image, x positive right, y positive down')
file.write('\n*** Currently missing: SIGMA, GAMMA, BETA')
file.write('\n***')
file.write('\nFRAME, COORDINATE_X, COORDINATE_Y, DISPLACEMENT_X, DISPLACEMENT_Y, ROTATION_Z')

# Loops through the data to write in the defined order above
for output in outputs:
	# Binary to Decimal
	#print 'Output_int: ' + str(output)
	Input_int = int(output, 10)
	Input_str = str(Bits(int = Input_int, length = 32).bin)
	pow = 0
	fraction = 0

	sign_str = Input_str[0]
	exp_str = Input_str[1:9]
	fraction_str = Input_str[9:32]

	sign = int(sign_str, 2)
	exp = int(exp_str, 2) - 127

	for i in range(0, 23):
		pow = pow - 1;
		fraction = fraction + int(fraction_str[i], 2) * (2 ** pow)
	
	if(Input_int == 0):
		Output_dec = 0;
	else:
		Output_dec = ((-1) ** sign) * (1 + fraction) * (2 ** exp)
	#print 'Output_dec: ' + str(Output_dec)
	line += ', ' + str(Output_dec)
	
frameNumber = 1
file.write('\n' + str(frameNumber) + ', 30, 30' + line)
outputendtime = time.time()

# Close the socket connnection and output file
print '\nClosing the socket.'
s.close()
file.close()

# Timestamp to mark how long the entire process took
endtime = time.time()

# Output timing results
print 'Total image preproccesing execution time: ' + str((endprocesstime - startprocesstime))
print 'Total image transfer execution time: ' + str((transferendtime - transferstarttime))
print 'Total image postprocessing (outputs) execution time: ' + str((outputendtime - outputstarttime))
print 'Total program execution time: ' + str((endtime - starttime))
