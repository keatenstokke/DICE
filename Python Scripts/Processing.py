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
print '\nStart: Converting all .tif images to 32-bit single precision floating-point (IEEE 754)'

# Counter for which image is currently being processed
imgNum = 1

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
	if imgNum < 10:
		imgName = 'Image_0' + str(imgNum)
	else:
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

print '\nConnected to IP: ' + TCP_IP
print 'Connected to Port: ' + str(TCP_PORT)

#print 'Default Time_Out: ' + str(s.gettimeout())
s.settimeout(None)


# Send subset parameters to the FPGA
file = open('Subsets.txt', 'r')

params = file.readline()
paramNum = 1
bigString = ''
numSubsets = 0
param = int(params)
coordsX = []
coordsY = []

# Base (starting) address for subset shape
shape_addr = 8

while param != '':
	param = int(param)
	
	if(paramNum >= 9):
		if(paramNum % 5 == 4):
			coordsX.append(str(param))
		elif(paramNum % 5 == 0):
			coordsY.append(str(param))
	
	if paramNum == 5:
		numSubsets = int(param)
		
	if((paramNum >= 8) and (paramNum % 5 == 3)):
			# A square was detected
			if(param == 1):
				circle = 0
			# A circle was detected
			else:
				circle = 1

	# These values need to be converted to IEEE 754 floating point format based on the Verilog code
	if(((paramNum >= 8) and ((paramNum % 5 == 4) or (paramNum % 5 == 0) or (paramNum % 5 == 2))) or ((paramNum >= 8) and (paramNum % 5 == 1) and (circle == 1))):
		# Converts each corresponding parameter to its an IEEE 754 representation
		pow = 1
		fraction_dec = 0
		fraction_bin = list("00000000000000000000000")
		n = param

		if param >= 0:
			sign = '0';
		else:
			sign  = '1';

		if param == 0:
			Output_bin = "00000000000000000000000"
		else:
			while abs(n) < 1 or abs(n) >= 2:
				n = float((param)) / float((2 ** pow))
				pow = pow + 1

			exp_int = pow - 1 + 127
			exp_bin_str = str( bin(exp_int)[2:].zfill(8))
			
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

		dec = int(Output_bin, 2)
		# Covnert decimal to a string of length 10 (add leading 0's)
		while len(str(dec)) < 10:
			dec = '0' + str(dec)
	
		bigString += str(dec)
	else:
		# Covnert decimal to a string of length 10 (add leading 0's)
		while len(str(param)) < 10:
			param = '0' + str(param)
		bigString += str(param)
	
	paramNum += 1
	param = file.readline()


# Transmits the parameter data
s.sendall(bigString)
amount_received = 0
amount_expected = len(bigString)

# For receiving data via echo server
while amount_received < amount_expected:
	data = s.recv(BUFFER_SIZE)
	amount_received += len(data)


# Begin of transferring files over Ethernet to FPGA-based server
fileNum = 1
fileType = '.txt'
transferstarttime = time.time()
bigString = ''
# Total number of images to process; reads directory to count total .tif images exist for processing
totalFiles = len(fnmatch.filter(os.listdir(pwd), '*' + fileType))
totalFiles = totalFiles - 1
print '\nTotal number of files of type ' + fileType + ' to be processed: ' + str(totalFiles) + '\n'
sendtime1 = time.time()
imgNum = 1
# Send the first 2 frames to the FGPA
# Loops through each text file of converted pixel data
while fileNum < 3:
	# Opening a .txt file with the image data stored in it

	if imgNum < 10:
		imgName = 'Image_0' + str(fileNum)
	else:
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

endtime1 = time.time()
print 'Total time to send 2 frames: ' + str(endtime1 - sendtime1)
fileNum = fileNum - 1


newFrame = 0
wait = 'Wait'
bigString = ''

# Send one frame at a time when its requested from the FPGA
while fileNum < totalImg:
	if (newFrame == 1):
		# Send a new frame
		# Opening a .txt file with the image data stored in it
		fileNum += 1
		if fileNum < 10:
			imgName = 'Image_0' + str(fileNum)
		else:
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
		newFrame = 0

	elif(newFrame == 0):
		s.sendall(wait)

		amount_received = 0
		amount_expected = len(wait)

		# For receiving data via echo server
		while amount_received < amount_expected:
			data = s.recv(BUFFER_SIZE)
			amount_received += len(data)
			if(data == 'Send'):
				newFrame = 1
			else:
				newFrame = 0


# The last image has been sent, now we need to wait until its done processing until we can receive the results
doneSignal = 0
done = 'Done!'

while(doneSignal == 0):
	s.sendall(done)

	amount_received = 0
	amount_expected = len(done)

	# For receiving data via echo server
	while amount_received < amount_expected:
		data = s.recv(BUFFER_SIZE)
		amount_received += len(data)
		if(data == 'Okay!'):
			doneSignal = 1
		else:
			doneSignal = 0



var = 'Results'
doneSignalTwo = 0
bigString_outputs = ""

while(doneSignalTwo == 0):
	amount_received2 = 0
	amount_expected2 = 5 # 8 is the smallest output we can receive from the FPGA  #len(var)
	BUFFER_SIZE2 = 100 #len(var)

	# Dummy data value so the last sent data gets echoed back
	s.sendall(var)

	while amount_received2 < amount_expected2:
		data = s.recv(BUFFER_SIZE2)
		if(data == 'Finished'):
			doneSignalTwo = 1
			amount_received2 += len(data)
		else:
			doneSignalTwo = 0
			bigString_outputs += data
			amount_received2 += len(data)
			print 'Received: ' + data
		
			
print '\nAll results have been received!'
		
# Timestamp to mark how long data transfer took
transferendtime = time.time()


# Timestamp to mark how long the output processing takes
outputstarttime = time.time()

# Create as many files as subsets
fileCreate = 1
while fileCreate <= numSubsets:
	# Create a file to store the results
	fileName = 'DICE_Solutions_' + str(fileCreate) + '.txt'
	file = open(fileName, 'w+')
	
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
	file.write('\nFRAME, COORDINATE_X, COORDINATE_Y, DISPLACEMENT_X, DISPLACEMENT_Y, ROTATION_Z\n')
	file.close()
	fileCreate += 1


outputs = bigString_outputs.split(",")

frameNumber = 1
subsetNumber = 1
newlineCount = 0
line = ""

# Loops through the data to write in the defined order above
for output in outputs:
	# Binary to Decimal
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
	line += ', ' + str(Output_dec)

	newlineCount += 1

	# The number "3" should reflect how many values we expect from the FPGA; currently expects: DISPLACEMENT_X, DISPLACEMENT_Y, ROTATION_Z
	if(newlineCount == 3):
		fileName = 'DICE_Solutions_' + str(subsetNumber) + '.txt'
		file = open(fileName, 'a+')
		file.write(str(frameNumber) + ', ' + coordsX[subsetNumber - 1] + ', ' + coordsY[subsetNumber - 1] + line + '\n')
		file.close()
		newlineCount = 0
		line = ''

		
		if(subsetNumber == numSubsets):
			subsetNumber = 1
			frameNumber += 1
		else:
			subsetNumber += 1
		

outputendtime = time.time()

# Close the socket connnection and output file
print '\nClosing the socket.\n'
s.close()
file.close()

# Timestamp to mark how long the entire process took
endtime = time.time()

# Output timing results
print 'Total image preproccesing execution time: ' + str((endprocesstime - startprocesstime))
print 'Total image transfer execution time: ' + str((transferendtime - transferstarttime))
print 'Total image postprocessing (outputs) execution time: ' + str((outputendtime - outputstarttime))
print 'Total program execution time: ' + str((endtime - starttime))
