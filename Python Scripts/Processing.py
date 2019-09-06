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
from multiprocessing import Process

if __name__ == '__main__':
	# ********************* PROGRAM START *********************
	print '\nProgram start.\n'
	starttime = time.time()
	
	# Name of the file type that is to be processed; currently .tif
	fileType = '.tif'

	# Working directory that contains the program and images ('pwd' on macOS Terminal shows the current directory)
	pwd = 'C:\Users\knstokke\Documents\Python'
	
	# Total number of images to process; reads directory to count total .tif images exist for processing
	totalImg = len(fnmatch.filter(os.listdir(pwd), '*' + fileType))

	# ********************* Start Processing the Parameter Data *********************
	paramstart = time.time()

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
					#print("n: ", str(n))
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

	paramend = time.time()
# ********************* End Processing the Parameter Data *********************






# ********************* Starting connection to the FPGA *********************
	# Create a TCP/IP socket (UDP to be explored in the future)
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	#s.setsockopt(socket.IPPROTO_IP,socket.IP_DONTFRAG)
	# Define the servers IP, port number, and buffer size
	TCP_IP = '192.168.1.10'
	TCP_PORT = 7
	BUFFER_SIZE = 1344 #1350 # 146 numbers stored per packet (1460 bytes per packet w/o header information)

	# Connect the socket to the port where the server is listening
	server_address = (TCP_IP, TCP_PORT)

	s.settimeout(None)
	print '\nConnecting to the FPGA...'
	
	try:
		s.connect(server_address)
	except:
		print '\n\n***** Failed to connect to the FPGA!!! *****\n'
		print '***** Program terminated. *****\n\n'
		sys.exit()
	
	print 'Connected to IP: ' + TCP_IP
	print 'Connected to Port: ' + str(TCP_PORT)
# ********************* Connection to the FPGA Established *********************






# ********************* Starting Transfer of Parameter Data to the FPGA *********************
	paramsendstart = time.time()
	
	# Transmits the parameter data
	s.sendall(bigString)
	amount_received = 0
	amount_expected = len(bigString)

	# For receiving data via echo server
	while amount_received < amount_expected:
		data = s.recv(BUFFER_SIZE)
		amount_received += len(data)

	paramsendend = time.time()
# ********************* End Transfer of Parameter Data to the FPGA *********************

	
	
	
	
	
# ********************* Starting Transfer of Image Data to the FPGA *********************	
	startimagesend = time.time()

	print '\nTotal number of images of type ' + fileType + ' to be processed: ' + str(totalImg) + '\n'

	# Begin of transferring files over Ethernet to FPGA-based server
	fileNum = 1
	
	# Counter for which image is currently being processed
	imgNum = 1
	
	# Loop through the first two images and send the data to the FPGA
	while imgNum < 3:
		bigString = ''
		
		# Determing which image to open and setting the name
		if(totalImg < 100):
			if(imgNum < 10):
				imgName = 'Image_0' + str(imgNum)
			else:
				imgName = 'Image_' + str(imgNum)
		elif(totalImg < 1000):
			if(imgNum < 10):
				imgName = 'Image_00' + str(imgNum)
			elif(imgNum < 100):
				imgName = 'Image_0' + str(imgNum)
			else:
				imgName = 'Image_' + str(imgNum)
		elif(totalImg < 10000):
			if(imgNum < 10):
				imgName = 'Image_000' + str(imgNum)
			elif(imgNum < 100):
				imgName = 'Image_00' + str(imgNum)
			elif(imgNum < 1000):
				imgName = 'Image_0' + str(imgNum)
			else:
				imgName = 'Image_' + str(imgNum)
		elif(totalImg < 100000):
			if(imgNum < 10):
				imgName = 'Image_0000' + str(imgNum)
			elif(imgNum < 100):
				imgName = 'Image_000' + str(imgNum)
			elif(imgNum < 1000):
				imgName = 'Image_00' + str(imgNum)
			elif(imgNum < 10000):
				imgName = 'Image_0' + str(imgNum)
			else:
				imgName = 'Image_' + str(imgNum)
		else:
			print 'Exceeding 100,000 limit - modify Python script!'
			sys.exit()

		# Open the image that needs to be processed
		img = Image.open(imgName + fileType)

		# Get all of the pixel values from the image
		allPixels = list(img.getdata(0))

		# Send the image data to the FPGA
		print 'Sending: ' + str(imgName)
		norm = 255.0
		# Loop through each pixel in the image and send the data
		for pixel in allPixels:
			
			if(len(str(pixel)) == 1):
				dec = '00' + str(pixel)
			elif(len(str(pixel)) == 2):
				dec = '0' + str(pixel)
			else:
				dec = str(pixel)

			# Send data as a packet through the socket (rstrip() gets rid of the trailing \n)
			dec = dec.rstrip()
			
			# Concat all pixel values to be sent to the FPGA
			bigString += str(dec)
			
			# Send the single packet
			if(len(bigString) == 1344):
				# Transmits the data
				s.send(bigString)
				amount_received = 0
				amount_expected = len(bigString)
				
				# For receiving data via echo server
				while amount_received < amount_expected:
					data = s.recv(BUFFER_SIZE)
					amount_received += len(data)	
				bigString = ''

		# Close the currently opened image
		img.close()
		
		randVar = '000000000000000'
		s.send(randVar)
		amount_received = 0
		amount_expected = len(randVar)
		
		#For receiving data via echo server
		while amount_received < amount_expected:
			data = s.recv(BUFFER_SIZE)
			amount_received += len(data)
		# Close the file and move to the next one
		imgNum += 1
	# Finished sending the first two frames


	# Begin sending the rest of the frames when the FPGA requests them
	newFrame = 0
	wait = 'Wait'
	bigString = ''
	
	while imgNum <= totalImg:
		bigString = ''
		
		if (newFrame == 1):
			# Determing which image to open and setting the name
			if(totalImg < 100):
				if(imgNum < 10):
					imgName = 'Image_0' + str(imgNum)
				else:
					imgName = 'Image_' + str(imgNum)
			elif(totalImg < 1000):
				if(imgNum < 10):
					imgName = 'Image_00' + str(imgNum)
				elif(imgNum < 100):
					imgName = 'Image_0' + str(imgNum)
				else:
					imgName = 'Image_' + str(imgNum)
			elif(totalImg < 10000):
				if(imgNum < 10):
					imgName = 'Image_000' + str(imgNum)
				elif(imgNum < 100):
					imgName = 'Image_00' + str(imgNum)
				elif(imgNum < 1000):
					imgName = 'Image_0' + str(imgNum)
				else:
					imgName = 'Image_' + str(imgNum)
			elif(totalImg < 100000):
				if(imgNum < 10):
					imgName = 'Image_0000' + str(imgNum)
				elif(imgNum < 100):
					imgName = 'Image_000' + str(imgNum)
				elif(imgNum < 1000):
					imgName = 'Image_00' + str(imgNum)
				elif(imgNum < 10000):
					imgName = 'Image_0' + str(imgNum)
				else:
					imgName = 'Image_' + str(imgNum)
			else:
				print 'Exceeding 100,000 limit - modify Python script!'
				sys.exit()
			
			bigString = ''


			# Open the image that needs to be processed
			img = Image.open(imgName + fileType)

			# Get all of the pixel values from the image
			allPixels = list(img.getdata(0))
			
			# Send the image data to the FPGA
			print 'Sending: ' + str(imgName)

			# Loop through each pixel in the image and send the data
			for pixel in allPixels:
					
				if(len(str(pixel)) == 1):
					dec = '00' + str(pixel)
				elif(len(str(pixel)) == 2):
					dec = '0' + str(pixel)
				else:
					dec = str(pixel)

				# Send data as a packet through the socket (rstrip() gets rid of the trailing \n)
				dec = dec.rstrip()
				
				# Concat all pixel values to be sent to the FPGA
				bigString += str(dec)
				
				if(len(bigString) == 1344):
					# Send the single packet
					# Send the image data to the FPGA
					#print 'Sending: ' + str(imgName) + ' [Full packet]'
					s.send(bigString)
					amount_received = 0
					amount_expected = len(bigString)
					
					# For receiving data via echo server
					while amount_received < amount_expected:
						data = s.recv(BUFFER_SIZE)
						amount_received += len(data)	
					bigString = ''

			# Close the currently opened image
			img.close()
			
			randVar = '000000000000000'
			#Transmits the data
			s.send(randVar)
			amount_received = 0
			amount_expected = len(randVar)
			
			#For receiving data via echo server
			while amount_received < amount_expected:
				data = s.recv(BUFFER_SIZE)
				amount_received += len(data)	
			# Close the file and move to the next one
			imgNum += 1

			# Sets new frame back to 0 until the FPGA requests another frame
			newFrame = 0

		elif(newFrame == 0):
			s.send(wait)
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


	# Finished sending all available frames to the FPGA


	# The last image has been sent, now we need to wait until its done processing until we can receive the results
	doneSignal = 0
	done = 'Done!'

	while(doneSignal == 0):
		s.send(done)
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

	# FPGA is done processing its last frame and is ready to send the results


	# FPGA will begin sending the results
	var = 'Results'
	doneSignalTwo = 0
	bigString_outputs = ""

	while(doneSignalTwo == 0):
		amount_received2 = 0
		amount_expected2 = 5 # 8 is the smallest output we can receive from the FPGA  #len(var)
		BUFFER_SIZE2 = 100 #len(var)

		# Dummy data value so the last sent data gets echoed back
		s.send(var)

		while amount_received2 < amount_expected2:
			data = s.recv(BUFFER_SIZE2)
			if(data == 'Finished'):
				doneSignalTwo = 1
				amount_received2 += len(data)
				print 'received: ' + str(data)
			else:
				doneSignalTwo = 0
				bigString_outputs += data
				amount_received2 += len(data)
			
				
	print '\nAll results have been received!'
	print 'Writing results to files...'
	# FPGA is finished and all results have been received
	
	# Timestamp to mark how long data transfer took
	endimagesend = time.time()
# ********************* End Transfer of Image Data to the FPGA *********************	






# ********************* Start writing result files *********************	
	# Timestamp to mark how long the output processing takes
	outputstarttime = time.time()

	# Create as many files as subsets
	fileCreate = 1
	while fileCreate <= numSubsets:
		# Create a file to store the results
		fileName = 'DICE_Solutions_' + str(fileCreate) + '.txt'
		file = open(fileName, 'w+')
		
		# Write the header to the output file and given parameters (Parameters will be updated as more functionality is added to the core DICe Verilog-based IPs)
		file.write('*** Currently missing: SIGMA, GAMMA, BETA')
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
		print 'received output: ' + str(output)
		Input_int = int(output, 10)
		Input_str = str(Bits(int = Input_int, length = 32).bin)
		pow = 0
		fraction = 0

		sign_str = Input_str[0]
		exp_str = Input_str[1:9]
		fraction_str = Input_str[9:32]

		sign = int(sign_str, 2)
		#exp = int(exp_str, 2) - 127
		exp = int(exp_str, 2)
		if(exp_str[0] == '1'):
			exp -= 2**len(exp_str)
		exp = exp - 127
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
	
	# Results have finished writing to the output files

	# Close the socket connnection and output file
	print '\nClosing the socket.\n'
	s.close()
	
	# Timestamp to mark how long the output processing takes
	outputendtime = time.time()
# ********************* End writing result files *********************






# ********************* Start output of processing times *********************
	# Timestamp to mark how long the entire process took
	endtime = time.time()


	# Output timing results
	print 'Total param preproccesing execution time: ' + str((paramend - paramstart))
	print 'Total param transfer execution time: ' + str((paramsendend - paramsendstart))
	print 'Total image transfer execution time: ' + str((endimagesend - startimagesend))
	print 'Total image postprocessing (outputs) execution time: ' + str((outputendtime - outputstarttime))
	print 'Total program execution time: ' + str((endtime - starttime))
	print '\nProgram end.\n'
# ********************* End output of processing times *********************
# ********************* END PROGRAM *********************
