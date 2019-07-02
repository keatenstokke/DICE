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

def img_process1(imgName):
	fileType = '.tif'

	# Open the image that needs to be processed
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
		#file.close()
		img.close()


if __name__ == '__main__':
	starting = time.time()

	p1 = Process(target=img_process1, args=('Image_01',))
	p2 = Process(target=img_process1, args=('Image_02',))
	p3 = Process(target=img_process1, args=('Image_03',))
	p4 = Process(target=img_process1, args=('Image_04',))
	
	p1.start()
	p2.start()
	p3.start()
	p4.start()
	
	p1.join()
	p2.join()
	p3.join()
	p4.join()
	
	ending = time.time()
	
	print 'Total execution time: ' + str(ending-starting)
	