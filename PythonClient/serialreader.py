###################################################################################################################
# File: serialreader.py
# Course: 17640
# Project: IoT Order Fulfillment Center
# Copyright: Copyright (c) 2018 Carnegie Mellon University (ajl)
# Versions:
#	1.0 April 2018 - Initial write (ajl).
#
# Description: This class serves as an example for how to write an application that can read serial data from
# an external device. The intent is to illustrate how to read data from an Arduino software serial port
# This example could be used as a basis for writing an application to control and get status from the 
# fultillment center robots.
#
# Parameters: Port or device file
#
# Internal Methods:
#  None
#
# External Dependencies:
#   - python 2.7
#	- time
#	- serial ## pyserial-3.2.1 library
###################################################################################################################

import sys
import time
import serial 

# Make sure the device/comm port was supplied on the command line. To get a list of valie devices please see
# serialports.py (lists the available ports).

if len(sys.argv) < 2:
    print 'You need to supply the comm port/device on the command line.'
    print 'python serialreader <comm port or device path>'
    exit()
else:
	print
	print '____________________________________________________________'		
	print 'Reading from port ' + sys.argv[1]
	print 'To stop, press control-C'
	print '____________________________________________________________'
	print

# Here we define the serial port. This reflects the settings for the software serial port on the arduino.
# Please refer to the companion arduino code: SoftSerialWrite.ino. Note that no errors are trapped here in the 
# name of brevity. If the comm port/device doesn't exist, it will crash here. For your production code you should
# probably add error handling here (try-catches).

ser = serial.Serial (
    port=sys.argv[1],
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

# Wait a couple of seconds for things to settle (typical in the embedded world ;-)

time.sleep(2)

# This is a pretty simple do-forever loop. If there is anything to read, we read it and print it.

while True: 
	if ser.inWaiting() > 0:				# Check to see if anything is in the buffer					
		line =ser.readline()			# Read the buffer
		print('read::' + line +'\n')



