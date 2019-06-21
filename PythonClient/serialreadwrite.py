import sys
import time
import serial 

ser = serial.Serial(
    port='/dev/tty.usbserial',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
print('\nopening port: ' + ser.name)

data=str()
exit='exit\n'

while 1 :
    # get keyboard input
    # print('Enter string (type exit to quit) >> ')
    # data = sys.stdin.readline()
    data = raw_input('Enter message>> ')

    if data == exit:
        ser.close()
        sys.exit(0)
        print('\n')
    else:
        # Send the character to the device. Note that I append a '+' character to the string. 
        # This is a simple protocol to let the device know that it is the end of the message.
 
        data += "+"
        b = str.encode(data)
        ser.write(b) 
        ser.flush()
 
        # let's wait one second before reading output - give device time to answer
        time.sleep(1)
 
        # Now we read the confirmation message
        if ser.inWaiting() > 0:
        	line = ser.readline()
        	print(line)