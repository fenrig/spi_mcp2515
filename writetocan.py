#/usr/bin/python

import time

f = open('/dev/spi_mcp2515', 'w')
s = bytearray(11)
s[0] = 01
s[1] = 64
# s[2] = int(input("Enter length [1 - 8]: "))
s[2] = 8
s[3] = b'1'
s[4] = b'2'
s[5] = b'3'
s[6] = b'4'
s[7] = b'5'
s[8] = b'6'
s[9] = b'7'
s[10] = b'8'
f.write(s)
f.close()

# f = open('/dev/spi_mcp2515', 'r')

# time.sleep(1)

#data = f.read(11)
#print(data)


#f.close()
