#####-----BUILD LORA EXAMPLE------#####
print("BUILDING LORA EXAMPLE...")
import os

image = open("testimage.jpg",'rb')
f = open("transmission_LoRa_COMM.bin",'wb')

mode = 1

mode_bytearray = bytearray([mode])

size = os.path.getsize('testimage.jpg')

bytesread = 0

read = True
while read:
    imagery = image.read(255)
    if len(imagery) < 255:
        imagery = imagery + bytearray(255-len(imagery))
        
    f.write(mode_bytearray+imagery)
    bytesread = bytesread+255
    if bytesread >= int(size):
        read = False
    

image.close()
f.close()

####BUILD RAW FSK EXAMPLE####
print("BUILDING FSK PHY EXAMPLE...")
import os
from reedsolo import RSCodec, ReedSolomonError
import random

image = open("testimage.jpg",'rb')
gps = open("GPS_STRINGS.txt",'r')
f = open("transmission_FSK_PHY.bin",'wb')

mode = 1

mode_bytearray = bytearray([mode])

sync = bytearray([127,0,200,0])
spacer = bytearray([0,0])

size = os.path.getsize('testimage.jpg')

bytesread = 0

read = True
rsc = RSCodec(9)
interleave_counter = 0
while read:
    interleave_counter = interleave_counter+ 1
    if interleave_counter % 300 == 0:
        interleave = 1
    else:
        interleave = 0
    if interleave == 1:
        mode = 100

        mode_bytearray = bytearray([mode])
        GPS_data = gps.readline()
        
        ##SEND GPS FRAME

        #print("Interleave GPS frame!")

        RS_data = rsc.encode(mode_bytearray+GPS_data.encode())

        zeros = bytearray(64-(len(RS_data)+4))
        f.write(sync+RS_data+zeros)
        
        
    else:
        mode = 1

        mode_bytearray = bytearray([mode])
        imagery = image.read(50)
        #RSC#
        RS_data = rsc.encode(mode_bytearray+imagery)
        data = RS_data[0:50]
        RS = RS_data[50:]
        """dd
        if len(sync+data+RS) < 64:
            data = data + bytearray(64-len(data))
        """
            
        f.write(sync+data+RS)
        bytesread = bytesread+50
        if bytesread >= int(size):
            read = False
    

image.close()
f.close()
