import os
from reedsolo import RSCodec, ReedSolomonError
import random

image = open("decode.jpg",'wb')
f = open("transmission_FSK_PHY.bin",'rb')

size = os.path.getsize('transmission_FSK_PHY.bin')

frames = int(size/64)

rsc = RSCodec(9)  # 9 ecc symbols

for i in range(frames):
    frame = bytearray(f.read(64))
    erase = random.randint(0,7)

    if erase == 2:
        for i in range(len(frame)):
            change = random.randint(0,64)
            if change == 10:
                newbyte = random.randint(0,255)
                frame[i] = newbyte

    framedata = frame[4:]

    try:

        payload = rsc.decode(framedata)[0]
        JPEG = payload[1:]

        if payload[0] == 100:
            #print("GPS FRAME")
            try:
                print(payload[1:10].decode())
            except:
                pass
        else:
            image.write(JPEG)
    except ReedSolomonError:
        print("RS BAD!")

    

image.close()
f.close()


    

