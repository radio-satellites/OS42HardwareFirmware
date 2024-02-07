import serial

ser = serial.Serial('COM10', 115200)

counter = 0

while True:
    try:
        line = ser.readline().decode()
    except:
        print("Error decoding Serial data.")
        break

    if len(line) >= 400:
        
    
        counter = counter + 1
        
        line = line.replace("\n","").replace(" ","").replace("\r","").lower()

        print("SSDV!")

        #end_location = line.find("ffd9")

        line = line[:len(line)-22]

        """


        bytes_jpeg = bytearray.fromhex(line)
        print("Writing SSDV "+str(counter))

        f = open("downlink_"+str(counter)+".ssdv",'wb')

        f.write(bytes_jpeg)

        f.close()

        """
        print(line)
    

        
    else:
        print("Non-SSDV line")
    
