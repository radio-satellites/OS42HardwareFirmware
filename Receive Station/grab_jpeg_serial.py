import serial

ser = serial.Serial('COM10', 115200)

counter = 0

while True:
    try:
        line = ser.readline().decode()
    except:
        print("Error decoding Serial data.")

    if "FF D8" in line:
    
        counter = counter + 1
        
        line = line.replace("\n","").replace(" ","").replace("\r","").lower()

        end_location = line.find("ffd9")

        line = line[:end_location]

        try:

            bytes_jpeg = bytearray.fromhex(line)
            print("Writing JPEG "+str(counter))

            f = open("downlink_"+str(counter)+".jpg",'wb')

            f.write(bytes_jpeg)

            f.close()
        except:
            pass

        
    else:
        print("Non-JPEG line")
    
