f = open("offline.txt",'r')
f_data = f.read().replace("\r","").replace("\n","").replace(" ","")
f.close()

bytes_jpeg = bytearray.fromhex(f_data)
print("Writing SSDV ")

f = open("downlink_1.ssdv",'wb')

f.write(bytes_jpeg)

f.close()
