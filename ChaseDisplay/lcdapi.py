import serial
import time

def init(port):
    global ser
    ser = serial.Serial(port,115200)
    time.sleep(0.05) #Wait for things to settle
    print("Using port "+str(ser.portstr))
    
def print_lcd(string):
    ser.write(string.encode())
    
def switch_line(line):
    if line == 1:
        ser.write(b">")
    elif line == 0:
        ser.write(b"<")
def clear_screen():
    ser.write(b"%")

def deinit():
    ser.close()         

