import serial
import time
ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/ttyUSB0'
ser.timeout = 0.1
ser.open()
print(ser.in_waiting)
ser.reset_input_buffer()
while True:
    if(ser.in_waiting):
        lmao = ser.readline()
        print(lmao.find(',M,'))
        if(lmao.find(',M,') != -1):
            print(lmao[16]+lmao[17]+lmao[18]+lmao[19]+lmao[20]+lmao[21])
