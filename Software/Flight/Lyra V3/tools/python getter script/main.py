import serial,sys,time

import serial.tools.list_ports as port_list

portname = "COM3"

try:
    serialport = serial.Serial()
    serialport.baudrate = 115200
    serialport.bytesize = 8
    serialport.parity = "N"
    serialport.stopbits = 1
    serialport.timeout = 10
except:
     print("serial fail")


print("open ports: ")

ports = list(port_list.comports())
for p in ports:
    print (p)

portname = input("type name of port from list above, ex COM3\n")

print("opening: "+portname)

serialport.port = portname
try:
    serialport.open()

except:
    print("unable to open port")
    sys.exit()

print("reading data")
serialport.write(b'D')
time.sleep(1)
readdata = ""
while serialport.in_waiting > 0:
    readdata = readdata + str(serialport.read_until('done'))
    readdata = readdata.replace('\\n','\n')

print(readdata)
serialport.close()



file = open('logfile '+str(time.strftime('%m,%d,%Y, %H,%M,%S'))+'.csv','w+')

file.write(readdata)