import serial,threading,time,sys

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

databuf = {
    "startingchecksum" : 0xAB,
    "accelx" : 0,
    "accely" : 0,
    "accelz" : 0,

    "gyrox" : 0,
    "gyroy" : 0,
    "gyroz" : 0,

    "alt" : 0,
    "vvel" : 0,

    "orientationx" : 0,
    "orientationy" : 0,
    "orientationz" : 0,

    "uptime" : 0,
    "errorflagmp" : 0,
    "errorflagnav" : 0,
    "dataage" : 0,
    "selfuptime" : 0,

    "state" : 8,

    "endingchecksum" : 0xCD
}





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

readdata = str(serialport.read_until('done'))
print(readdata)
readdata.replace('\n'," \t ")
print(readdata)

parseddata = readdata.split("\t")
for i in parseddata:
    i = i.split(",")

print(parseddata)

serialport.close()