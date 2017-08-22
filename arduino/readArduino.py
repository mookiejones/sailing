
import serial
import time
baudRate = 9600
serPort = "/dev/ttyACM0"

ser = serial.Serial(serPort, baudRate)
ser.flushInput()
ser.flushOutput()
print "Serial port " + serPort + " opened  Baudrate " + str(baudRate)

startMarker = 60
endMarker = 62
while True:
  data_raw=ser.readline();
  print data_raw

def recvFromArduino():
  global startMarker, endMarker
  
  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many
  
  # wait for the start character
  while  ord(x) != startMarker: 
    x = ser.readline()
    print x
  
  # save data until the end marker is found
  while ord(x) != endMarker:
    if ord(x) != startMarker:
      ck = ck + x 
      byteCount += 1
    x = ser.read()
  
  return(ck)


#============================

def waitForArduino():

   # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
   # it also ensures that any bytes left over from a previous message are discarded
   
    global startMarker, endMarker
    
    msg = ""
    while msg.find("Arduino is ready") == -1:

      while ser.inWaiting() == 0:
        pass
        
      msg = recvFromArduino()

      print msg
      print
      
#======================================


n=0
while 1>0:
  print 'Reading'
  waitForArduino()
  dataRecvd=recvFromArduino()
  print "Reply Received "+dataRecvd
  n +=1
  time.sleep(5)
print "quitting"