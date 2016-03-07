#!/usr/bin/python
import socket                                         
from time import sleep
import serial

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time
import atexit
z=90
serversocket = socket.socket(
	        socket.AF_INET, socket.SOCK_STREAM) 

host= "192.168.0.17"                           

port = 9997                                           


# bind to the port
serversocket.bind((host, port))                                  

# queue up to 5 requests
serversocket.listen(5)                                           


# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

################################# DC motor test!
myMotor = mh.getMotor(3)

# set the speed to start, from 0 (off) to 255 (max speed)
myMotor.setSpeed(150)
myMotor.run(Adafruit_MotorHAT.FORWARD);
# turn on motor
myMotor.run(Adafruit_MotorHAT.RELEASE);


while (True):
        clientsocket,addr = serversocket.accept() 
	print("Got a connection from %s" % str(addr))
	a=clientsocket.recv(1024)
	print( a)
	
	if "up" in a:
		temp=a.split(" ")
		t=int(temp[1])/22.0
		z=z-int(temp[1])
		print "Arm at "+ str(z)
		myMotor.run(Adafruit_MotorHAT.FORWARD)
		clientsocket.send( str(z))
#	print "\tSpeed up..."
#	for i in range(255):
		myMotor.setSpeed(128)
		time.sleep(t)
		a="stop"

	if "down" in a:
		temp=a.split(" ")
                t=int(temp[1])/22.0
		z=z+int(temp[1])
		clientsocket.send(str(z))
#	print "\tSlow down..."
#	for i in reversed(range(255)):
#	myMotor.setSpeed(128)
#	time.sleep(0.5)

		print "Arm at "+str(z)
		myMotor.run(Adafruit_MotorHAT.BACKWARD)

#	print "\tSpeed up..."
#	for i in range(255):
		myMotor.setSpeed(128)
		time.sleep(t)
		a="stop"

#	print "\tSlow down..."
#	for i in reversed(range(255)):
#		myMotor.setSpeed(i)
#		time.sleep(0.01)
	if a=="stop":
#		print "Release"
		myMotor.run(Adafruit_MotorHAT.RELEASE)
#		time.sleep(1.0)


clientsocket.close()
"Dhanesh Pradhan" 
