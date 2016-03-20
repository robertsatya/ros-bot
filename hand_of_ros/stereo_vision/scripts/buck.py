#!/usr/bin/python2
import cv2,cv
import numpy as np
import socket


import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host = "192.168.43.34"
port = 9995

s.bind((host, port))
# queue up to 5 requests
s.listen(5)


wrongcounter=0
bluecounter=0

circlecount=0

lower_blue=np.array([78,47,56])
upper_blue=np.array([115,147,135])

prevcxb=0
prevcyb=0

cxb=0
cyb=0

sc,addr = s.accept()

while True:

    print "start"

    a=sc.recv(1024)
    print a

    if a=="ball drop":
        cv2.namedWindow("preview")
        vc = cv2.VideoCapture(0)
#	time.sleep(5)
        if vc.isOpened(): # try to get the first frame
            rval, frame = vc.read()
            frame = cv2.flip(frame, 1)
        else:
            rval = False

        while rval:
            cv2.waitKey(1)
            rval, frame = vc.read()
            frame = cv2.flip(frame, 1)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            bmask = cv2.inRange(hsv, lower_blue, upper_blue)
            bmask = cv2.erode(bmask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
            bmask = cv2.dilate(bmask, cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3) ))
            bmoments=cv2.moments(bmask, False)

            if(bmoments['m00'] >0):
                cxb = bmoments['m10']/bmoments['m00']
                cyb = bmoments['m01']/bmoments['m00']
            else:
                cxb=0
                cyb=0

            if (abs(cxb-prevcxb)<25 and abs(cyb-prevcyb)<25 and cxb!=0 and cyb!=0):
                cv2.circle(frame,(int(cxb),int(cyb)),10,(0,0,255),2)
                bluecounter+=1
                wrongcounter=0
                if(bluecounter >= 2):
#		   print str(cxg)+" "+str(cyg)
                    if(cxb >= 480):
                        sc.send("left".encode('ascii'))
                    if(cxb <= 320):
                        sc.send("right".encode('ascii'))
                    if(cxb > 320 and cxb < 480):
                        sc.send("forward".encode('ascii'))
                    bluecounter=0

            else:
#       cv2.imshow("preview",frame)
                wrongcounter+=1
                bluecounter=0
                if(wrongcounter >=10):
                    circlecount+=10
                    sc.send("right 10".encode('ascii'))
                    if(circlecount==360):
                        sc.send("forward".encode('ascii'))
                        circlecount=0

            prevcxb=cxb
            prevcyb=cyb

            cv2.imshow("preview", frame)
#            vc.release()
#            break
    else:
        print "third cam mode"
#            break
#            sc.close()
#    cv2.imshow("preview", frame)
#    cv2.destroyAllWindows()
#    else:

sc.close()
