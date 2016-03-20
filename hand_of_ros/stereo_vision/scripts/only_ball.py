#!/usr/bin/python

import cv2,cv
import numpy as np
import socket


import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host = "192.168.43.34"
port = 3335

s.bind((host, port))
# queue up to 5 requests
s.listen(5)



odroidflag=0

greencounter=0
orangecounter=0
wrongcounter=0

circlecount=0


lower_green = np.array([37,108,44]) # 1
upper_green = np.array([54,215,255]) # 11

lower_orange = np.array([4,113,51]) # 1
upper_orange = np.array([18,203,250]) # 11


prevcx=0
prevcy=0

prevcxg=0
prevcyg=0

cxg=0
cyg=0

cx=0
cy=0


sc,addr = s.accept()

while True:

    print "start"

    a=sc.recv(1024)
    print a

    if a=="ball detect":
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
            mask = cv2.inRange(hsv, lower_green, upper_green)
            gmask=cv2.inRange(hsv, lower_orange, upper_orange)
            mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3) ))
            gmask = cv2.erode(gmask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
            gmask = cv2.dilate(gmask, cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3) ))
            moments=cv2.moments(mask, False)
            gmoments=cv2.moments(gmask,False)

            if(moments['m00'] >0):
                cx = moments['m10']/moments['m00']
                cy = moments['m01']/moments['m00']
            else:
                cx=0
                cy=0

            if(gmoments['m00'] >0):
                cxg = gmoments['m10']/gmoments['m00']
                cyg = gmoments['m01']/gmoments['m00']
            else:
                cxg=0
                cyg=0

            if (abs(cxg-prevcxg)<25 and abs(cyg-prevcyg)<25 and cxg!=0 and cyg!=0):
                cv2.circle(frame,(int(cxg),int(cyg)),10,(0,0,255),2)
                orangecounter+=1
                greencounter=0
                wrongcounter=0
                if(orangecounter >= 2):
#		   print str(cxg)+" "+str(cyg)
                    if(cxg >= 480):
                        sc.send("forward".encode('ascii'))
                    if(cxg < 480):
                        sc.send("right".encode('ascii'))
                    orangecounter=0

            elif (abs(cx-prevcx)<25 and abs(cy-prevcy)<25 and cx!=0 and cy!=0):
#            print ("arm green"+str(int(cx))+" "+str(int(cy)))
                cv2.circle(frame,(int(cx),int(cy)),10,(0,0,255),2)
                greencounter+=1
                orangecounter=0
                wrongcounter=0
                if(greencounter >= 2):
#		   print(str(cx)+" "+str(cy))
                    if(cx >= 480):
                        sc.send("forward".encode('ascii'))
                    if(cx < 480):
                        sc.send("right".encode('ascii'))
                    greencounter=0
            else:
#       cv2.imshow("preview",frame)
                wrongcounter+=1
                greencounter=0
                orangecounter=0
                if(wrongcounter >=10):
                    circlecount+=10
                    sc.send("right 10".encode('ascii'))
                    if(circlecount==360):
                        sc.send("forward".encode('ascii'))
                        circlecount=0

            prevcx=cx
            prevcy=cy
            prevcxg=cxg
            prevcyg=cyg

#            vc.release()
#            break
            cv2.imshow("preview", frame)

    else:
        print "third cam mode"
#            break
#            sc.close()
#    cv2.imshow("preview", frame)
#    cv2.destroyAllWindows()
#    else:

sc.close()
