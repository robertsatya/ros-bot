import dist
import cv2,cv
import numpy as np
import socket
import ultrasonicthree
import wiringpi2 as wiringpi
import time


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host = "192.168.43.97"
port = 9999

s.bind((host, port))
# queue up to 5 requests
s.listen(5)

wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(25, 1) # pin 22 
wiringpi.digitalWrite(25,0)
wiringpi.pinMode(24, 1) # pin 18
wiringpi.digitalWrite(24,0) # pin 18
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(11, 0) # pin 23 

odroidflag=0

greencounter=0
orangecounter=0
bluecounter=0
wrongcounter=0

flag=0
prevflag=0

lower_green = np.array([36,92,100]) # 1
upper_green = np.array([57,255,255]) # 11
lower_orange = np.array([7,58,128]) # 1
upper_orange = np.array([24,255,255]) # 11
lower_blue = np.array([100,150,0])
upper_blue = np.array([140,255,255])

prevcxb=0
prevcyb=0
cxb=0
cyb=0

prevcx=0
prevcy=0

prevcxg=0
prevcyg=0

cxg=0
cyg=0

cx=0
cy=0

wrongbuck=0
sc,addr = s.accept()

while True:
    oballpickcount=0
    gballpickcount=0
    flag=0
    prevflag=0
    print "start"
    a=sc.recv(1024)
    print a

    if a=="3":
        vc = cv2.VideoCapture(0)
        if vc.isOpened(): # try to get the first frame
            rval, frame = vc.read()
            frame = cv2.flip(frame, 1)
        else:
            rval = False

        while rval:
            #cv2.waitKey(1)
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


            if (abs(cxb-prevcxb)<5 and abs(cyb-prevcyb)<5 and cxb!=0 and cyb!=0):
#                cv2.circle(frame,(int(cxg),int(cyg)),10,(0,0,255),2)
                bluecounter+=1
                wrongbuck=0
                if(bluecounter >= 2):
                    if(int(cxb) >480):
                        print "left to odroid"
                        sc.send("2 1".encode('ascii'))
                        time.sleep(0.2)
                    if(int(cxb) < 160):
                        print "right to odroid"
                        sc.send("3 1".encode('ascii'))
                        time.sleep(0.2)
                    if(int(cxb)<480 and int(cxb)>160):
#                        u=ultrasonicthree.ultrathree()
#                        u=0
                        print "inside bucket"
                        time.sleep(0.1)
                        sc.send("6 0".encode('ascii'))
                        time.sleep(1)
                        print "stop roomba"
                        u=ultrasonicthree.ultrathree()
                        if(u>=10):
                            if(int(cxb) >480):
                                print "left to odroid"
                                sc.send("2 1".encode('ascii'))
                                time.sleep(0.1)
                            if(int(cxb) < 160):
                                print "right to odroid"
                                sc.send("3 1".encode('ascii'))
                                time.sleep(0.1)
                            print u
                            sc.send("0 5".encode('ascii'))
                            time.sleep(1)
#                           sc.send("3 1".encode('ascii'))
                            time.sleep(0.1)
                            u=ultrasonicthree.ultrathree()

                        if(u<10):

                                print u
                                print "arm drop"
                                wiringpi.digitalWrite(24,1)
                                time.sleep(1)
                                wiringpi.digitalWrite(24,0)
                                time.sleep(5)
                                print "send 4 0 success"
                                sc.send("4 0".encode('ascii'))
                                time.sleep(5)
                                vc.release()
                                a="9"
                                print "break"
                                break
                    bluecounter=0
            else:
                wrongbuck+=1
                bluecounter=0
                if(wrongbuck >=15):
                    sc.send("8 0".encode('ascii'))
                    time.sleep(0.1)
                    sc.send("0 1".encode('ascii'))
                    time.sleep(0.1)
                    print "turn"
                                time.sleep(5)
                                print "send 4 0 success"
                                sc.send("4 0".encode('ascii'))
                                time.sleep(5)
                                vc.release()
                                a="9"
                                print "break"
                                break
                    bluecounter=0
            else:
                wrongbuck+=1
                bluecounter=0
                if(wrongbuck >=15):
                    sc.send("8 0".encode('ascii'))
                    time.sleep(0.1)
                    sc.send("0 1".encode('ascii'))
                    time.sleep(0.1)
                    print "turn"
            prevcxb=cxb
            prevcyb=cyb
#       a=sc.recv(1024)
        #TODO: send to odroid
    elif a=="1":
        #cv2.namedWindow("preview")
        vc = cv2.VideoCapture(0)
#       time.sleep(5)
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

            if (abs(cxg-prevcxg)<5 and abs(cyg-prevcyg)<5 and cxg!=0 and cyg!=0):
#                cv2.circle(frame,(int(cxg),int(cyg)),10,(0,0,255),2)
                orangecounter+=1
                greencounter=0
                wrongcounter=0
                if(orangecounter >= 2):
                    flag=1
#                   print str(cxg)+" "+str(cyg)
                    if(int(cxg)>307 and int(cxg) <380 and int(cyg) > 358 and int(cyg) <395):
                        if(oballpickcount==0):
                            wiringpi.digitalWrite(25,1)
                            print " orange get dist from arduino"+str(cxg)+" "+str(cyg)
                            time.sleep(1)
                            wiringpi.digitalWrite(25,0)
                            time.sleep(30)
                            if(wiringpi.digitalRead(11)==0):
                                sc.send("4 0".encode('ascii'))
                                sc.send("1 8".encode('ascii'))
                                vc.release()
                                break
#                           sc.send("1 8".encode('ascii'))
                            oballpickcount+=1
                    if(int(cxg) <307):
                        print "right to odroid"
                        sc.send("3 1".encode('ascii'))
                    if(int(cxg) > 380):
                        print "left to odroid"
                        sc.send("2 1".encode('ascii'))
                    if(int(cyg) > 395):
                        print "backward to odroid"
                        sc.send("1 1".encode('ascii'))
                    if(int(cyg) < 358):
                        print "forward to odroid"
                        sc.send("0 1".encode('ascii'))
#            print ("arm orange"+str(int(cxg))+" "+str(int(cyg)))
                    orangecounter=0

            elif (abs(cx-prevcx)<5 and abs(cy-prevcy)<5 and cx!=0 and cy!=0):
#            print ("arm green"+str(int(cx))+" "+str(int(cy)))            
#                cv2.circle(frame,(int(cx),int(cy)),10,(0,0,255),2)
                greencounter+=1
                orangecounter=0
                wrongcounter=0
                if(greencounter >= 2):
                    flag=2
#                   print(str(cx)+" "+str(cy))
                    if(int(cx)>325 and int(cx) <435 and int(cy) > 383 and int(cy) <457):
                       if(gballpickcount==0):
                            print " green get dist from arduino"
                            wiringpi.digitalWrite(25,1)
                            time.sleep(1)
                            wiringpi.digitalWrite(25,0)
                            time.sleep(30)
                            if(wiringpi.digitalRead(11)==0):
                                sc.send("4 0".encode('ascii'))
                                sc.send("1 8".encode('ascii'))
                                print "Successfully picked up"
                                vc.release()
                                break
#                           sc.send("1 8".encode('ascii'))
                            gballpickcount+=1
            #           get dist from arduino
                    if(int(cx) <325):
                        print "right to odroid"
                        sc.send("3 1".encode('ascii'))
                    if(int(cx) > 435):
                        print "left to odroid"
                        sc.send("2 1".encode('ascii'))
                    if(int(cy) > 457):
                        print "backward to odroid"
                        sc.send("1 1".encode('ascii'))
                    if(int(cy) < 383):
                        print "forward to odroid"
                        sc.send("0 1".encode('ascii'))
                    greencounter=0
            else:
#       cv2.imshow("preview",frame)
                wrongcounter+=1
                greencounter=0
                orangecounter=0
                if(wrongcounter >=10):
                    flag=3
#                    print " no ball odroid"
#                    sc.send("5 0".encode('ascii'))
#            print "wrong"

#            if((prevflag==1 or prevflag==2) and flag==3):
#                sc.send("4 0".encode('ascii'))
#               print "send 4 0"
#               vc.release()
#               break
#               a=sc.recv(1024)
#               print a
            if((prevflag==3 and flag==3)):
                sc.send("5 0".encode('ascii'))
                print "send 5 0"
                vc.release()
                break
#           if((prevflag==flag) and (flag==2 or flag==1)):
#               print "lol"
#               vc.release()
#               break
#                a=sc.recv(1024)
#               print a
            prevflag=flag
            prevcx=cx
            prevcy=cy
            prevcxg=cxg
            prevcyg=cyg

#            break
#            sc.close()
#    cv2.imshow("preview", frame)
#    cv2.destroyAllWindows()
#    else:
#    print("Unknown input")

    else:
#       sc.send("5 0".encode('ascii'))
        print "5 0 0 no input"
sc.close()
