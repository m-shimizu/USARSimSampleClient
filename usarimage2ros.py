#!/usr/bin/env python
import rospy
#import roslib; roslib.load_manifest('hinomiyagura')
import socket
import math
from sensor_msgs.msg import Image
import Image as pillow_image
from StringIO import StringIO
import time

#socket info
#host = '192.168.11.4'
host = '127.0.0.1'
port = 5003

#UDK's camera info(height,width)
UDK_camera=(240,320)
#box(left, upper, right, lower)
box_A = (0, 0, UDK_camera[1],UDK_camera[0]) 
box_B = (UDK_camera[1],0,UDK_camera[1]*2,UDK_camera[0])
box_C = (UDK_camera[1]*2,0,UDK_camera[1]*3,UDK_camera[0])
box_D = (UDK_camera[1]*3,0,UDK_camera[1]*4,UDK_camera[0])
#box_C = (0,UDK_camera[0],UDK_camera[1],UDK_camera[0]*2)
#box_D = (UDK_camera[1],UDK_camera[0],UDK_camera[1]*2,UDK_camera[0]*2)

clientsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientsock.connect((host,port))
c_msg = 'OK\r\n'
clientsock.sendall(c_msg)

#set image info 
img_A=Image()
img_A.encoding='bgr8'
img_A.is_bigendian =0
img_A.step =1
img_A.height = UDK_camera[0]
img_A.width = UDK_camera[1]

img_B=Image()
img_B.encoding='bgr8'
img_B.is_bigendian =0
img_B.step =1
img_B.height = UDK_camera[0]
img_B.width = UDK_camera[1]

img_C=Image()
img_C.encoding='bgr8'
img_C.is_bigendian =0
img_C.step =1
img_C.height = UDK_camera[0]
img_C.width = UDK_camera[1]

img_D=Image()
img_D.encoding='bgr8'
img_D.is_bigendian =0
img_D.step =1
img_D.height = UDK_camera[0]
img_D.width = UDK_camera[1]

img_E=Image()
img_E.encoding='bgr8'
img_E.is_bigendian =0
img_E.step =1
img_E.height = UDK_camera[0]
img_E.width = UDK_camera[1]


#publish topic
pub_A = rospy.Publisher('UDKImage_A',Image)
pub_B = rospy.Publisher('UDKImage_B',Image)
pub_C = rospy.Publisher('UDKImage_C',Image)
pub_D = rospy.Publisher('UDKImage_D',Image)
#pub_E = rospy.Publisher('UDKImage_E',Image)
#start node
rospy.init_node('UDKView')
rcvmsg=''
smallPict=[]
while True:
    rcvmsg = clientsock.recv(65536) 
    try:
        print len(rcvmsg)
        if (len(rcvmsg) == 0):
          break
        if ((len(rcvmsg)>5) and (len(rcvmsg) == (ord(rcvmsg[1])*math.pow(256,3)) + (ord(rcvmsg[2])*math.pow(256,2)) + (ord(rcvmsg[3])*256) + ord(rcvmsg[4])+5)):
            if(ord(rcvmsg[0]) !=0): #case by jpeg data
                im = pillow_image.open(StringIO(rcvmsg[5:]))
                im_a=im.crop(box_A)
                im_b=im.crop(box_B)
                im_c=im.crop(box_C)
                im_d=im.crop(box_D)
                x,y=0,0
                smallPict_A=[]
                smallPict_B=[]
                smallPict_C=[]
                smallPict_D=[]
#                smallPict_E=[]
                while y<UDK_camera[0]:
                    x=0
                    while x< UDK_camera[1]:
                        smallPict_A.append(im_a.getpixel((x,y))[2])
                        smallPict_A.append(im_a.getpixel((x,y))[1])
                        smallPict_A.append(im_a.getpixel((x,y))[0])
                        smallPict_B.append(im_b.getpixel((x,y))[2])
                        smallPict_B.append(im_b.getpixel((x,y))[1])
                        smallPict_B.append(im_b.getpixel((x,y))[0])
                        smallPict_C.append(im_c.getpixel((x,y))[2])
                        smallPict_C.append(im_c.getpixel((x,y))[1])
                        smallPict_C.append(im_c.getpixel((x,y))[0])
                        smallPict_D.append(im_d.getpixel((x,y))[2])
                        smallPict_D.append(im_d.getpixel((x,y))[1])
                        smallPict_D.append(im_d.getpixel((x,y))[0])
                        x+=1
                    y+=1
#                x,y=0,0
#                bb = im.getbbox()
#                img_E.height = bb[3] 
#                img_E.width = bb[2]
#                while y<bb[3]:
#                    x=0
#                    while x<bb[2]:
#                        smallPict_E.append(im.getpixel((x,y))[2])
#                        smallPict_E.append(im.getpixel((x,y))[1])
#                        smallPict_E.append(im.getpixel((x,y))[0])
#                        x+=1
#                    y+=1
#                x,y=0,0
                img_A.data = tuple(smallPict_A)
                img_B.data = tuple(smallPict_B)
                img_C.data = tuple(smallPict_C)
                img_D.data = tuple(smallPict_D)
#                img_E.data = tuple(smallPict_E)
                #print len(img_A.data)
                #jpeg to row
#            else:   #row data
#                i,j=0,0
#                smallPict=[]
#                while i<240:
#                    while j < 960:
#                        smallPict.append(ord(rcvmsg[9 + j + (i * (ord(rcvmsg[5])*256 + ord(rcvmsg[6]))*3)]))
#                        j = j+1
#                    i=i+1
#                    j=0 
#                img_A.data = tuple(smallPict)
#                i,j,smallPict=0,960,[]
#                while i<240:
#                    while j < 1920:
#                        smallPict.append(ord(rcvmsg[9 + j + (i * (ord(rcvmsg[5])*256 + ord(rcvmsg[6]))*3 )]))
#                        j = j+1
#                    i=i+1
#                    j=960
#                img_B.data = tuple(smallPict)
#                j,smallPict=0,[]
#                while i<480:
#                    while j < 960:
#                        smallPict.append(ord(rcvmsg[9 + j + (i * (ord(rcvmsg[5])*256 + ord(rcvmsg[6]))*3 )]))
#                        j = j+1
#                    i=i+1
#                    j=0 
#                img_C.data = tuple(smallPict)
#                i,j,smallPict=240,960,[]
#                while i<480:
#                    while j < 1920:
#                        smallPict.append(ord(rcvmsg[9 + j + (i * (ord(rcvmsg[5])*256 + ord(rcvmsg[6]))*3 )]))
#                        j = j+1
#                    i=i+1
#                    j=960 
#                img_D.data = tuple(smallPict)
#                print 'raw'
#                img_E.data = tuple(rcvmsg[5:])

            pub_A.publish(img_A)
            pub_B.publish(img_B)
            pub_C.publish(img_C)
            pub_D.publish(img_D)
#            pub_E.publish(img_E)
            rcvmsg=''
            clientsock.sendall(c_msg)
            print 'looping'
    except:
        print 'error'
        break
clientsock.close()
