from threading import Thread 
import cv2
import os
import imutils
import numpy as np
import time
#import serial

## Global variables

# Define HSV range as globals
h_min = 150
h_max = 179
s_min = 100
s_max = 255
v_min = 32
v_max = 255
lower = np.array([h_min,s_min,v_min])
upper = np.array([h_max,s_max,v_max])

times=[]

# test to see if globals help speed up pi
global hsv, mask

hsv = np.zeros((720,1280,3))
mask = np.zeros((720,1280))

## Function definitions

def myVision(frame):
    t0=time.time()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert BGR to HSV
    mask = cv2.inRange(hsv, lower, upper) # get parts in HSV range
    mask = cv2.erode(mask, None, iterations=2) # erode mask
    mask = cv2.dilate(mask, None, iterations=2) # dilate mask
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) #findcountours
    cnts = imutils.grab_contours(cnts) # grab contours

    x=999
    y=999
    r=999

    if len(cnts) > 0:
    	c = max(cnts, key=cv2.contourArea) # find contour with largest area
    	((x, y), r) = cv2.minEnclosingCircle(c) # find min enclosing circle

    print(f'(x,y)=({x},{y})')
    times.append(time.time()-t0)

    return x,y,r

def read_from_port(ser):
     while True:
         reading = ser.readline(1).decode()
         if reading == 's':
             print("broken")
             flag = False
             break

def sendCoords(x0,y0,port):
    pass
    chords=np.array([x0,y0])
    
    coords = np.trunc(chords*10)/10
    if(coords[0]>= 0 and coords[0] < 10):
        x = str(coords[0])+"00"
    elif(coords[0]>= 10 and coords[0] < 100):
        x = str(coords[0])+ "0"
    else:
        x = str(coords[0])
    if(coords[1]>= 0 and coords[1] < 10):
        y = str(coords[1])+"00"
    elif(coords[1]>= 10 and coords[1] < 100):
        y = str(coords[1])+ "0"
    else:
        y = str(coords[1])

    port.write(x.encode('ascii'))
    port.write(y.encode('ascii'))

## VideoGet class, responsible for input and sending frames to be processed

class VideoGet:
    def __init__(self, src, port): # constructor, gets called on instantiation
        self.stream = cv2.VideoCapture(src) # start stream from src, defaults to 0
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False # initialize stopped attribute
        self.port = port

        self.x=0
        self.y=0
        self.r=0

    def start(self):    
        Thread(target=self.get, args=()).start() # create thread that targets get method
        return self # return current VideoGet object

    def get(self):
        while not self.stopped: # loop as long as not stopped
            if not self.grabbed:
                self.stop() # stop if frame read unsuccessful
            else:
                (self.grabbed, self.frame) = self.stream.read() # read frame
                (self.x,self.y,self.r) = myVision(self.frame)
                sendCoords(self.x,self.y,self.port)

    def stop(self):
        self.stopped = True # stop if stop() is called

## Main function

def threadedCV(source,port):
    start_time = time.time()
    video_getter = VideoGet(source,port)
    video_getter.start()
    listener = Thread(target = read_from_port,args=(port,queue))
    listener.start()
    ret = True
    while ret == True:
        ret = listener.is_alive()
        cv2.circle(video_getter.frame, (int(video_getter.x), int(video_getter.y)), int(video_getter.r),(0, 255, 255), 2)
        if video_getter.stopped or ret == False:
            video_getter.stop()
            break

    stop_time = time.time()-start_time
    print("stoptime: " + str(stop_time))
    print('shit stopped')


## Main

os.system("sudo chmod 666 /dev/ttyAMA0")
port = serial.Serial(port="/dev/ttyAMA0",baudrate=9600,timeout = 1)
threadedCV(0,port)
print(f'Average myVision time: {np.mean(times)}')
cv2.destroyAllWindows()
port.close()