#!/usr/bin/env python
# coding: utf-8

import PySimpleGUI as sg
import mido as md
import threading
import time
import serial
import numpy as np

#    ***Function Declarations***

def xyToSpherical(x0,y0):
    # shift origin
    x = -320 + x0
    y = 240 + y0

    # compute theta
    if y >= 0:
        theta = 180 - 180*np.arctan2(y,x)/np.pi
    else:
        theta = -180 - 180*np.arctan2(y,x)/np.pi

    # compute phi
    r = np.sqrt(x**2 + y**2)
    phi = 90-r/2
    return theta, phi

def sphericalToPitch(theta,phi):
    p_az = -int((theta/180)*8192) # negative pitch gives positive angle
    p_el = int((phi/90)*8192)
    return p_az, p_el

def read_from_port_and_send2midi(ser,flag):
    i = 0
    gotCleanData = False
    while True:
        st = (ser.read(5)).decode('ascii')
        if st != "":
            if st == "sssss" or flag==False:
                print("shit")
                break
            if i%2 == 0:
                x = float(st)
            elif i%2 == 1:
                y = float(st)
                if x != 999 and y != 999:
                    gotCleanData = True
                if x == 999 and y == 999 and gotCleanData == True: # if getting junk data, set to prev
                    x = prev_x
                    y = prev_y
                else: # if not getting junk data, save prev
                    prev_x = x
                    prev_y = y
                if gotCleanData == True: # if there's non-999 data to send
                    theta,phi = xyToSpherical(x,y)
                    p_az,p_el = sphericalToPitch(theta,phi)
                    az_msg = md.Message('pitchwheel', channel=0, pitch=p_az)
                    el_msg = md.Message('pitchwheel', channel=1, pitch=p_el)
                    outport.send(az_msg)
                    outport.send(el_msg)
            print("message sent!")
            i = i+1

def write_to_port(ser):
    stop = "s"
    ser.write(stop.encode('ascii'))

def link_az(outport):
    for i in range(100):
            az_msg=md.Message('pitchwheel',channel=0,pitch=-80*i)
            outport.send(az_msg)
            time.sleep(0.05)

def link_el(outport):
    for i in range(100):
            el_msg=md.Message('pitchwheel',channel=1,pitch=50*i)
            outport.send(el_msg)
            time.sleep(0.05)

#    ***Creating GUI Window***

layout = [           
            [sg.Text('Start Reading From Cameras',justification='left',size=(100,1))],
            [sg.Button('Start', size=(20, 2))], 

            [sg.Text('Stop Reading From Cameras',justification='left',size=(100,1))],
            [sg.Button('Stop', size=(20, 2))], 

            [sg.Text('Link Azimuth Channel to REAPER',justification='left',size=(100,1))],
            [sg.Button('Link Azimuth', size=(20, 2))], 

            [sg.Text('Link Elevation Channel to REAPER',justification='left',size=(100,1))],
            [sg.Button('Link Elevation', size=(20, 2))], 
        ]

event = 0

window = sg.Window('Kinesthetic 3D Audio Mixer', layout, size=(200, 300)) # Creates GUI Window
outport = md.open_output(name='midoport 1') # open mido port that connects to loopMIDI port

#    ***Programming Buttons***

while True:
    event, values = window.read()
    flag = True

    if event == "Stop": 
        flag = False
        bubba.flush()
        while True:
            if receive.is_alive(): 
                stop = "s"
                bubba.write(stop.encode('ascii'))
            else:
                bubba.close()
                break

    elif event == "Start":
        bubba = serial.Serial(port="COM5",baudrate=9600,timeout = 4)
        receive = threading.Thread(target=read_from_port_and_send2midi,args=(bubba,flag))
        receive.start() # start Serial Listener                                                         
    
    if event == "Link Azimuth":
        threading.Thread(target=link_az,args=(outport,),daemon=True).start()
    
    if event == "Link Elevation":
        threading.Thread(target=link_el,args=(outport,),daemon=True).start()

    if event == sg.WIN_CLOSED: # If window is closed
        outport.close()
        bubba.close()
        break




