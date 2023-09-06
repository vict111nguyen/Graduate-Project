from tkinter import *
import asyncio
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk
import os
from tkintermapview import TkinterMapView
import serial
import tkinter.messagebox
import threading
import time

EARTH_RADIUS = 6372795 
SAFE_DISTANCE = 1
STOP_DISTANCE = 0.3

coorTargetList = []
currentCoor = []
temp = 0
ser= serial.Serial('COM20',9600,timeout =1)
def sent_from_gui(a,b):
    byte0=255&0xFF
    byte1= int(a)>>8&0xFF
    byte2= int(a)&0xFF
    byte3= int(b)>>8&0xFF
    byte4= int(b)&0xFF
    t=[byte0,byte1,byte2,byte3,byte4] 
    t[0]='{0:02x}'.format(t[0]) 
    t[1]='{0:02x}'.format(t[1])
    t[2]='{0:02x}'.format(t[2])
    t[3]='{0:02x}'.format(t[3])
    t[4]='{0:02x}'.format(t[4])
    return t
def worker():
    global lat
    global log
    global currentCoor
    b_newDT = False
    i = 0
    buff = [0,0,0,0,0,0,0,0,0]
    while True:
        if ser.in_waiting > 0:
            temp = ser.read()[0]
            if temp == 255:
                b_newDT = True
                i = 0
            if b_newDT:
                buff[i] = temp
                i += 1
            if i >= 9:
                b_newDT = False
            lat = ((buff[1] << 24) + (buff[2] << 16) + (buff[3]<<8) + (buff[4]<<0))/1000000
            print(lat)
            log = ((buff[5] << 24) + (buff[6] << 16) + (buff[7]<<8) + (buff[8]<<0))/1000000
            print(log)
            currentCoor = [lat,log]
            with open('currentCoord.txt', 'a') as f:
                f.write('\n')
                f.write(str(currentCoor))

class btn_map_clicked:
    
    def __init__(self):
        print ("Vào chế độ điều khiển bằng map") 
        # create tkinter window
        global lat
        global log
        
        map = Toplevel()
        map.geometry("600x600")
        map.title("Map")
        # Frame, button, slider
        def find():
            map_widget.set_address(my_entry.get())
        def slider(s):
            map_widget.set_zoom(my_slider.get())


        my_frame=LabelFrame(map)
        my_frame.pack(pady= 20)
        my_entry = Entry(my_frame, font=("Alike", 18))
        my_entry.grid(row= 0, column=0, padx=10, pady=10)

        my_button = Button(my_frame, text="Find", font=("Alike", 12), command=find)
        my_button.grid(row=0,column=1, padx= 10)

        my_slider = ttk.Scale(my_frame, from_=22, to=2, orient=HORIZONTAL, command=slider, length= 220, value= 20)
        my_slider.grid(row=0, column=2, padx= 10)

        # create map widget
        map_widget = TkinterMapView(map, width=600, height=600, corner_radius=5)
        map_widget.pack(fill="both", expand=True)

        #API của google map (Kiểu normal và kiểu satellite)
        # map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        
        # create marker through .set_address() with image, which is visible at zoom levels 14 to infinity
        marker_1 = map_widget.set_address("21.1608767 105.9217283" )
        # make image visible/invisible when marker is clicked
        import math

        thread = threading.Thread(target=worker)
        thread.start()
        def get_distance_and_course(start_lat, start_long, end_lat, end_long):
            delta_long = math.radians(end_long - start_long)
            start_lat = math.radians(start_lat)
            end_lat = math.radians(end_lat)
            a = math.sin(delta_long) * math.cos(end_lat)
            b = math.sin(start_lat) * math.cos(end_lat) * math.cos(delta_long)
            b = math.cos(start_lat) * math.sin(end_lat) - b
            heading = math.atan2(a, b)
            if heading < 0.0:
                heading += 2 * math.pi
            distance = math.acos((math.sin(start_lat) * math.sin(end_lat)) + (math.cos(start_lat) * math.cos(end_lat) * math.cos(delta_long))) * EARTH_RADIUS
            return (distance, math.degrees(heading))

        # create marker through .set_marker() with image, which is visible at all zoom levels  
        def current_marker():
            global lat
            global log
            global temp
            global coorTargetList
            while True:
                map_widget.delete_all_marker()
                map_widget.set_marker(lat,log)

                if len(coorTargetList) != 0:
                    map_widget.delete_all_path()
                    map_widget.set_path([(lat, log), (coorTargetList[0][0], coorTargetList[0][1])])
                    d,angle=get_distance_and_course(lat, log, coorTargetList[0][0], coorTargetList[0][1])
                    if(d<3.5):
                        coorTargetList.pop(0)
                        map_widget.delete_all_path()
                        if(len(coorTargetList) == 0):
                            print("Hoan thanh")
                            tkinter.messagebox.showinfo(title="", message="Done!")
                        else:
                            d,angle=get_distance_and_course(lat, log, coorTargetList[0][0], coorTargetList[0][1])
                    sent=sent_from_gui(d,angle)
                    print(d,angle)
                    mystring = " ".join(str(char) for char in sent)   
                    msg = bytes.fromhex(mystring)   
                    ser.write(msg)  
                    print(sent)
                    time.sleep(0.5)
                for index in range(len(coorTargetList)):
                    map_widget.set_marker_end(coorTargetList[index][0], coorTargetList[index][1])
                    if index < len(coorTargetList) - 1:
                        if len(coorTargetList) > 1:
                            coorTargetLast = coorTargetList[index + 1]
                            coorTargetFirst = coorTargetList[index]
                            if index != len(coorTargetList):
                                map_widget.set_path([(coorTargetLast[0], coorTargetLast[1]), (coorTargetFirst[0], coorTargetFirst[1])])

        def add_marker_event(coords):
            print("Add marker:", coords)
            global coorTargetList
            coorTargetList.append(coords)
            with open('coorTargetList.txt', 'a') as f:
                f.write('\n')
                f.write(str(coords))
            
        def showCurrent(arg):
            threadShowCurent = threading.Thread(target=current_marker)
            threadShowCurent.start()

        #Hàm event cho nhấp chuột phải    
        map_widget.add_right_click_menu_command(label="Add Marker",
                                            command=add_marker_event,
                                            pass_coords=True)
        map_widget.add_left_click_map_command(showCurrent)                               
        
        map.mainloop()
        return




