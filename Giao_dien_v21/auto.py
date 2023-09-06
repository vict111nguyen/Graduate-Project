#Chế độ tự động nếu lắp dò line

from tkinter import *
from tkinter import messagebox
from PIL import Image, ImageTk
from tkinter import ttk
from functools import partial
import tkinter
import os
from tkintermapview import TkinterMapView
import serial
import math

class btn_auto_clicked:

    def __init__(self):
        print("Vào chế độ tự động")
        
        auto = Toplevel()
        auto.geometry("751x199")
        auto.configure(bg = "#F0F1F1")
        canvas_auto = Canvas(
            auto,
            bg = "#F0F1F1",
            height = 199,
            width = 751,
            bd = 0,
            highlightthickness = 0,
            relief = "ridge")
        canvas_auto.place(x = 0, y = 0)


        # background_img_auto = ImageTk.PhotoImage(Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "images", "background_auto.png")))
        # background = canvas_auto.create_image(
        #     375.5, 99.5,
        #     image=background_img_auto)
        L = Label(auto, text= "Xe đang tự động chạy theo line", font=("Alike", 28)).place(x = 40, y = 60)
    
        img0_auto = ImageTk.PhotoImage(Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "images", "return.png")))
        
        b_auto = Button(
            auto,
            image = img0_auto,
            borderwidth = 0,
            highlightthickness = 0,
            command = lambda: auto.destroy(),
            relief = "flat")

        b_auto.place(
            x = 588, y = 112,
            width = 160,
            height = 79)

        auto.resizable(True, True)
        auto.mainloop()
