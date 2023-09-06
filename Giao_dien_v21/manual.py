#Điều khiển bằng tay

from tkinter import *
from PIL import Image, ImageTk
import os
from map import*                    #import file map.py
class btn_control_clicked:
    def __init__(self):
        
        self.control = Toplevel()
        self.control.geometry("800x600")
        self.control.configure(bg = "#F0F1F1")
        print("Vào chế độ điều khiển thủ công")
        
        # Define các button command
        # my_slider.grid(row=0, column=2, padx= 10)

        canvas_ctrl = Canvas(
            self.control,
            bg = "#F0F1F1",
            height = 423,
            width = 620,
            bd = 0,
            highlightthickness = 0,
            relief = "ridge")

        
        canvas_ctrl.place(x = 0, y = 0)

        def slide(v):
            new_value =  mySlider.get() 
            
            return new_value
        mySlider = Scale(self.control, from_=0, to=100, orient=HORIZONTAL, command=slide, length= 200)
        mySlider.set(40)
        mySlider.pack(anchor= SE)
        
        def handle_button_click(c):
            a = int(2*mySlider.get())
            b=int(254)  
            sent=self.sent_from_gui(b,c,a)
            mystring = " ".join(str(char) for char in sent)
            print(sent)
            msg = bytes.fromhex(mystring)   
            # ser.write(msg)

        images = [("upleft.png", 1),("up.png", 2),("upright.png", 3),("left.png", 4),("stop.png", 5),
                  ("right.png", 6),("downleft.png", 7),("down.png", 8),("downright.png", 9)]

        for img in images:
            photo = ImageTk.PhotoImage(Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                "images", img[0])))
            button = Button(
                self.control,
                image = photo,
                borderwidth = 0,
                highlightthickness = 0,
                command = lambda n=img[1]: handle_button_click(n),
                relief = "flat"
            )
            button.image = photo
            button.place(
                x = 30 + 160 * ((img[1] - 1) % 3), y = 60 + 160 * ((img[1] - 1) // 3),
                width = 130, height = 130)


        self.returnn = ImageTk.PhotoImage(Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                                                  "images", "return.png")))
        self.b_return = Button(
            self.control,
            image = self.returnn,
            borderwidth = 0,
            highlightthickness = 0,
            command = lambda: self.control.destroy(),
            relief = "flat")
        self.b_return.place(
            x = 650, y = 500,
            width = 143,
            height = 90)
        self.control.resizable(False, False)
        self.control.mainloop()
    def btn_clicked(self):
            print("Button clicked")

    mode=""
    direct=""
    speed=""
    def sent_from_gui(self, mode, direct, speed):
        a=[int(mode) , int(direct), int(speed)]
        a[0]='{0:02x}'.format(a[0])
        a[1]='{0:02x}'.format(a[1])
        a[2]='{0:02x}'.format(a[2])
        return a

