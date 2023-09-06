#Đây là file màn hình chính để lựa chọn các chế độ, có import vào các file auto.py, map.py, manu.py là các chế độ


from tkinter import *
from PIL import Image, ImageTk
import os
from map import*                    #import file map.py
from manual import*                 #import file manual.py

window = Tk()

window.geometry("461x459")
window.configure(bg = "#ffffff")
canvas = Canvas(
    window,
    bg = "#F0F1F1",
    height = 459,
    width = 461,
    bd = 0,
    highlightthickness = 0,
    relief = "ridge")
canvas.place(x = 0, y = 0)
# background_img = ImageTk.PhotoImage(Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "images", "background.png")))
# background = canvas.create_image(
#     230.5, 229.5,
#     image=background_img)
img0 = ImageTk.PhotoImage(Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                                  "images", "img0.png")))
b0 = Button(
    image = img0,
    borderwidth = 0,
    highlightthickness = 0,
    command = btn_map_clicked,
    relief = "flat")

b0.place(
    x = 87, y = 193-80,
    width = 290,
    height = 80)

img1 = ImageTk.PhotoImage(Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                                  "images", "img1.png")))
b1 = Button(
    image = img1,
    borderwidth = 0,
    highlightthickness = 0,
    command = btn_control_clicked,
    relief = "flat")

b1.place(
    x = 88, y = 307-40,
    width = 290,
    height = 80)

window.resizable(True, True)
window.mainloop()


