from tkinter import *
from serial import *
from PIL import *
import time

ser= serial.Serial(port='COM8',baudrate=4800, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS, timeout=0)

root = Tk()

root.title("Segundo Proyecto")
root.geometry('300x170')
root.config(bg='gray')
root.resizable(0,0)

def boton1():
    print("Iniciar Grabacion")
def boton2():
    print("Detener Grabacion")
def boton3():
    print("Reproducir")
def boton4():
    print("Pausa")

Label1 = Label(text="Segundo Proyecto", bg='gray', fg='white')
Label2 = Label(text="Programacion de Microcontroladores", bg='gray', fg='white')
b1 = Button(text="  Iniciar Grabacion  ", command=boton1, bg='gray', fg='white')
b2 = Button(text="Detener Grabacion", command=boton2, bg='gray', fg='white')
b3 = Button(text="       Reproducir       ", command=boton3, bg='gray', fg='white')
b4 = Button(text="            Pausa            ", command=boton4, bg='gray', fg='white')

l1 = Label(text="Voltaje 1: ", bg='gray', fg='white')
l2 = Label(text="Voltaje 2: ", bg='gray', fg='white')
l3 = Label(text="Voltaje 3: ", bg='gray', fg='white')
l4 = Label(text="Voltaje 4: ", bg='gray', fg='white')

Labelesp = Label(bg='gray')

Label1.grid(row=0)
Label2.grid(row=1)
Labelesp.grid(row=2)
b1.grid(row=3, column=0)
b2.grid(row=4, column=0)
b3.grid(row=5, column=0)
b4.grid(row=6, column=0)

l1.grid(row=3, column=1)
l2.grid(row=4, column=1)
l3.grid(row=5, column=1)
l4.grid(row=6, column=1)



root.mainloop()
