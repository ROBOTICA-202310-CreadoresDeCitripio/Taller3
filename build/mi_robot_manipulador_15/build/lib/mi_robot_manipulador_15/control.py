import keyboard
import serial

arm = serial.Serial('COM3', 9600)
while True:
    if keyboard.is_pressed("a"):
        arm.write("1.0,0.0,0.0")
    elif keyboard.is_pressed("q"):
        arm.write("-1.0,0.0,0.0")
    elif keyboard.is_pressed("s"):
        arm.write("0.0,1.0,0.0")
    elif keyboard.is_pressed("w"):
        arm.write("0.0,-1.0,0.0")
    elif keyboard.is_pressed("d"):
        arm.write("0.0,0.0,1.0")
    elif keyboard.is_pressed("e"):
        arm.write("0.0,0.0,-1.0")
    elif keyboard.is_pressed("c"):
        arm.write("1.0,1.0,1.0")
    else:
        arm.write("0.0,0.0,0.0")

