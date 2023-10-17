#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor
from ev3dev2.sensor.lego import ColorSensor
cs = ColorSensor()
lm = LargeMotor()
lm.on(10)
lm.off()

print(cs.color)

if cs.color == cs.COLOR_BLACK:
    print("BLACK")
else:
    print("NOT black")
