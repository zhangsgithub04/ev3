#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor
from ev3dev2.sensor.lego import ColorSensor

cs = ColorSensor()
lm = LargeMotor()

lm.on(30)
while not cs.color == cs.COLOR_YELLOW:
    pass
lm.off()
