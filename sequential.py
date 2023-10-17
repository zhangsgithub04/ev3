#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds

import time

# TODO: Add code here

ml = LargeMotor(OUTPUT_A)

mr = LargeMotor(OUTPUT_D)

num=0
while num<3:
        ml.on_for_rotations(SpeedPercent(75), 5)
        mr.on_for_rotations(SpeedPercent(75), 5)
        time.sleep(1)
        num+=1
