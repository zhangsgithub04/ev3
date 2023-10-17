#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
import threading # Let us use two threads, for two wheel motors :)
import time

# TODO: Add code here
def lm(n):
    ml.on_for_rotations(SpeedPercent(75), n)

def rm(n):
    mr.on_for_rotations(SpeedPercent(75), n)

ml = LargeMotor(OUTPUT_A)
mr = LargeMotor(OUTPUT_D)


num=0
while num<3:
    tl=threading.Thread(target=lm,args=(10,))
    tr=threading.Thread(target=rm,args=(10,))
    tl.start()
    tr.start()
    tl.join()
    tr.join()
    time.sleep(1)
    num+=1
