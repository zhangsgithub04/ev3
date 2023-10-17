#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTan$
from ev3dev2.sensor.lego import TouchSensor, ColorSensor
#Touch sensor just in case

from ev3dev2.led import Leds

# The code is for illustration purpose, pay attention to how to two motors run sequentially (blocking) 
# This version won't work for our purpose. We will have to use multithreading so the each thread runs a motor, two threads concurrently.
# Actually, tank drive or tank steer should also implemented two threads, as far as I believe. 
# https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.ht$

from time import sleep

class SZLineFollower:
    # Constructor
    def __init__(self, p, i, d, speed, dt):

        # PID tuning
        self.Kp = p # proportional coefficient
        self.Ki = i # integral coefficient
        self.Kd = d # derivative coeffient
        self.speed=speed
        self.dt=dt


    # Main method
    def Run(self):

        # sensors measures light intensity
        cs = ColorSensor()
        cs.mode = 'COL-REFLECT' # measure light intensity

        # motors
        lm = LargeMotor(OUTPUT_A);
        #assert lm.connected # left motor
        rm = LargeMotor(OUTPUT_D);
        #assert rm.connected # right motor
        leds=Leds()
        integral = 0
        previous_error = 0

        # initial measurment
        target_value = cs.value()

        # Start the self correction steering loop
        while True:
            print("...")
            cv=cs.value()
            print(cv)
            # Calculate steering using PID algorithm
            error = target_value-cv
            integral += (error * self.dt)
            derivative = (error - previous_error)/self.dt

            u = (self.Kp * error) + (self.Ki * integral) + (self.Kd *
derivative)


            # limit u to safe values: [-1000, 1000] deg/sec
            if self.speed + abs(u) > 1000:
                if u >= 0:
                    u = 1000 - self.speed
                else:
                    u = self.speed - 1000


            # u zero: on target, drive forward u positive: too bright,
            # turn right u negative: too dark, turn left

            # run motors
            side="LEFT" #??? other options?
            if u > 0:
                lmspeed=self.speed+u
                rmspeed=self.speed-u
                side="RIGHT"
            elif u<0 :
                lmspeed=self.speed-u
                rmspeed=self.speed+u
                side="LEFT"
            lm.run_timed(time_sp=self.dt, speed_sp=self.speed - u,
stop_action='coast')
            rm.run_timed(time_sp=self.dt, speed_sp=self.speed + u,
stop_action='coast')
            leds.set_color(side, "RED")

            previous_error = error
            sleep(self.dt/1000.0) #in second


# Main function
if __name__ == "__main__":
    car = SZLineFollower(1,0.1,0.1, 90, 500)
    #you can try different values
    car.Run()






