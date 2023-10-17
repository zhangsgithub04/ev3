#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor.lego import ColorSensor


# The code is for illustration purpose, not fully debugged.
# need to deal with range of u and speed .... to be debugged together
# https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html

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

        TankDrive=MoveTank(OUTPUT_A, OUTPUT_D)
        integral = 0
        previous_error = 0

        # initial measurment
        target_value = cs.value()
        print("Target value is: ", target_value)
        # Start the self correction steering loop
        while True:
            cv=cs.value()
            print("Reflected Light: ", cv)

            # Calculate steering using PID algorithm
            error = cv-target_value
            integral += (error * self.dt)
            derivative = (error - previous_error)/self.dt

            u = (self.Kp * error) + (self.Ki * integral) + (self.Kd *
derivative)

            print("u is: ", u)
            # limit u to safe values: [0, 100] deg/sec

            u=u%100

            # u zero: on target, drive forward u positive: too bright,

            # turn right u negative: too dark, turn left

            # run motors
            lmspeed=rmspeed=self.speed
            if u > 0:
                lmspeed=self.speed-u
                rmspeed=self.speed+u
            elif u<0 :
                lmspeed=self.speed+u
                rmspeed=self.speed-u
            #lm.run_timed(time_sp=self.dt, speed_sp=self.speed - u, stop_action='coast')
            #rm.run_timed(time_sp=self.dt, speed_sp=self.speed + u, stop_action='coast')
            TankDrive.on_for_seconds(SpeedPercent(lmspeed), SpeedPercent(rmspeed),self.dt/1000)
            previous_error = error
            #sleep(self.dt/1000.0) #in second


# Main function
if __name__ == "__main__":

    car = SZLineFollower(0.005,0.001,0.001, 50, 500)
    #you can try different values
    car.Run()

