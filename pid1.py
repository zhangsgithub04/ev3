# Import needed classes from the dev3dev2
# The code is for illustration purpose, not fully debugged. 
# We will try to debugg it together. 
# https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/stable/motors.html

import ev3dev.ev3 as ev3  #may need to use ev3dev2 ??? 
from time import sleep

class SZLineFollower:
    # Constructor
    def __init__(self, p, i, d, speed, dt):

        # PID tuning
        self.Kp = p  # proportional coefficient
        self.Ki = i  # integral coefficient
        self.Kd = d  # derivative coeffient
        self.speed=speed
        self.dt=dt
      
  
    # Main method
    def Start(self):

        # sensors
        # measures light intensity
        cs = ev3.ColorSensor()    
        assert cs.connected  
        cs.mode = 'COL-REFLECT'  # measure light intensity

        # motors
        lm = ev3.LargeMotor('outA');  assert lm.connected  # left motor
        rm = ev3.LargeMotor('outD');  assert rm.connected  # right motor

        integral = 0
        previous_error = 0

        # initial measurment
        target_value = cs.value()

        # Start the self correction steering loop
        while True:
            print("...")
            # Calculate steering using PID algorithm
            error = target_value - cs.value()
            integral += (error * self.dt)
            derivative = (error - previous_error)/self.dt

            u = (self.Kp * error) + (self.Ki * integral) + (self.Kd * derivative)


            # limit u to safe values: [-1000, 1000] deg/sec
            if speed + abs(u) > 1000:
                if u >= 0:
                    u = 1000 - speed
                else:
                    u = speed - 1000


            # u zero:     on target,  drive forward
            # u positive: too bright, turn right
            # u negative: too dark,   turn left

            # run motors 

            if u > 0:
                lmspeed=self.speed+u
                rmspeed=self.speed-u
                side=ev3.Leds.RIGHT
            elif u<0 :
                lmspeed=self.speed-u
                rmspeed=self.speed+u
                side=ev3.Leds.LEFT
            lm.run_timed(time_sp=self.dt, speed_sp=self.speed - u, stop_action='coast')
            rm.run_timed(time_sp=self.dt, speed_sp=self.speed + u, stop_action='coast')   
            ev3.Leds.set_color(side, ev3.Leds.GREEN)

            previous_error = error            
            sleep(dt/1000.0) #in second


# Main function
if __name__ == "__main__":
    car = SZLineFollower(1,0.1,0.1, 90, 500) // you can try different values
    car.start()
