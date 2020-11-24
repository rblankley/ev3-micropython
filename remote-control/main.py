#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# -------------------------------------------------------------------------------------------------
def main():
    ev3 = EV3Brick()
    ev3.speaker.beep()
    ev3.screen.clear()
    ev3.light.on( Color.YELLOW )

    colors = ColorSensor( Port.S3 )
    ir = InfraredSensor( Port.S4 )

    left_motor = Motor( Port.B, Direction.CLOCKWISE )
    left_speed = 0
    left_pressed = False

    right_motor = Motor( Port.C, Direction.CLOCKWISE )
    right_speed = 0
    right_pressed = False    

    speed_mult = 128

    ev3.light.off()

    while not ev3.buttons.pressed():

        b = []
        ch = 0

        # look for button(s) pressed in channel
        for channel in range( 1, 5 ):
            cb = ir.buttons( channel )

            if ( len(cb) ):
                b = cb
                ch = channel
                break

        # left motor control            
        if ( Button.LEFT_UP in b ):
            if ( not left_pressed ):
                left_pressed = True
                if ( left_speed < 0 ):
                    left_speed = 0
                else:
                    left_speed = speed_mult * ch

        elif ( Button.LEFT_DOWN in b ):
            if ( not left_pressed ):
                left_pressed = True
                if ( 0 < left_speed ):
                    left_speed = 0
                else:
                    left_speed = -1 * speed_mult * ch

        else:
            left_pressed = False

        # right motor control
        if ( Button.RIGHT_UP in b ):
            if ( not right_pressed ):
                right_pressed = True
                if ( right_speed < 0 ):
                    right_speed = 0
                else:
                    right_speed = speed_mult * ch

        elif ( Button.RIGHT_DOWN in b ):
            if ( not right_pressed ):
                right_pressed = True
                if ( 0 < right_speed ):
                    right_speed = 0
                else:
                    right_speed = -1 * speed_mult * channel

        else:
            right_pressed = False

        left_motor.run( left_speed )
        right_motor.run( right_speed )

        wait(50)

# -------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
