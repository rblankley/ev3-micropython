#!/usr/bin/env pybricks-micropython

"""
More about EV3 MicroPython programming here:
https://pybricks.github.io/ev3-micropython/hubs.html
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# -------------------------------------------------------------------------------------------------
def checkForLine( ev3, colors ):
    """! Check for a line under color sensor
    Look for a mostly blue line under color sensor. We use that blue paint masking tape on our
    floor (which is brown in our case). Look for a blue reflectivity coming off color sensor.
    @param ev3  ev3 brick
    @param colors  color sensor
    @return  @c True if line found, @c False otherwise
    """
    rgb = colors.rgb()

    if ( 3 == len( rgb ) ):
        if (( rgb[0] < rgb[2] ) and ( rgb[1] < rgb[2] )):
            ev3.light.on( Color.GREEN )
            return True

    ev3.light.on( Color.RED )
    return False

# -------------------------------------------------------------------------------------------------
def findLineSmallMotion( ev3, colors, left_motor, right_motor, dir ):
    """! Move slightly left or right to see if we can find the line
    @param ev3  ev3 brick
    @param colors  color sensor
    @param left_motor  left motor
    @param right_motor  right motor
    @param dir  direction of first turn; 1 = clockwise, -1 = counter clockwise
    @return  0 if no line found, 1 if line found while turning clockwise, -1 if line found while
             turning counter clockwise
    """

    speed = 100
    half_robot_width = 150 # 200
    half_line_width = 75

    # move forward to center robot on top of line
    left_motor.run_angle( 2 * speed, half_robot_width, then=Stop.HOLD, wait=False )
    right_motor.run_angle( 2 * speed, half_robot_width, then=Stop.HOLD, wait=True )

    # dir
    # 1 = clockwise
    # -1 = counter clockwise

    # turn direction
    left_motor.run_angle( dir * speed, half_line_width, then=Stop.HOLD, wait=False )
    right_motor.run_angle( dir * -speed, half_line_width, then=Stop.HOLD, wait=True )

    if ( checkForLine( ev3, colors ) ):
        return dir

    # turn opposite of direction
    right_motor.run_angle( -1 * dir * -speed, 2 * half_line_width, then=Stop.HOLD, wait=False )
    left_motor.run_angle( -1 * dir * speed, 2 * half_line_width, then=Stop.HOLD, wait=True )

    if ( checkForLine( ev3, colors ) ):
        return (-1 * dir)

    # reset
    left_motor.run_angle( dir * speed, half_line_width, then=Stop.HOLD, wait=False )
    right_motor.run_angle( dir * -speed, half_line_width, then=Stop.HOLD, wait=True )

    return 0

# -------------------------------------------------------------------------------------------------
def findLineLargeMotion( ev3, colors, left_motor, right_motor, dir ):
    """! Rotate in both directions to see if we can find the line
    @param ev3  ev3 brick
    @param colors  color sensor
    @param left_motor  left motor
    @param right_motor  right motor
    @param dir  direction of first turn; 1 = clockwise, -1 = counter clockwise
    @return  0 if no line found, 1 if line found while turning clockwise, -1 if line found while
             turning counter clockwise
    """

    speed = 200
    half_line_width = 75

    rotate_180_width = 1000
    rotate_180_width_stop_short = 800

    # turn direction
    left_motor.run_angle( dir * speed, rotate_180_width_stop_short, then=Stop.HOLD, wait=False )
    right_motor.run_angle( dir * -speed, rotate_180_width_stop_short, then=Stop.HOLD, wait=False )

    # turn until we find line or we complete turn
    while True:
        if ( checkForLine( ev3, colors ) ):
            left_motor.stop()
            right_motor.stop()
            left_motor.run_angle( dir * speed, half_line_width, then=Stop.HOLD, wait=False )
            right_motor.run_angle( dir * -speed, half_line_width, then=Stop.HOLD, wait=True )
            return dir

        if (( 0 == left_motor.speed() ) and ( 0 == right_motor.speed() )):
            break

        wait( 10 )

    if ( checkForLine( ev3, colors ) ):
        return dir

    # reset
    left_motor.run_angle( -1 * dir * speed, rotate_180_width_stop_short, then=Stop.HOLD, wait=False )
    right_motor.run_angle( -1 * dir * -speed, rotate_180_width_stop_short, then=Stop.HOLD, wait=True )

    # turn opposite direction
    left_motor.run_angle( -1 * dir * speed, rotate_180_width_stop_short, then=Stop.HOLD, wait=False )
    right_motor.run_angle( -1 * dir * -speed, rotate_180_width_stop_short, then=Stop.HOLD, wait=False )

    # turn until we find line or we complete turn
    while True:
        if ( checkForLine( ev3, colors ) ):
            left_motor.stop()
            right_motor.stop()
            left_motor.run_angle( -1 * dir * speed, half_line_width, then=Stop.HOLD, wait=False )
            right_motor.run_angle( -1 * dir * -speed, half_line_width, then=Stop.HOLD, wait=True )
            return (-1 * dir)

        if (( 0 == left_motor.speed() ) and ( 0 == right_motor.speed() )):
            break

        wait( 10 )

    if ( checkForLine( ev3, colors ) ):
        return (-1 * dir)

    # reset
    left_motor.run_angle( dir * speed, rotate_180_width_stop_short, then=Stop.HOLD, wait=False )
    right_motor.run_angle( dir * -speed, rotate_180_width_stop_short, then=Stop.HOLD, wait=True )

    return 0

# -------------------------------------------------------------------------------------------------
def main():
    ev3 = EV3Brick()
    ev3.speaker.beep()
    ev3.screen.clear()
    ev3.light.on( Color.YELLOW )

    colors = ColorSensor( Port.S3 )
    left_motor = Motor( Port.B, Direction.CLOCKWISE )
    right_motor = Motor( Port.C, Direction.CLOCKWISE )

    startup = True
    direction = 1

    ev3.light.off()

    while ( not ev3.buttons.pressed() ):

        if ( not checkForLine( ev3, colors ) ):
            left_motor.stop()
            right_motor.stop()

            # no more line! either:
            # a) the line turned
            # b) we were not running straight and went to side of line
            # c) there was never a line to begin with

            # (b)
            newDir = findLineSmallMotion( ev3, colors, left_motor, right_motor, direction )

            if ( 0 != newDir ):
                direction = newDir
            else:
                # (a)
                newDir = findLineLargeMotion( ev3, colors, left_motor, right_motor, direction )

                if ( 0 != newDir ):
                    direction = -1 * newDir
                else:
                    startup = True

            # (c)
            if ( startup ):
                ev3.speaker.beep()

                ev3.speaker.say( "put me on the line" )
                while ( not checkForLine( ev3, colors ) ):
                    wait( 100 )

                ev3.speaker.say( "press any button to start" )
                while ( not ev3.buttons.pressed() ):
                    wait( 10 )
                
                continue

        startup = False

        # go
        left_motor.run( 400 )
        right_motor.run( 400 )

        wait( 10 )

# -------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
