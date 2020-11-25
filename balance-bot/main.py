#!/usr/bin/env pybricks-micropython

"""
More about EV3 MicroPython programming here:
https://pybricks.github.io/ev3-micropython/hubs.html

This robot build using 31313_BALANC3R.pdf instructions. Instead of the EV3 Gyroscopic sensor or
the HiTechnic Gyroscopic sensor we are using the Dexter Industries dIMU.

Ports
-----

B - Right side (looking from back) large motor
D - Left side (looking from back) large motor

S2 - dIMU
S3 - IR sensor

Remote Controls (channel 1)
---------------------------
Left Up + Right Up              forward
Left Down + Right Down          backward
Left Up                         rotate clockwise
Right Down                      rotate clockwise
Left Down                       rotate counter-clockwise
Right Up                        rotate counter-clockwise
Left Up + Right Down            rotate clockwise quickly
Left Down + Right Up            rotate counter-clockwise quickly
Beacon                          quit

"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from di_sensors import dCompass
from di_sensors import dIMU

import time

# -------------------------------------------------------------------------------------------------
def performCalibration( ev3, imu ):
    """! Calibrate the dIMU
    @param ev3  ev3 brick
    @param imu  dIMU
    """

    ev3.speaker.say( "calibrating in 3" )
    ev3.speaker.say( "2" )
    ev3.speaker.say( "1" )

    imu.calibrate()

    ev3.speaker.say( "calibration complete" )

# -------------------------------------------------------------------------------------------------
def main():
    ev3 = EV3Brick()
    ev3.speaker.beep()
    ev3.screen.clear()
    ev3.light.on( Color.RED )

    right_motor = Motor( Port.B, Direction.CLOCKWISE )
    left_motor = Motor( Port.D, Direction.CLOCKWISE )
    wheel_diameter = 42

    right_motor.reset_angle( 0.0 )
    left_motor.reset_angle( 0.0 )

    imu = dIMU( Port.S2 )
    ir = InfraredSensor( Port.S3 )

    # calibrate IMU
    # hold balance bot upright while calibration is in process
    performCalibration( ev3, imu )

    ev3.light.on( Color.YELLOW )

    ## ---- ##

    ir_channel = 1

    emer_off_tilt_angle = 0.3

    emer_off_ticks_max = 4
    emer_off_ticks = 0

    gyro_ang = 0.0                                  # gyro angle in degrees

    motor_pos = 0.0                                 # Rotation angle of motor in degrees
    motor_sum = 0.0

    d = [0.0, 0.0, 0.0, 0.0]
    stamp = time.time()

    loops = 10                                      # Initialization

    speed = 0                                       # Forward motion speed of robot [-10,10]
    direction = 0                                   # Direction of robot [-50(left),50(right)]

    while True:

        # Get time in seconds since last step
        now = time.time()
        dt = now - stamp

        stamp = now

        # read gyro
        (gx, gy, gz) = imu.gyro_axes()

        gyro_spd = gz                               # degrees / sec
        gyro_ang += (gyro_spd * dt);                # integrate angle speed to get angle

        # Get motor rotation angle and rotational angle speed
        prev_motor_sum = motor_sum
        motor_sum = right_motor.angle() + left_motor.angle()

        d.append( motor_sum - prev_motor_sum )
        d.pop( 0 )
        
        motor_pos += d[-1]
        motor_spd = ( sum(d) / len(d) ) / dt;           # motor rotational speed

        # check initialized
        if ( 0 < loops ):
            loops -= 1

            if ( not loops ):
                ev3.light.on( Color.GREEN )

        # compute new motor power in [-100,100]
        else:
            motor_pos -= speed
            pwr = 0.08 * motor_spd + 0.12 * motor_pos + 1.0 * gyro_spd + 25 * gyro_ang

            if ( pwr > 100 ):
                pwr = 100
            elif ( pwr < -100 ):
                pwr = -100

            right_motor.dc( int( pwr - direction ) )
            left_motor.dc( int( pwr + direction ) )

        # check too much tilt
        (ax, ay, az) = imu.acc_axes( True )

        if ( abs( ay ) <= emer_off_tilt_angle ):
            emer_off_ticks = 0

        else:
            emer_off_ticks += 1

            if ( emer_off_ticks_max <= emer_off_ticks ):
                ev3.light.on( Color.RED )
                break

        # check buttons
        b = ir.buttons( ir_channel )

        if ( Button.BEACON in b ):
            break

        elif (( Button.LEFT_UP in b ) and ( Button.RIGHT_UP in b )):
            speed = 20
            direction = 0
        elif (( Button.LEFT_DOWN in b ) and ( Button.RIGHT_DOWN in b )):
            speed = -20
            direction = 0

        elif (( Button.LEFT_UP in b ) and ( Button.RIGHT_DOWN in b )):
            speed = 0
            direction = 35
        elif (( Button.RIGHT_UP in b ) and ( Button.LEFT_DOWN in b )):
            speed = 0
            direction = -35

        elif (( Button.LEFT_UP in b ) or ( Button.RIGHT_DOWN in b )):
            speed = 0
            direction = 25
        elif (( Button.RIGHT_UP in b ) or ( Button.LEFT_DOWN in b )):
            speed = 0
            direction = -25

        else:
            speed = 0
            direction = 0

    right_motor.stop()
    left_motor.stop()

# -------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
