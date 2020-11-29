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

from di_sensors import dGPS

# -------------------------------------------------------------------------------------------------
def main():
    ev3 = EV3Brick()
    ev3.speaker.beep()
    ev3.screen.clear()
    ev3.light.on( Color.YELLOW )

    gps = dGPS( Port.S3 )

    print( 'Firmware Version', gps.firmwareVersion() )

    while ( not ev3.buttons.pressed() ):

        # wait for link
        if ( not gps.link() ):
            print( "No link..." )

        else:
            ev3.light.on( Color.GREEN )

            print( 'Link', True )
            print( 'UTC', gps.utc() )
            
            (lat, lon) = gps.location()
            print( 'Latitude %.6f' % lat )
            print( 'Longitude %.6f' % lon )
            
            print( 'Heading', gps.heading() )
            print( 'Velocity', gps.velocity() )
            print( 'Altitude', gps.altitude() )
            print( 'HDOP', gps.hdop() )
            print( 'Satellites in view', gps.satellitesInView() )
 
        wait( 2500 )

# -------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
