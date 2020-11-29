from pybricks.iodevices import I2CDevice

import math
import time

# =================================================================================================
class dCompass( I2CDevice ):
    """! Dexter Industries dCompass (HMC5883L) """
    
    SCALES = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    I2C_ADDRESS = 0x1e                              # Device address

    CONFIG_REGA = 0x00                              # Address of Configuration register A
    CONFIG_REGB = 0x01                              # Address of configuration register B
    MODE_REG = 0x02                                 # Address of mode register

    IDENT_REGA = 0x0a                               # Address of Identification register A
    DATA_REG = 0x03                                 # Address of X-axis MSB data register

    # ---------------------------------------------------------------------------------------------
    def __init__( self, port, declination = 0, gauss = 1.3 ):
        """! Initialize Class
        Lookup your declination here: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
        @param self  this object
        @param port  mindstorms port
        @param declination  declination (degrees)
        @param gauss  gauss setting
        """
        super().__init__( port, self.I2C_ADDRESS )

        self.__declination = declination * math.pi/180
        self.__valid = False

        (regval, self.__scale) = self.SCALES[gauss]

        # Validate
        data = self.read( self.IDENT_REGA, 3 )

        if ( b'H43' == data ):
            self.__valid = True

            # 8 Average, 15 Hz, normal measurement
            self.write( self.CONFIG_REGA, bytes((0x70,)) )

            # Scale
            self.write( self.CONFIG_REGB, bytes((regval << 5,)) )

            # Continuous measurement
            self.write( self.MODE_REG, bytes((0x00,)) )

    # ---------------------------------------------------------------------------------------------
    def declination( self ):
        """! Retrieve compass declination
        @param self  this object
        @return  declination (degrees)
        """
        return self.__declination * 180/math.pi

    # ---------------------------------------------------------------------------------------------
    def twos_complement( self, val, len ):
        """! Convert twos compliment to integer
        @param self  this object
        @param val  value
        @param len  length in bits
        @return  twos complement
        """
        if ( ( val & (1 << len - 1) ) != 0 ):
            val = val - (1<<len)
        return val

    # ---------------------------------------------------------------------------------------------
    def __convert( self, data, offset ):
        """! Convert raw value
        @param self  this object
        @param data  raw readings from device
        @param offset  offset of reading
        @return  converted value
        """
        val = self.twos_complement( data[offset] << 8 | data[offset+1], 16 )
        return round( val * self.__scale, 4 )

    # ---------------------------------------------------------------------------------------------
    def axes( self ):
        """! Retrieve axis data from device
        @param self  this object
        @return  (x, y, z) reading from device
        """
        if ( not self.__valid ):
            return None

        data = self.read( self.DATA_REG, 6 )

        if ( len(data) != 6 ):
            return None

        x = self.__convert( data, 0 )
        y = self.__convert( data, 4 )
        z = self.__convert( data, 2 )

        return ( x, y, z )

    # ---------------------------------------------------------------------------------------------
    def calcHeading( self ):
        """! Calculate heading
        @param self  this object
        @return  heading (degrees)
        """
        (x, y, z) = self.axes()
        headingRad = math.atan2( y, x )
        headingRad += self.__declination

        # Correct for reversed heading
        if ( headingRad < 0 ):
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif ( headingRad > 2 * math.pi ):
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        return headingRad * 180/math.pi

# =================================================================================================
class dIMU:
    """! Dexter Industries dIMU (L3G4200D gyro, MMA7455L accel)

    Axis Alignment
    --------------

    Looking down at the top of the DIMU as such:

      +-------------+           Accel Axis              Gyro Axis
    +---+           |
    |   |           |                ^  Y                    ^  X
    +---+     G   A |                |                       |
      +-------------+            Z   |                   Z   |
                                (X)  +-----> X          (X)  +-----> Y
    """

    GYRO_DPS = {
        250: [0x00, 128.0],
        500: [0x10, 64.0],
        2000: [0x30, 16.0],
    }

    ACCEL_RANGE = {
        2: [0x04, 64.0],
        4: [0x08, 32.0],
        8: [0x00, 16.0],
    }

    GYRO_I2C_ADDR = 0x69                            # Gyro I2C address
    ACC_I2C_ADDR = 0x1D                             # Accelerometer I2C address

    CTRL4_BLOCKDATA = 0x80

    GYRO_WHOAMI = 0x0F
    GYRO_CTRL_REG1 = 0x20                           # CTRL_REG1 for Gyro
    GYRO_CTRL_REG2 = 0x21                           # CTRL_REG2 for Gyro
    GYRO_CTRL_REG3 = 0x22                           # CTRL_REG3 for Gyro
    GYRO_CTRL_REG4 = 0x23                           # CTRL_REG4 for Gyro
    GYRO_CTRL_REG5 = 0x24                           # CTRL_REG5 for Gyro
    GYRO_DATA_REG = 0x28                            # Angular data register for Gyro

    ACC_WHOAMI = 0x0F
    ACC_MCTRL_REG = 0x16
    ACC_DATA8_REG = 0x06
    ACC_DATA10_REG = 0x00
    ACC_OFF_REG = 0x10

    ACC_MODE_STBY = 0x00                            # Accelerometer standby mode
    ACC_MODE_MEAS = 0x01                            # Accelerometer measurement mode
    ACC_MODE_LVLD = 0x02                            # Accelerometer level detect mode
    ACC_MODE_PLSE = 0x03                            # Accelerometer pulse detect mode

    # ---------------------------------------------------------------------------------------------
    def __init__( self, port, gyro_dps = 250, acc_range = 8 ):
        """! Initialize Class
        @param self  this object
        @param port  mindstorms port
        @param gyro_dps  gyro degrees per second
        """
        self.__gyro = I2CDevice( port, self.GYRO_I2C_ADDR )
        self.__acc = I2CDevice( port, self.ACC_I2C_ADDR )

        self.__valid = True

        # Validate Gyro
        data = self.__gyro.read( self.GYRO_WHOAMI, 1 )

        if ( b'\xd3' != data ):
            self.__valid = False
        else:
            (dps, self.__gyro_divisor) = self.GYRO_DPS[gyro_dps]

            self.__gyro_calib = (0.0, 0.0, 0.0)

            # Write CTRL_REG2
            # No High Pass Filter
            self.__gyro.write( self.GYRO_CTRL_REG2, bytes((0x00,)) )

        	# Write CTRL_REG3
	        # No interrupts.  Date ready.
            self.__gyro.write( self.GYRO_CTRL_REG3, bytes((0x08,)) )

        	# Write CTRL_REG4
	        # Full scale range.
            self.__gyro.write( self.GYRO_CTRL_REG4, bytes((dps + self.CTRL4_BLOCKDATA,)) )

            # Write CTRL_REG5
            self.__gyro.write( self.GYRO_CTRL_REG5, bytes((0x00,)) )

            # Write CTRL_REG1
            # Enable all axes. Disable power down.
            self.__gyro.write( self.GYRO_CTRL_REG1, bytes((0x0F,)) )

        # Validate Accel
        data = self.__acc.read( self.ACC_WHOAMI, 1 )

        if ( b'\x55' != data ):
            self.__valid = False
        else:
            (range, self.__acc_divisor) = self.ACCEL_RANGE[acc_range]

	        # Clear existing offsets
            self.__acc.write( self.ACC_OFF_REG, bytes((0x00, 0x00, 0x00, 0x00, 0x00, 0x00,)) )

	        # Set the Mode Control
            self.__acc.write( self.ACC_MCTRL_REG, bytes((range | self.ACC_MODE_MEAS,)) )

    # ---------------------------------------------------------------------------------------------
    def twos_complement( self, val, len ):
        """! Convert twos compliment to integer
        @param self  this object
        @param val  value
        @param len  length in bits
        @return  twos complement
        """
        if ( ( val & (1 << len - 1) ) != 0 ):
            val = val - (1<<len)
        return val

    # ---------------------------------------------------------------------------------------------
    def __convert_gyro( self, data, offset ):
        """! Convert raw value
        @param self  this object
        @param data  raw readings from device
        @param offset  offset of reading
        @return  converted value
        """
        val = self.twos_complement( data[offset+1] << 8 | data[offset], 16 )
        return round( val / self.__gyro_divisor, 4 )

    # ---------------------------------------------------------------------------------------------
    def gyro_axes( self ):
        """! Retrieve gyro axis data from device
        @param self  this object
        @return  (x, y, z) reading from device
        """
        if ( not self.__valid ):
            return None

        data = self.__gyro.read( self.GYRO_DATA_REG + 0x80, 6 )

        if ( len(data) != 6 ):
            return None

        x = self.__convert_gyro( data, 0 ) + self.__gyro_calib[0]
        y = self.__convert_gyro( data, 2 ) + self.__gyro_calib[1]
        z = self.__convert_gyro( data, 4 ) + self.__gyro_calib[2]

        return ( x, y, z )

    # ---------------------------------------------------------------------------------------------
    def __convert_acc8( self, data, offset ):
        """! Convert raw value
        @param self  this object
        @param data  raw readings from device
        @param offset  offset of reading
        @return  converted value
        """
        val = self.twos_complement( data[offset], 8 )
        return round( val / self.__acc_divisor, 4 )

    # ---------------------------------------------------------------------------------------------
    def __convert_acc10( self, data, offset ):
        """! Convert raw value
        @param self  this object
        @param data  raw readings from device
        @param offset  offset of reading
        @return  converted value
        """
        val = self.twos_complement( 0x3ff & ( data[offset+1] << 8 | data[offset] ), 10 )
        return round( val / 64.0, 4 )

    # ---------------------------------------------------------------------------------------------
    def acc_axes( self, use_10_bits = False ):
        """! Retrieve accel axis data from device
        @param self  this object
        @param use_10_bits  use 10-bit precision if possible
        @return  (x, y, z) reading from device
        """
        if ( not self.__valid ):
            return None

        # 10-bit mode only valid for +/- 8g range
        if (( use_10_bits ) and ( 16.0 == self.__acc_divisor )):
            data = self.__acc.read( self.ACC_DATA10_REG, 6 )

            if ( len(data) != 6 ):
                return None

            x = self.__convert_acc10( data, 0 )
            y = self.__convert_acc10( data, 2 )
            z = self.__convert_acc10( data, 4 )

        else:
            data = self.__acc.read( self.ACC_DATA8_REG, 3 )

            if ( len(data) != 3 ):
                return None

            x = self.__convert_acc8( data, 0 )
            y = self.__convert_acc8( data, 1 )
            z = self.__convert_acc8( data, 2 )

        return ( x, y, z )

    # ---------------------------------------------------------------------------------------------
    def calibrate( self ):
        """! Calibrate sensor
        It is very important the sensor remain motionless during this process
        """
        if ( not self.__valid ):
            return None

        gyro_xoff = 0.0
        gyro_yoff = 0.0
        gyro_zoff = 0.0

        acc_xoff = 0
        acc_yoff = 0
        acc_zoff = 0

        if ( 32.0 == self.__acc_divisor ):
            multiplier = -64.0
        else:
            multiplier = -128.0

        for i in range(32):

            # adjust gyro offset
            (x, y, z) = self.gyro_axes()

            gyro_xoff -= x
            gyro_yoff -= y
            gyro_zoff -= z

            self.__gyro_calib = (gyro_xoff, gyro_yoff, gyro_zoff)

            # adjust accel offset
            (x, y, z) = self.acc_axes( True )

            acc_xoff += int( x * multiplier )
            acc_yoff += int( y * multiplier )
            acc_zoff += int( z * multiplier )

            offsets = bytes((acc_xoff & 0xff, (acc_xoff >> 8) & 0x07, acc_yoff & 0xff, (acc_yoff >> 8) & 0x07, acc_zoff & 0xff, (acc_zoff >> 8) & 0x07,))

            self.__acc.write( self.ACC_OFF_REG, offsets )

            # sleep just a little
            time.sleep( 0.05 )

# =================================================================================================
class dGPS( I2CDevice ):
    """! Dexter Industries EV3 dGPS """
    
    I2C_ADDRESS = 0x03                              # Sensor I2C Address 

    CMD_UTC = 0x00                                  # Fetch UTC 
    CMD_STATUS = 0x01                               # Status of satellite link: 0 no link, 1 link 
    CMD_LAT = 0x02                                  # Fetch Latitude 
    CMD_LONG = 0x04                                 # Fetch Longitude 
    CMD_VELO = 0x06                                 # Fetch velocity in cm/s
    CMD_HEAD = 0x07                                 # Fetch heading in degrees 
    CMD_DIST = 0x08		                            # Fetch distance to destination
    CMD_ANGD = 0x09                                 # Fetch angle to destination 
    CMD_ANGR = 0x0A                                 # Fetch angle travelled since last request
    CMD_SLAT = 0x0B                                 # Set latitude of destination 
    CMD_SLONG = 0x0C                                # Set longitude of destination
    CMD_XFIRM = 0x0D	                            # Extended firmware
    CMD_ALTTD = 0x0E	                            # Altitude
    CMD_HDOP = 0x0F	                                # HDOP
    CMD_VWSAT = 0x10	                            # Satellites in View

    # ---------------------------------------------------------------------------------------------
    def __init__( self, port ):
        """! Initialize Class
        @param self  this object
        @param port  mindstorms port
        """
        super().__init__( port, self.I2C_ADDRESS )

        # extend firmware
        self.write( self.CMD_XFIRM, bytes(( 1, )) )
        data = self.read( self.CMD_XFIRM, 3 )

        self.__firmware = int( data[0] << 16 ) + int( data[1] << 8 ) + int( data[2] )

    # ---------------------------------------------------------------------------------------------
    def utc( self ):
        """! Retrieve UTC Time
        @param self  this object
        @return  utc time
        """
        data = self.read( self.CMD_UTC, 4 )

        val = int( data[0] << 24 ) + int( data[1] << 16 ) + int( data[2] << 8 ) + int( data[3] )
        return val

    # ---------------------------------------------------------------------------------------------
    def location( self ):
        """! Retrieve current location
        @param self  this object
        @return  location as (latitude, longitude)
        """
        latData = self.read( self.CMD_LAT, 4 )
        lat = int( latData[0] << 24 ) + int( latData[1] << 16 ) + int( latData[2] << 8 ) + int( latData[3] )

        lonData = self.read( self.CMD_LONG, 4 )
        lon = int( lonData[0] << 24 ) + int( lonData[1] << 16 ) + int( lonData[2] << 8 ) + int( lonData[3] )

        if ( 10 < latData[0] ):         #if the 0th byte >10, then the longitude was negative and use the 2's compliment of the longitude
            lat = (4294967295^lat)+1
            lat = (-float(lat)/1000000)
        else:
            lat = (float(lat)/1000000)

        if ( 10 < lonData[0] ):         #if the 0th byte >10, then the longitude was negative and use the 2's compliment of the longitude
            lon = (4294967295^lon)+1
            lon = (-float(lon)/1000000)
        else:
            lon = (float(lon)/1000000)

        return (lat, lon)

    # ---------------------------------------------------------------------------------------------
    def heading( self ):
        """! Retrieve current heading
        @param self  this object
        @return  heading (degrees)
        """
        data = self.read( self.CMD_HEAD, 2 )

        val = int( data[0] << 8 ) + int( data[1] )
        return val

    # ---------------------------------------------------------------------------------------------
    def link( self ):
        """! Check for satellite link
        @param self  this object
        @return  @c True if link exists, @c False otherwise
        """
        data = self.read( self.CMD_STATUS, 1 )

        if ( int( data[0] ) ):
            return True

        return False

    # ---------------------------------------------------------------------------------------------
    def velocity( self ):
        """! Retrieve current velocity
        @param self  this object
        @return  velocity (cm/s)
        """
        data = self.read( self.CMD_VELO, 3 )

        val = int( data[0] << 16 ) + int( data[1] << 8 ) + int( data[2] )
        return val

    # ---------------------------------------------------------------------------------------------
    def altitude( self ):
        """! Retrieve current altitude
        @param self  this object
        @return  altitude (meters)
        """
        data = self.read( self.CMD_ALTTD, 4 )

        val = int( data[0] << 24 ) + int( data[1] << 16 ) + int( data[2] << 8 ) + int( data[3] )
        return val

    # ---------------------------------------------------------------------------------------------
    def firmwareVersion( self ):
        """! Retrieve firmware version
        @param self  this object
        @return  firwmare version
        """
        return self.__firmware

    # ---------------------------------------------------------------------------------------------
    def hdop( self ):
        """! Retrieve horizontal dilution of precision
        @param self  this object
        @return  hdop
        """
        data = self.read( self.CMD_HDOP, 4 )

        val = int( data[0] << 24 ) + int( data[1] << 16 ) + int( data[2] << 8 ) + int( data[3] )
        return val

    # ---------------------------------------------------------------------------------------------
    def satellitesInView( self ):
        """! Retrieve number of satellites in view
        @param self  this object
        @return  number of satellites in view
        """
        data = self.read( self.CMD_VWSAT, 4 )

        val = int( data[0] << 24 ) + int( data[1] << 16 ) + int( data[2] << 8 ) + int( data[3] )
        return val
