#!/usr/bin/env python
# -*- coding: utf-8 -*-

# bmp180lib.py
#
# pressure and temperature sensor library 
# class BMP085: supports BMP085 (untested) and BMP180 (tested) using a handle to 
#               i2c bus (i.e from smbus module)
#
# class BMP180: supports BMP180 (tested)using fcntl stream
#
# if invoked directly, tests both classes.
# 
# Author: Andy Paul
#         nurazur@gmail.com
# 
#


# import required modules
import time
import sys
import fcntl

# BMP085 class
# this class gets a handle to i2c
class BMP085():
# initialize BMP085 sensor
    def __init__(self, i2c, address, mode=3):
        self.i2c = i2c
        self.address = address

        # set operating mode
        # 0: ultra low power mode
        # 1: standard mode
        # 2: high resolution mode
        # 3: ultra high resolution mode
        self.mode = mode
        self._mode = mode #Adafruit compatibility

        # Read the calibration data
        self.readCalibrationData()


    # read calibration data from BMP085 sensor
    def readCalibrationData(self):
        self.ac1 = self.readSignedWord(0xAA)
        self.ac2 = self.readSignedWord(0xAC)
        self.ac3 = self.readSignedWord(0xAE)
        self.ac4 = self.readWord(0xB0)
        self.ac5 = self.readWord(0xB2)
        self.ac6 = self.readWord(0xB4)
        self.b1 = self.readSignedWord(0xB6)
        self.b2 = self.readSignedWord(0xB8)
        self.mb = self.readSignedWord(0xBA)
        self.mc = self.readSignedWord(0xBC)
        self.md = self.readSignedWord(0xBE)


    # read word from specified register
    def readWord(self, reg):
     # read most significant byte
     msb = self.i2c.read_byte_data(self.address, reg)

     # read least significant byte
     lsb = self.i2c.read_byte_data(self.address, reg+1)

     # return calculated value
     value = (msb << 8 ) + lsb
     return value


    # read signed word from specified register
    def readSignedWord(self, reg):
     # read most significant byte
     msb = self.i2c.read_byte_data(self.address, reg)

     # read least significant byte
     lsb = self.i2c.read_byte_data(self.address, reg+1)

     # return calculated value
     if (msb > 127):
       msb = msb - 256
     value = (msb << 8 ) + lsb
     return value


    # read uncompensated temperature value
    def readRawTemperature(self):
     # write 0x2E into register 0xF4 to request temperature reading
     self.i2c.write_byte_data(self.address, 0xF4, 0x2E)

     # wait 5ms
     time.sleep(0.005)

     # read two byte result from address 0xF6
     value = self.readWord(0xF6)

     # return uncompensated temperature value
     return value


    # read uncompensated pressure value
    def readRawPreassure(self):
     # write 0x34+(BMP085_OVERSAMPLING_SETTING<<6) into register 0xF4 to request pressure reading with oversampling setting
     self.i2c.write_byte_data(self.address, 0xF4, 0x34 + (self.mode << 6))

     # wait for conversion, delay time depends on oversampling setting
     delay = (2 + (3 << self.mode)) / 1000.0
     time.sleep(delay)

     # read three byte result from address 0xF6 (0xF6 = MSB, 0xF7 = LSB, 0xF8 = XLSB)
     msb = self.i2c.read_byte_data(self.address, 0xF6)
     lsb = self.i2c.read_byte_data(self.address, 0xF7)
     xlsb = self.i2c.read_byte_data(self.address, 0xF8)

     # calculate uncompensated pressure value
     value = ((msb << 16) + (lsb << 8 ) + xlsb) >> (8 - self.mode)
     # print 'Raw pressure 0x{0:04X} ({1})'.format(value & 0xFFFF, value)
     # return uncompensated pressure value
     return value


    # read compensated temperature value in degrees celcius
    def readTemperature(self):
     # read uncompensated temperature value
     ut = self.readRawTemperature()

     # calculate and return compensated temperature value
     x1 = ((ut - self.ac6) * self.ac5) >> 15
     x2 = (self.mc << 11) / (x1 + self.md)
     b5 = x1 + x2
     value = ((b5 + 8 ) >> 4) / 10.0
     return value


    # read compensated pressure value in pascal
    def readPressure(self):
     # read uncompensated temperature value
     ut = self.readRawTemperature()

     # read uncompensated pressure value
     up = self.readRawPreassure()

     # calculate compensated temperature value
     x1 = ((ut - self.ac6) * self.ac5) >> 15
     x2 = (self.mc << 11) / (x1 + self.md)
     b5 = x1 + x2
     temp_compensated_degC = ((b5 + 8 ) >> 4) / 10.0

     # calculate and return compensated pressure value
     b6 = b5 - 4000
     x1 = (self.b2 * (b6 * b6) >> 12) >> 11
     x2 = (self.ac2 * b6) >> 11
     x3 = x1 + x2
     b3 = (((self.ac1 * 4 + x3) << self.mode) + 2) / 4

     x1 = (self.ac3 * b6) >> 13
     x2 = (self.b1 * ((b6 * b6) >> 12)) >> 16
     x3 = ((x1 + x2) + 2) >> 2
     b4 = (self.ac4 * (x3 + 32768)) >> 15
     b7 = (up - b3) * (50000 >> self.mode)

     if (b7 < 0x80000000):
       p = (b7 * 2) / b4
     else:
       p = (b7 / b4) *2

     x1 = (p >> 8) * (p >> 8)
     x1 = (x1 * 3038) >> 16
     x2 = (-7357 * p) >> 16

     value = p + ((x1 + x2 + 3791) >> 4)
     return value


    # read altitude in meters
    def readAltitude(self, seaLevelPressure=101325):
        # read compensated pressure value
        pressure = float(self.readPressure())

        # calculate and return altitude value
        altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903))
        return altitude

    #calculate Sea Level pressure from altitude and absolute pressure
    def calcSeaLevelPressure(self, p_abs=101325, altitude=0):
        p0 = p_abs / pow ((1.0 - altitude/44300.0), 5.255)
        return p0

    #read absolute pressure and calculate normalized (MSL) pressure, return both in [Pa]
    def readSeaLevelPressure(self, altitude):
        pabs_Pa = float(self.readPressure())
        pmsl_Pa = self.calcSeaLevelPressure(pabs_Pa, altitude)
        return pmsl_Pa, pabs_Pa

    # Adafruit compatibility functions
    def read_sealevel_pressure(self, altitude):
        pabs_Pa = float(self.readPressure())
        pmsl_Pa = self.calcSeaLevelPressure(pabs_Pa, altitude)
        return pmsl_Pa
        
    def read_temperature(self):
        return self.readTemperature()
    
    def read_pressure(self):
        return float(self.readPressure())
    
# End Class BMP085        

# Class BMP180 using io streams and fcntl        
class BMP180(): 
    # From: /linux/i2c-dev.h
    I2C_SLAVE = 0x0703
    I2C_SLAVE_FORCE = 0x0706
    
    # initialize BMP085 sensor
    def __init__(self, bus, mode=3):
        self.i2c = open('/dev/i2c-%s' % bus, 'r+', 0)
        fcntl.ioctl(self.i2c, self.I2C_SLAVE, 0x077)

        # set operating mode
        # 0: ultra low power mode
        # 1: standard mode
        # 2: high resolution mode
        # 3: ultra high resolution mode
        self.mode = mode
        self._mode = mode #Adafruit compatibility

        # Read the calibration data
        self.readCalibrationData()


    # read calibration data from BMP085 sensor
    def readCalibrationData(self):
        self.ac1 = self.readSignedWord(0xAA)
        self.ac2 = self.readSignedWord(0xAC)
        self.ac3 = self.readSignedWord(0xAE)
        self.ac4 = self.readWord(0xB0)
        self.ac5 = self.readWord(0xB2)
        self.ac6 = self.readWord(0xB4)
        self.b1 = self.readSignedWord(0xB6)
        self.b2 = self.readSignedWord(0xB8)
        self.mb = self.readSignedWord(0xBA)
        self.mc = self.readSignedWord(0xBC)
        self.md = self.readSignedWord(0xBE)


    # read word from specified register
    def readWord(self, reg):
     # read most significant byte
     self.i2c.write(chr(reg))
     msb = ord(self.i2c.read(1))

     # read least significant byte
     self.i2c.write(chr(reg+1))
     lsb = ord(self.i2c.read(1))

     # return calculated value
     value = (msb << 8 ) + lsb
     return value


    # read signed word from specified register
    def readSignedWord(self, reg):
        
     # read most significant byte
     self.i2c.write(chr(reg))
     msb = ord(self.i2c.read(1))

     # read least significant byte
     self.i2c.write(chr(reg+1))
     lsb = ord(self.i2c.read(1))

     # return calculated value
     if (msb > 127):
       msb = msb - 256
     value = (msb << 8 ) + lsb
     return value


    # read uncompensated temperature value
    def readRawTemperature(self):
     # write 0x2E into register 0xF4 to request temperature reading
     self.i2c.write("%c%c" % (0xF4, 0x2E))

     # wait 5ms
     time.sleep(0.005)

     # read two byte result from address 0xF6
     value = self.readWord(0xF6)

     # return uncompensated temperature value
     return value


    # read uncompensated pressure value
    def readRawPreassure(self):
     # write 0x34+(BMP085_OVERSAMPLING_SETTING<<6) into register 0xF4 to request pressure reading with oversampling setting
     reg = 0xF4
     val = 0x34 + (self.mode << 6)
     self.i2c.write("%c%c"  % ( reg, val ))

     # wait for conversion, delay time depends on oversampling setting
     delay = (2 + (3 << self.mode)) / 1000.0
     time.sleep(delay)

     # read three byte result from address 0xF6 (0xF6 = MSB, 0xF7 = LSB, 0xF8 = XLSB)
     self.i2c.write(chr(0xF6))
     msb = ord(self.i2c.read(1))
     
     self.i2c.write(chr(0xF7))
     lsb = ord(self.i2c.read(1))
     
     self.i2c.write(chr(0xF8))
     xlsb = ord(self.i2c.read(1))
     

     # calculate uncompensated pressure value
     value = ((msb << 16) + (lsb << 8 ) + xlsb) >> (8 - self.mode)
     # print 'Raw pressure 0x{0:04X} ({1})'.format(value & 0xFFFF, value)
     # return uncompensated pressure value
     return value


    # read compensated temperature value in degrees celcius
    def readTemperature(self):
     # read uncompensated temperature value
     ut = self.readRawTemperature()

     # calculate and return compensated temperature value
     x1 = ((ut - self.ac6) * self.ac5) >> 15
     x2 = (self.mc << 11) / (x1 + self.md)
     b5 = x1 + x2
     value = ((b5 + 8 ) >> 4) / 10.0
     return value


    # read compensated pressure value in pascal
    def readPressure(self):
     # read uncompensated temperature value
     ut = self.readRawTemperature()

     # read uncompensated pressure value
     up = self.readRawPreassure()

     # calculate compensated temperature value
     x1 = ((ut - self.ac6) * self.ac5) >> 15
     x2 = (self.mc << 11) / (x1 + self.md)
     b5 = x1 + x2
     temp_compensated_degC = ((b5 + 8 ) >> 4) / 10.0

     # calculate and return compensated pressure value
     b6 = b5 - 4000
     x1 = (self.b2 * (b6 * b6) >> 12) >> 11
     x2 = (self.ac2 * b6) >> 11
     x3 = x1 + x2
     b3 = (((self.ac1 * 4 + x3) << self.mode) + 2) / 4

     x1 = (self.ac3 * b6) >> 13
     x2 = (self.b1 * ((b6 * b6) >> 12)) >> 16
     x3 = ((x1 + x2) + 2) >> 2
     b4 = (self.ac4 * (x3 + 32768)) >> 15
     b7 = (up - b3) * (50000 >> self.mode)

     if (b7 < 0x80000000):
       p = (b7 * 2) / b4
     else:
       p = (b7 / b4) *2

     x1 = (p >> 8) * (p >> 8)
     x1 = (x1 * 3038) >> 16
     x2 = (-7357 * p) >> 16

     value = p + ((x1 + x2 + 3791) >> 4)
     return value


    # read altitude in meters
    def readAltitude(self, seaLevelPressure=101325):
        # read compensated pressure value
        pressure = float(self.readPressure())

        # calculate and return altitude value
        altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903))
        return altitude

    #calculate Sea Level pressure from altitude and absolute pressure
    def calcSeaLevelPressure(self, p_abs=101325, altitude=0):
        p0 = p_abs / pow ((1.0 - altitude/44300.0), 5.255)
        return p0

    #read absolute pressure and calculate normalized (MSL) pressure, return both in [Pa]
    def readSeaLevelPressure(self, altitude):
        pabs_Pa = float(self.readPressure())
        pmsl_Pa = self.calcSeaLevelPressure(pabs_Pa, altitude)
        return pmsl_Pa, pabs_Pa

    # Adafruit compatibility functions
    def read_sealevel_pressure(self, altitude):
        pabs_Pa = float(self.readPressure())
        pmsl_Pa = self.calcSeaLevelPressure(pabs_Pa, altitude)
        return pmsl_Pa
        
    def read_temperature(self):
        return self.readTemperature()
    
    def read_pressure(self):
        return float(self.readPressure())
    
# End Class BMP180

# main function
def main():
    
    import smbus
    # init I2C bus
    i2c = smbus.SMBus(1)

    # init BMP085 sensor
    bmp085 = BMP085(i2c, 0x77)
    
    #bmp085 = BMP180(1)
    
    #altitude of sensor
    my_altitude = 182.0
    
    # read and print temperature
    temperature = bmp085.readTemperature()
    print("T= %.1f degC" % temperature)
    
    # read and print absolute and normalized pressure
    pmsl, pabs = bmp085.readSeaLevelPressure(my_altitude)
    
    # print result to standard output
    print("Pa = %.2f hPa" % (pabs / 100.0))
    print("Pn = %.2f hPa" % (pmsl / 100.0)) 
    
    # print in which mode the BMP180 is being used
    print "mode = %i" % bmp085.mode
    
    bmp180 = BMP180(1)
    # read and print temperature
    temperature = bmp180.readTemperature()
    print("T= %.1f degC" % temperature)
    
    # read and print absolute and normalized pressure
    pmsl, pabs = bmp180.readSeaLevelPressure(my_altitude)
    
    # print result to standard output
    print("Pa = %.2f hPa" % (pabs / 100.0))
    print("Pn = %.2f hPa" % (pmsl / 100.0)) 
    
    
    # quit python script
    sys.exit(0)


if __name__ == '__main__':
    main()