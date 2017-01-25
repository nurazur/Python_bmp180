#!/usr/bin/env python
import sys

# bmp180test.py
#
# test for bmp180lib:  BMP085 pressure and temperature sensor class 
# and compare vs results obtained from Adafruit driver
# Author: Andy Paul
#         nurazur@gmail.com
# 

ADAFRUIT = False
if ADAFRUIT:

    # Can enable debug output by uncommenting:
    #import logging
    #logging.basicConfig(level=logging.DEBUG)

    import Adafruit_BMP.BMP085 as BMP085

    # Default constructor will pick a default I2C bus.
    sensor = BMP085.BMP085(mode=BMP085.BMP085_ULTRAHIGHRES)

    # Optionally you can override the bus number:
    #sensor = BMP085.BMP085(busnum=2)

    # You can also optionally change the BMP085 mode to one of BMP085_ULTRALOWPOWER, 
    # BMP085_STANDARD, BMP085_HIGHRES, or BMP085_ULTRAHIGHRES.  
else:
    import bmp180lib as BMP085
    #import smbus
    # init I2C bus
    # define I2C bus
    # 0: Raspberry Pi Model B Rev 1.0
    # 1: Raspberry Pi Model B Rev 2.0
    #i2c = smbus.SMBus(1)
    # init BMP085 sensor
    #sensor = BMP085.BMP085(i2c, 0x77)
    sensor = BMP085.BMP180(1)

#altitude of sensor
my_altitude = 182.0


if ADAFRUIT:
    p_msl = float(sensor.read_sealevel_pressure(my_altitude))/100
    T = float(sensor.read_temperature())
    p_abs = float(sensor.read_pressure()/100.0)
else:
    # read temperature
    T = sensor.readTemperature()
    # read absolute and normalized pressure
    p_msl, p_abs = sensor.readSeaLevelPressure(my_altitude)
    p_msl = p_msl/100.0
    p_abs = p_abs/100.0
    
    
print "T= %.1f degC" % (T)
print "Pa = %.2f hPa" % (p_abs)
print 'Pn = %.2f hPa' %  (p_msl)
print "mode = %s" % sensor._mode
