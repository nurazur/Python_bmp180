#!/usr/bin/env python

# bmp180_test_no_ada.py
#
# test for bmp180lib.py: test BMP085 pressure and temperature sensor class 
# 
# Author: Andy Paul
#         nurazur@gmail.com
# 


import bmp180lib as BMP085
import smbus
import sys

# init I2C bus
# define I2C bus
# 0: Raspberry Pi Model B Rev 1.0
# 1: Raspberry Pi Model B Rev 2.0
i2c_bus = 1
i2c = smbus.SMBus(i2c_bus)

# define I2C address of BMP085 sensor
# 0x77: default address
i2c_addr = 0x77

# init BMP085 sensor
sensor = BMP085.BMP085(i2c, i2c_addr)

#altitude of sensor
my_altitude = 180.0

# read and print temperature
temperature = sensor.readTemperature()
print("T= %.1f degC" % temperature)

# read and print absolute and normalized pressure
pmsl, pabs = sensor.readSeaLevelPressure(my_altitude)

# print result to standard output
print("Pa = %.2f hPa" % (pabs / 100.0))
print("Pn = %.2f hPa" % (pmsl / 100.0)) 
# print in which mode the BMP180 is being used
print "mode = %i" % sensor.mode

'''
#Adafruit compatible code
p0 = float(sensor.read_sealevel_pressure(my_altitude))/100
T = float(sensor.read_temperature())
pa = float(sensor.read_pressure()/100.0)
print "T= %.1f degC" % (T)
print "Pa = %.2f hPa" % (pa)
print 'Pn = %.2f hPa' %  (p0)
print "mode = %s" % sensor._mode
'''
# quit python script
sys.exit(0)