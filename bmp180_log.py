#!/usr/bin/env python

# bmp180_log.py
#
# read and log BMP085 or BMP180 pressure and temperature sensor from I2C bus
#
# Author: Andy Paul
#         nurazur@gmail.com
# 

#Version 05.01.2016 : dateiname wird nach ISO Wochennummer berechnet (bisher python style+ ww01 ist ab dem 1. Montag im Jahr)
#Version 25.01.2017 : Entferne auskommentierte Referenzen zu Adafruit code

import sys
import os
import time
import datetime


# meine "ohne Adafruit" Implementierung
import bmp180lib as BMP085
import smbus

i2c_bus = 1
i2c = smbus.SMBus(i2c_bus)
i2c_addr = 0x77
sensor = BMP085.BMP085(i2c, i2c_addr)

#lese Zeit - formatiere fuer Ausgabe in Datei        
os.environ['TZ'] = 'Europe/Paris'
time.tzset()
loctime = time.localtime(time.time())

# alter standard Dateiname
standard_verzeichnis = '../../var/www/logfiles/'
filename = '%sluftdruck_log.csv' % standard_verzeichnis

# herausfinden in welche Datei geschrieben wird (neuer Standard)
# th_week = int ( time.strftime('%W', loctime) ) + 1
th_week= datetime.date.fromtimestamp(time.time()).isocalendar()[1]
if th_week >9:
    filename = '%s%i_%s' % ( standard_verzeichnis, th_week, time.strftime('%Y_bmp180_log.csv', loctime) )
else:
    filename = '%s0%i_%s' % ( standard_verzeichnis, th_week, time.strftime('%Y_bmp180_log.csv', loctime) )
    
#Lese Kommandozeilenparameter
argc = len(sys.argv)
for i in range (argc):
    if sys.argv[i] == '-f' and (i+1) < argc: 
        filename = sys.argv[i+1]    


t = time.strftime('%W,%a,%d.%m.%Y,%H:%M:%S', loctime)

#oeffne log datei
logfile=open(filename,'a')
logfile.write(t)
alt = float(180.0)


#altitude of sensor
my_altitude = 180.0

# read and print temperature
T = sensor.readTemperature()

# read absolute and normalized pressure
p_msl, p_abs = sensor.readSeaLevelPressure(my_altitude)
p_msl=  p_msl / 100.0
p_abs = p_abs / 100.0
md = sensor.mode


print "T= %.1f degC" % (T)
print "Pa = %.2f hPa" % (p_abs)
print 'Pn = %.2f hPa' %  (p_msl)
print "mode = %s" % md

ld = ",%.2f,%.2f,%.1f\n" % (p_msl, p_abs, T)
logfile.write(ld)
logfile.close()