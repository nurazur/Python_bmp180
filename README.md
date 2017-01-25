# Python_bmp180
A Python library and some test scripts for Raspberry Pi for the BMP085 and BMP180 air pressure and temperature sensors. The library does intentially NOT use the Adafruit driver & libraries

bmp180_log.py: 
reads air pressure and temperature and stores results with a time stamp into a csv formatted file.

bmp180lib.py : 
2 classes BMP085 and BMP180. The latter uses fcntl and file-like reading/writing on I2C bus, the first needs a handle to an i2c object, i.e. using smbus module.
The library can be executed to test both classes. Normal use would be to import the library.
The library has been tested with a BMP180 device, BMP085 devices haven't been tested yet but I expect them to work.

bmp180test.py: 
Test for the BMP180 class. Attention! Altitude is hard coded and must be edited by the user before obtaining correct results.  
Results can be compared with results obtained from Adafruit driver, if installed.

bmp180_test_no_ada.py:
Simple demo of the BMP085 class 

