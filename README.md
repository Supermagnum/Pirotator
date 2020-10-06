# PiRotator #
This is cloned with permission from:
Ville Jussila (wilho / OH8ETB)

This is the rewritten code for raspberrypi based antenna rotator.
See more of project: https://jkry.org/ouluhack/PiRotator


### How do I get set up? ###

* You should get familiar with the code, most things should be pretty selfexplainig
* fill / modify paramaters in the code (pins,ttys, ip:s, speed, etc)
* Try / Error


### Disclaimer ###

 Ville Jussila (wilho / OH8ETB)
 https://jkry.org/ouluhack/pirotator
 
 Free as a free beer or something.. 
 use and do what ever you want, i will be pleased if you mention me on credits.

----------------------------------------------------------------------------
My personal notes and ideas:

It could use:
1x Stepper Motor HAT for Raspberry Pi WaveshareSKU: 102692 

https://thepihut.com/products/stepper-motor-hat-for-raspberry-pi
----------------------------------------------------------------------------

1x Ozzmaker BerryGPS-IMU V3 

https://ozzmaker.com/product/berrygps-imu/
----------------------------------------------------------------------------

1X Adafruit LSM6DS33 + LIS3MDL - 9 DoF IMU with Accel / Gyro / Mag

https://www.adafruit.com/product/4485
----------------------------------------------------------------------------


1X Adafruit ADS1115 16-Bit ADC - 4 Channel with Programmable Gain Amplifier

https://thepihut.com/products/adafruit-ads1115-16-bit-adc-4-channel-with-programmable-gain-amplifier

----------------------------------------------------------------------------


Using the BerryGPS-IMU's onboard GPS should permit the raspberry Pi to know its geographic location, that is useful for those who are running mobile setups as that could be used in the calculations of where to point the antennas. It can also detect magnetic north. It will also detect if the mobile setups main mast is not 100% alligned tu the "up" or X axis, and can be used to compensate for that if its not perfetcly straight up.
The Adafruit LSM6DS33's IMU can be used to detect the azimuth and elevation of the antennas mounting axle,its movement speed and where it is pointing related to North.

All the part above can communicate using a I2C bus,that and GPS support is apparently not supported in the code.
