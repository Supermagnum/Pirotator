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
2X PicoBorg Reverse - Dual 5A Motor Controller ( already supported in the code )

https://www.piborg.org/motor-control-1135/picoborgrev

Or:
Adafruit DC & Stepper Motor Bonnet

https://www.adafruit.com/product/4280

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


Using the BerryGPS-IMU's onboard GPS should permit the raspberry Pi to know its geographic location, that is useful for those who are running mobile setups as that could be used in the calculations of where to point the antennas. It can also detect magnetic north. It will also detect if the mobile setups main mast is not 100% alligned to the "up" or X axis, and can be used to compensate for that if its not perfetcly straight up.
The Adafruit LSM6DS33's IMU can be used to detect the azimuth and elevation of the antennas mounting axle,its movement speed and where it is pointing related to North.

All the part above can communicate using a I2C bus which are supported in the code,in respect to the Borgpi stepper board. Magnetometer and GPS support is apparently not supported in the code.
Neither is "level compensation" .

The suggested features is explained in this picture:

https://github.com/Supermagnum/Pirotator/blob/main/axis.png

-----------------------------------------

It should be possible to supply DC voltages over the Coax cable using:

https://github.com/Supermagnum/Mjollnir

--------

I have made technical drawings fro a sturdy aluminium insert that fits a box made by Fibox.
Its Product nbr: is 5330372
EAN: 6418074005540
http://www.fibox.com/catalog/135/product/862/5330372_ENG3.html

It has mouting holes for bolts that holds 2X Nema23 Stepper Motors 1.1Nm L56mm 3A + RV30 Worm Reducer gearbox.
It is to be made of aluminium, aircraft type. Some TIG or MIG welding, drilling of holes and milling is required.

PDF files for manufacturing,with drawings and assembly is located here:

https://github.com/Supermagnum/Pirotator/tree/main/mechanical-hardware

That also includes the Freecad file used to make the drawings.
The adapters that goes on the axles needs to be drilled as you feel fit, and needs a M5 or M6 bolt to stay put on the axle.




