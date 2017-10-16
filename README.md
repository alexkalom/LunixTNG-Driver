# LunixTNG-Driver

#Disclaimer
This the result of an assignment given during the course of the os 
lab at ntua. The assignment was based on skeleton code provided by professors.

#Descritpion
The Lunix:TNG Driver is a character device driver for Linux that gives access 
to a set of measurements obtained from a wireless set of sensors. These 
sensors are connected to each other through a wireless mesh, that ends up to a
base station. The base station is connected to a serial port on the pc running
the driver. The measurements are available to the end users, through some device
nodes. For example, the end user, in order to access the temperature measurements
from sensor 0, needs only to read from /dev/lunix0-temp.

#Installation
The installation process of this driver is fairly simple. Just clone this 
repository and build the driver 

$ cd /path/to/repository/
$ make

And then run 
$ sudo ./mod_init.sh
