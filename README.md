# Arduino_progs
===============
programs for arduino / stm32

##Notes : 
In order to use the STM32 with Arduino Software, some manipulations mustbe done.
First, copy the Arduino_STM32 folder in the hardware folder of your Arduino installation directory. (tested with Arduino 1.6.11)
Then run Arduino_STM32/<linux version>/install.sh
**Please read the wiki (https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki) for full details**
Arduino_STM32 folder originialy comes from : 
* https://github.com/rogerclarkmelbourne/Arduino_STM32

### Reason to use STM32 :
the STM32 has a 'real' USB compared to most of Arduino boards. In our application, this serial communication is of real importance since we will receive all the data through here ! We need full speed USB.

##Directories:

###MPU6050_raw :
	Allows to receive raw data from a MPU6050 at 1 Khz using a STM32.
	The MPU is set to work at full scale 4g (accelerometer) and 250 degree/sec (gyroscope)
	you can either receive data in ASCII (to plot using arduino 1.6.11) or receive data on USB ports.
	Once received on USB, the log program will convert data into m/sec^2 for accelerometer and rad/sec for gyroscope. These conversions are done using constant variables with double precision.

###trigger_cam_mpu :
	This program will perform a hardware trigger on an external camera using the STM32 Maple mini.
	Camera image frequency can be changed in the code through thecamera_freq variable (in number of milliseconds between two openning of the camera's shutter).
	The STM32 send the following data :
| 0x47 | MPU Acc | MPU Gyro | Counter | ShutterDownFlag |

Counter is set to 0 each time the camera will be triggered (i.e. at image acquisition frequency).
ShutterDownFlag passes to 1 when shutter is closed, to let us know how long the shutter was kept open. 

The shutter value can also be changed in the Arduino code. (shutter_value variable)


