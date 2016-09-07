# Arduino_progs
programs for arduino / stm32

MPU6050_raw :
	Allows to receive raw data from a MPU6050 at 1 Khz using a STM32.
	The MPU is set to work at full scale 4g (accelerometer) and 250 degree/sec (gyroscope)
	you can either receive data in ASCII (to plot using arduino 1.6.11) or receive data on USB ports.
	Once received on USB, the log program will convert data into m/sec^2 for accelerometer and rad/sec for gyroscope. These conversions are done using constant variables with double precision.

trigger_cam_mpu :
	This program will perform a hardware trigger on an external camera using the STM32 Maple mini.
	Camera image frequency can be changed in the code through thecamera_freq variable (in number of milliseconds between two openning of the camera's shutter).
	The STM32 send the following data :
| 0x47 | MPU Acc | MPU Gyro | Counter | ShutterDownFlag |

Counter is set to 0 each time the camera will be triggered (i.e. at image acquisition frequency).
ShutterDownFlag passes to 1 when shutter is closed, to let us know how long the shutter was kept open. 

The shuttervalue can also be changed in the Arduino code. (shutter_value variable)
