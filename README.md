# Arduino_progs
programs for arduino / stm32

MPU6050_raw :
	Allows to receive raw data from a MPU6050 at 1 Khz using a STM32.
	The MPU is set to work at full scale 4g (accelerometer) and 250 degree/sec (gyroscope)
	you can either receive data in ASCII (to plot using arduino 1.6.11) or receive data on USB ports.
	Once received on USB, the log program will convert data into m/sec^2 for accelerometer and rad/sec for gyroscope. These conversions are done using constant variables with double precision.

trigger_cam_mpu :
