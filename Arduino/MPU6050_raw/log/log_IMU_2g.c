
#include <stdlib.h>
#include <stdio.h>
//~ #include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
//~ #include <errno.h>
#include <sys/ioctl.h>
//~ #include <stdint.h>
#include <sys/time.h>

//#define RAW_IMU_LH
//#define NORMALIZED_LH

typedef int bool;
#define true 1
#define false 0

int main(int argc, char *argv[])
{
  bool save_file = true;
  struct timeval tv, t_loop;
  struct timeval start_tv, start_loop;
  int fd,n;
  int i;
  FILE * logFile;
  unsigned char buf[64] = {0};
	double gravity = 9.81;
	double sec_to_rad = 3.14159265359/180.0;
	double accel_LSB = 1.0/16384.0; //2g
	double gyro_LSB = 1.0/131.0; // 250 deg/sec
	double accel_LSB_g = accel_LSB * gravity;
	double gyro_LSB_rad = gyro_LSB * sec_to_rad;
  double Ax, Ay, Az, Gx, Gy, Gz;
  int16_t Axi, Ayi, Azi, Gxi, Gyi, Gzi;

  double current_time;
  double process_time;
  struct termios toptions;
  int time_stop = 10;
  /* open serial port */
  printf("open port...\n");
  fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
  if (fd != -1)
	printf("open ok\n");
  else printf("open() unsuccessful\n");


  tcgetattr(fd, &toptions);

  //cfsetispeed(&toptions, B230400);
  //cfsetospeed(&toptions, B230400);
  cfsetispeed(&toptions, B1000000);
  cfsetospeed(&toptions, B1000000);
  toptions.c_cflag     |= (CLOCAL | CREAD);
  toptions.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
  toptions.c_oflag     &= ~OPOST;
  toptions.c_cc[VMIN]  = 0;
  toptions.c_cc[VTIME] = 10;
  tcsetattr(fd, TCSANOW, &toptions);
 printf("begin acquisition\n");
 if(fd != -1){
 if(save_file) {
    /*open logFile*/
  logFile = fopen(argv[1], "w");
  /*write header*/
  //fprintf(logFile,"Time\t Voltage_A\t Current_A\t Voltage_B\t Current_B\r\n");
  fprintf(logFile,"%%Current_Time\t Acc_x\t Acc_y\t Acc_z\t Gyro_x\t Gyro_y\t Gyro_z\n");
  gettimeofday(&start_tv, NULL);
  while(1)
  {
	gettimeofday(&start_loop, NULL);
    
    do n = read(fd, buf, 1);//READ IT
    while (buf[0]!=0x47);
    n = read(fd, buf, 12);
      
     gettimeofday(&tv, NULL);
     current_time = (tv.tv_sec - start_tv.tv_sec) +  (tv.tv_usec - start_tv.tv_usec) / 1000000.0;
    if (n>3) 
    {
				Ax   = (double)((int16_t)((buf[1]<<8)|buf[0]))*accel_LSB_g;
				Ay   = (double)((int16_t)((buf[3]<<8)|buf[2]))*accel_LSB_g;
				Az   = (double)((int16_t)((buf[5]<<8)|buf[4]))*accel_LSB_g;
				Gx   = (double)((int16_t)((buf[7]<<8)|buf[6]))*gyro_LSB_rad;
				Gy   = (double)((int16_t)((buf[9]<<8)|buf[8]))*gyro_LSB_rad;
				Gz   = (double)((int16_t)((buf[11]<<8)|buf[10]))*gyro_LSB_rad;

		gettimeofday(&t_loop, NULL);
		//process_time = (t_loop.tv_sec - start_loop.tv_sec) +  (t_loop.tv_usec - start_loop.tv_usec) / 1000000.0;
       fprintf(logFile,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", current_time, Ax, Ay, Az, Gx, Gy, Gz);
       //printf("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",current_time,Ax, Ay, Az, Gx, Gy, Gz);
    }
	fflush(stdin);
     if (current_time > time_stop) break;
  }
  close(fd);
  fclose(logFile);
   }
 else {
   gettimeofday(&start_tv, NULL);
  while(1)
  {
      read(fd, buf, 1);
      printf("c=%x %c\n",buf[0],buf[0]);
	if (buf[0]==0x47){
		read(fd, buf, 12);
		#ifdef NORMALIZED_LH
		Ax   = (double)((int16_t)((buf[1]<<8)|buf[0]))*(2.0/32768.0);
       	Ay   = (double)((int16_t)((buf[3]<<8)|buf[2]))*(2.0/32768.0);
       	Az   = (double)((int16_t)((buf[5]<<8)|buf[4]))*(2.0/32768.0);
       	Gx   = (double)((int16_t)((buf[7]<<8)|buf[6]))*(250.0/32768.0);
       	Gy   = (double)((int16_t)((buf[9]<<8)|buf[8]))*(250.0/32768.0);
       	Gz   = (double)((int16_t)((buf[11]<<8)|buf[10]))*(250.0/32768.0);
		printf("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",current_time,Ax, Ay, Az, Gx, Gy, Gz);
		#endif
		#ifdef RAW_IMU_LH
		Axi   = (int16_t)((buf[1]<<8)|buf[0]);
       	Ayi   = (int16_t)((buf[3]<<8)|buf[2]);
       	Azi   = (int16_t)((buf[5]<<8)|buf[4]);
       	Gxi   = (int16_t)((buf[7]<<8)|buf[6]);
       	Gyi   = (int16_t)((buf[9]<<8)|buf[8]);
       	Gzi   = (int16_t)((buf[11]<<8)|buf[10]);
		printf("%lf\t%d\t%d\t%d\t%d\t%d\t%d\n",current_time,Axi, Ayi, Azi, Gxi, Gyi, Gzi);
		#endif
		/*for(i=0; i<11; i++){
		printf("%2x ",buf[i]);		
		}
		printf("\n");*/
	}
      gettimeofday(&tv, NULL);
      current_time = (tv.tv_sec - start_tv.tv_sec) +  (tv.tv_usec - start_tv.tv_usec) / 1000000.0;
      if (current_time > time_stop) break;
  }
  close(fd);
 }
 }
else printf("could not connect to arduino. Leaving the program.\n");

  return 0;  
}
