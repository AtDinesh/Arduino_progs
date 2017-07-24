#include <HardWire.h>

//Pin MAP
#define LED_PIN PB1

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2

#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C


HardWire HWire(1, I2C_FAST_MODE); // I2c1 I2C_FAST_MODE
const int MPU_addr=0x68;//0x69;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
unsigned char buffC[14]={0};
int loop_time=0;
int t_tmp = 0;
int i;
int dt = 1000; //(1ms)
int next_time=0;

void setup() {
  
  // initialize digital pin PA14 as an output.
  pinMode(LED_PIN, OUTPUT);
  //pinMode(PA3, OUTPUT);
  for(i=0;i<5;i++) {digitalWrite(LED_PIN, HIGH);delay(10);digitalWrite(LED_PIN, LOW);delay(300);}
  digitalWrite(LED_PIN, LOW);
  Serial.begin(250000);
  delay(1000);
  Serial.println("start!");
  HWire.begin();
  HWire.beginTransmission(MPU_addr);
  HWire.write(0x6B);  // PWR_MGMT_1 register
  HWire.write(0);     // set to zero (wakes up the MPU-6050)
  HWire.endTransmission();

  delay(1000);
  //setup imu
  //setting full scale Accelerometer to 4g
  HWire.beginTransmission(MPU_addr);
  HWire.write(MPU6050_RA_ACCEL_CONFIG);
  HWire.write(MPU6050_ACCEL_FS_4);
  
  //writeBits(MPU_addr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_4);
  
  HWire.endTransmission();
  digitalWrite(LED_PIN, LOW);

  delay(1000);
}


void loop() {
  unsigned char flag_send_raw = 0;
  unsigned char flag_write_serial = 1;
  while(1)
  {
    if (Serial.available())
    {
      unsigned char c = Serial.read();
      switch(c)
      {
        case 'h':
          print_help();
          break;
        case 'r':
          flag_send_raw = 1;
          break;
        case 'R':
          flag_send_raw = 0;
          break;
        case 'w' :
          flag_write_serial = 1;
          break;
        case 'W' :
          flag_write_serial = 0;
      }
    }
 
  //blink RED LED
  digitalWrite(LED_PIN, (millis()%1000)<10);

  MPU_read_RAW();

  if (flag_send_raw)
  {
    Serial.print(AcX);
    Serial.print(","); Serial.print(AcY);
    Serial.print(","); Serial.print(AcZ);
    Serial.print(","); Serial.print(Tmp/340.00+36.53);  //temperature in degrees C
    Serial.print(","); Serial.print(GyX);
    Serial.print(","); Serial.print(GyY);
    Serial.print(","); Serial.print(GyZ);
    Serial.print("\n");
  }

  if (flag_write_serial)
  {
    buffC[0]=0x47;
    buffC[1]=AcX&0xff;
    buffC[2]=(AcX>>8)&0xff;
    buffC[3]=AcY&0xff;
    buffC[4]=(AcY>>8)&0xff;
    buffC[5]=AcZ&0xff;
    buffC[6]=(AcZ>>8)&0xff;
    buffC[7]=GyX&0xff;
    buffC[8]=(GyX>>8)&0xff;
    buffC[9]=GyY&0xff;
    buffC[10]=(GyY>>8)&0xff;
    buffC[11]=GyZ&0xff;
    buffC[12]=(GyZ>>8)&0xff;

    Serial.write(buffC,13);
  }

  while(micros()<next_time) delayMicroseconds(1); //1khz
  }
}


char MPU_read(unsigned char address)
{
  unsigned char data;
  HWire.beginTransmission(MPU_addr);
  HWire.write(address);
  HWire.endTransmission();
  
  HWire.requestFrom(MPU_addr, 1);
  while(!HWire.available())
    ;
  return HWire.read();
}

char MPU_read_RAW()
{
  HWire.beginTransmission(MPU_addr);
  HWire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  HWire.endTransmission();
  HWire.requestFrom(MPU_addr,14);  // request a total of 14 registers
  while(!HWire.available());
  AcX=HWire.read()<<8|HWire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=HWire.read()<<8|HWire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=HWire.read()<<8|HWire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=HWire.read()<<8|HWire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=HWire.read()<<8|HWire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=HWire.read()<<8|HWire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=HWire.read()<<8|HWire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}



void print_help()
{
 Serial.println("Commands are");
 Serial.println(" h: Help ");
 Serial.println(" r: send RAW data at 1Khz (Ax,Ay,Az,Temp,Gx,Gy,Gz)");
 Serial.println(" R: stop sending raw data");
 Serial.println(" w: send raw data at 1KHz (Ax,Ay,Az,Gx,Gy,Gz) through USB");
 Serial.println(" W: stop sending raw data on USB");
}

