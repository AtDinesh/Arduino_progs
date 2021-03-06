// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//     2012-06-20 - improved FIFO overflow handling and simplified read process
//     2012-06-19 - completely rearranged DMP initialization code and simplification
//     2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//     2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//     2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                - add 3D math helper file to DMP6 example sketch
//                - add Euler output and Yaw/Pitch/Roll output formats
//     2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//     2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//     2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

//#include <HardWire.h>
// Arduino HWire library is required if I2Cdev I2CDEV_ARDUINO_HWire implementation
// is used in I2Cdev.h
#include "HardWire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
//#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
//AD0 low = 0x68 //(default for SparkFun breakout and InvenSense evaluation board)
//AD0 high = 0x69
MPU6050 mpu1, mpu2;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

HardWire HWire(1, I2C_FAST_MODE); // I2c1 I2C_FAST_MODE
HardWire HWire2(2, I2C_FAST_MODE); // I2c2 I2C_FAST_MODE

//TwoWire Wire(PB6, PB7, SOFT_STANDARD); //defined in I2Cdev.cpp
//TwoWire Wire2(PB10, PB11, SOFT_STANDARD);

#define LED_PIN 33 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define PULSE_PIN 17
bool blinkState = false;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t address = 0x68;
int i;
int16_t AcX1,AcY1,AcZ1,GyX1,GyY1,GyZ1,AcX2,AcY2,AcZ2,GyX2,GyY2,GyZ2;
int16_t AcX_offset, AcY_offset, AcZ_offset, GyX_offset, GyY_offset, GyZ_offset; 
unsigned char buffC[25]={0};
int next_time=0;
int dt = 1000;
uint16_t micros_sample = 0;
uint16_t micros_sample_m = 0;
unsigned char flag_send_raw = 0;
unsigned char flag_write_serial = 1;



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(PULSE_PIN, OUTPUT);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    //HWire.begin();
    HWire.begin();
    HWire2.begin();

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
//    Serial1.print("AT+COND03972BEA7DE");
//    delay(500);
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu1.initialize();
    mpu2.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu1.testConnection() ? F("MPU6050_1 connection successful") : F("MPU6050_1 connection failed"));
    Serial.println(mpu2.testConnection() ? F("MPU6050_2 connection successful") : F("MPU6050_2 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    ///while (Serial.available() && Serial.read()); // empty buffer again

    
    for(i=0;i<5;i++) {digitalWrite(LED_PIN, HIGH);delay(10);digitalWrite(LED_PIN, LOW);delay(300);}

    HWire.beginTransmission(address);
    // Set Accel Full Scale to 4g
    mpu1.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    // Set Gyroscope full scale to 500 deg/sec
    mpu1.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    // Set bandwith to 256 Hz //switch to MPU6050_DLPF_BW_42 for previous configuration
    mpu1.setDLPFMode(MPU6050_DLPF_BW_256);
    
    mpu1.setXAccelOffset(-114);
    mpu1.setYAccelOffset(-166);
    mpu1.setZAccelOffset(1259);
    mpu1.setXGyroOffset(138);
    mpu1.setYGyroOffset(-29);
    mpu1.setZGyroOffset(97);

    HWire.endTransmission();

    HWire2.beginTransmission(address);
    mpu2.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    mpu2.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    mpu2.setDLPFMode(MPU6050_DLPF_BW_256);
    mpu2.setXAccelOffset(-114);
    mpu2.setYAccelOffset(-166);
    mpu2.setZAccelOffset(1259);
    mpu2.setXGyroOffset(138);
    mpu2.setYGyroOffset(-29);
    mpu2.setZGyroOffset(97);
    
    HWire2.endTransmission();

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    // reset interrupt flag and get INT_STATUS byte
    //mpuInterrupt = false;
    //mpuIntStatus = mpu.getIntStatus();
   
    while(1){
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
    
    //blink RED LED
    //digitalWrite(LED_PIN, (millis()%1000)<10);
    digitalWrite(LED_PIN, (millis()%1000)<10);
    digitalWrite(PULSE_PIN, blinkState);
    blinkState = !blinkState;
    HWire.beginTransmission(address);
    HWire2.beginTransmission(address);
    
    micros_sample = micros();
    mpu1.getMotion6(&AcX1, &AcY1, &AcZ1, &GyX1, &GyY1, &GyZ1);
    
    micros_sample_m = micros();
    mpu2.getMotion6(&AcX2, &AcY2, &AcZ2, &GyX2, &GyY2, &GyZ2);
    
    HWire.endTransmission();
    HWire2.endTransmission();
    if (flag_send_raw)
  {
    Serial.print(AcX1);
    Serial.print(","); Serial.print(AcY1);
    Serial.print(","); Serial.print(AcZ1);
    Serial.print(","); Serial.print(GyX1);
    Serial.print(","); Serial.print(GyY1);
    Serial.print(","); Serial.print(GyZ1);
    Serial.print("\n");

    Serial.print(AcX2);
    Serial.print(","); Serial.print(AcY2);
    Serial.print(","); Serial.print(AcZ2);
    Serial.print(","); Serial.print(GyX2);
    Serial.print(","); Serial.print(GyY2);
    Serial.print(","); Serial.print(GyZ2);
    Serial.print("\n");
  }
  
  if (flag_write_serial)
  {
    buffC[0]=0x47;
    buffC[1]=AcX1&0xff;
    buffC[2]=(AcX1>>8)&0xff;
    buffC[3]=AcY1&0xff;
    buffC[4]=(AcY1>>8)&0xff;
    buffC[5]=AcZ1&0xff;
    buffC[6]=(AcZ1>>8)&0xff;
    buffC[7]=GyX1&0xff;
    buffC[8]=(GyX1>>8)&0xff;
    buffC[9]=GyY1&0xff;
    buffC[10]=(GyY1>>8)&0xff;
    buffC[11]=GyZ1&0xff;
    buffC[12]=(GyZ1>>8)&0xff;

    buffC[13]=AcX1&0xff;
    buffC[14]=(AcX1>>8)&0xff;
    buffC[15]=AcY1&0xff;
    buffC[16]=(AcY1>>8)&0xff;
    buffC[17]=AcZ1&0xff;
    buffC[18]=(AcZ1>>8)&0xff;
    buffC[19]=GyX1&0xff;
    buffC[20]=(GyX1>>8)&0xff;
    buffC[21]=GyY1&0xff;
    buffC[22]=(GyY1>>8)&0xff;
    buffC[23]=GyZ1&0xff;
    buffC[24]=(GyZ1>>8)&0xff;

    //buffC[25]=micros_sample&0xff;
    //buffC[26]=(micros_sample>>8)&0xff;

    Serial.write(buffC,25);
  }

  digitalWrite(PULSE_PIN, blinkState);
  blinkState = !blinkState;

  if ((!flag_write_serial) && (!flag_send_raw)){
    Serial.print("ts 1 : "); Serial.print(micros_sample);
    Serial.print(", \t ts 2 : "); Serial.print(micros_sample_m);
    Serial.print("\n");
  }

    // blink LED to indicate activity
    digitalWrite(LED_PIN, (millis()%1000)<10);
    while(micros()<next_time) delayMicroseconds(1); //1khz
    next_time +=dt;
    blinkState = !blinkState;
    digitalWrite(PULSE_PIN, blinkState);
  }
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

