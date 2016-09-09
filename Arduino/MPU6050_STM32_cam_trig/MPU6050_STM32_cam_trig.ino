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

#include <HardWire.h>
// Arduino HWire library is required if I2Cdev I2CDEV_ARDUINO_HWire implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
//AD0 low = 0x68 //(default for SparkFun breakout and InvenSense evaluation board)
//AD0 high = 0x69
MPU6050 mpu;

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

#define LED_PIN 33 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define PULSE_PIN 17
#define CAM_TRIGGER_PIN 12

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int i;
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
unsigned char buffC[15]={0};
int next_time=0;
int dt = 1000;
int8_t counter = 0;
const int trigger_freq = 33; //how many milliseconds to wait between 2 trigger of camera. = 33 for ~ 30 Hz
const int shutter_value = 2; // how many milliseconds the shutter of camera should be open ?
bool cam_trigger_flag = false;
int8_t cam_trigger_down_flag = false;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector



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
    pinMode(CAM_TRIGGER_PIN,OUTPUT);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    HWire.begin();

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(250000);
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
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    ///while (Serial.available() && Serial.read()); // empty buffer again

    
    for(i=0;i<5;i++) {digitalWrite(LED_PIN, HIGH);delay(10);digitalWrite(LED_PIN, LOW);delay(300);}

    // Set Accel Full Scale to 4g
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    digitalWrite(CAM_TRIGGER_PIN, LOW);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    // reset interrupt flag and get INT_STATUS byte
    //mpuInterrupt = false;
    //mpuIntStatus = mpu.getIntStatus();
    
    unsigned char flag_send_raw = 0;
    unsigned char flag_write_serial = 1;
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
    
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    //double control to avoid digitalWrite at each step ! only use digitalWrite when necessary
    if(cam_trigger_flag == true && counter == 0){
      digitalWrite(CAM_TRIGGER_PIN, HIGH);
    }
    else if(cam_trigger_flag == false && counter == (shutter_value)){
      digitalWrite(CAM_TRIGGER_PIN, LOW);
      cam_trigger_down_flag = 1;
    }

    if (flag_send_raw)
    {
      Serial.print(AcX);
      Serial.print(","); Serial.print(AcY);
      Serial.print(","); Serial.print(AcZ);
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
      buffC[13]= counter;
      buffC[14] = cam_trigger_down_flag;

      Serial.write(buffC,15);
    }

    // reset counters, booleans, ...
    counter++;
    if(counter == shutter_value)
      cam_trigger_flag = false;
    else if(counter == trigger_freq){
      cam_trigger_flag = true;
      counter = 0;
    }
    if(cam_trigger_down_flag == 1)
      cam_trigger_down_flag = 0;

    blinkState = !blinkState;
    digitalWrite(PULSE_PIN, blinkState);
    
    // blink LED to indicate activity
    digitalWrite(LED_PIN, (millis()%1000)<10);
    while(micros()<next_time) delayMicroseconds(1); //1khz
    next_time +=dt;
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

