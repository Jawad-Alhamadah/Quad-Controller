//Quad Weight in Grams : 156
//Quad thrust Per Pot Value increase:  
//5=1 157
//7=2 -158
//8-9=3 -160
//10 -161
//11 162 -    4
//13 164  -   6
//14 166  -   8
//15 168 -   10
//20 177     21
//24 182  -  26
//26 187  -  31
//33 197  -  41

#include <Servo.h>


//===== Const ======================
const byte Size=10;
const int LedPin=4;
const int LedPin2=3;
const int LedPin3=12;
//const int LedPin4=4;


//====== Motor Const ================

const int Motor1ESC=5;
const int Motor2ESC=6;
const int Motor3ESC=7;
const int Motor4ESC=8;

//====== MessageVariables =========

char RecievedMessage[Size];
char Buff[Size];
int MessagePosition=0;
Servo ESC;
Servo ESC2;
Servo ESC3;
Servo ESC4;

//============ Speed of Each Motor after applying PID ==========

double MotorPower=0;                 //Motor Power Scales with the Pot Reading from the Controller and is multiplied by the Scaler
double MotorPower_Scaler=3;
double Motor_1_Speed=0;
double Motor_2_Speed=0;
double Motor_3_Speed=0;
double Motor_4_Speed=0;
double MinMicroSeconds_Motor=1000;    //Max MicroSeconds write for the Servo.
double MaxicroSeconds_Motor=2000;     //Min Micro Seconds Write for the Servo.

//============== Initial , Desired,Current, Integral Compunent, Derivative Compunent,Error , Input For all Angles:  yaw,pitch,roll =======

float Initial_Yaw,Initial_Pitch,Initial_Roll=0;   //Ini Val for the Angles.Its read initially and is used as 0,0. We do this to Avoid balancing the Quad to the 0,0 of the MPU module because the Mpu wont be installed to be 100% level. there will be slight initial tilts.
float Desired_Yaw,Desired_Pitch,Desired_Roll=0;   // The Desired Angles at any given moment. initially they will equal the inital values because we want  the quad fly up with no tilts.
float Integral_Yaw,Integral_Pitch,Integral_Roll,Integral_Altitude=0; 
float Derivative_Yaw,Derivative_Pitch,Derivative_Roll=0;
float Input_Yaw,Input_Pitch,Input_Roll,Input_Altitude=0;    //the result of applying the PID to yaw,pitch and roll;

float Error_Yaw,Error_Pitch,Error_Roll=0;    // error between the current Angles and Previous angles.
float Prev_Error_Yaw,Prev_Error_Pitch,Prev_Error_Roll,Prev_Error_Altitude=0;

float Current_Altitude=0;
float CurrentReading_Yaw;
float CurrentReading_Roll;
float CurrentReading_Pitch;
float CurrentPidTime;


//============= Signal Messages ==========

char Signal[4]="200-";
char Signal2[4]="300-";

// =========== Message Handeling Bools =====

bool SignalEnd=false;
int MessTime=0;
int interval=27000;
bool ReadInitialAccValues=true;
int Data;

//============ PID vars====================

float Porpotional_gain=2;
float Integral_gain=0.005;
float Derivative_gain=1.1;
float LastPidTime;
float PidTime;
float TimeError;
//

////////////////////////////////////////////////////////////////////////////////////////////
//=====================================================================================================================================
   //                                                  Code Provided By IMU Chip Creators
//=====================================================================================================================================
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;
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



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



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
//=====================================================================================================================================
 //               Code Provided By IMU Chip Creators
//=====================================================================================================================================


void setup() {
//=====================================================================================================================================
   //             Code Provided By IMU Chip Creators
//=====================================================================================================================================

 // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif





    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    Serial.begin(9600);
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
//Serial.print("j");
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

//=====================================================================================================================================
   //             Code Provided By IMU Chip Creators
//=====================================================================================================================================
//===============================================================================
//=====================================================================================================================================
    //                        My Code
//=====================================================================================================================================

  pinMode(LedPin,OUTPUT); 
  pinMode(LedPin2,OUTPUT);
  pinMode(LedPin3,OUTPUT);
  digitalWrite(LedPin,LOW);
  digitalWrite(LedPin2,LOW);
  digitalWrite(LedPin3,LOW);

  ESC.attach(Motor1ESC , MinMicroSeconds_Motor , MaxicroSeconds_Motor);
  ESC2.attach(Motor2ESC , MinMicroSeconds_Motor , MaxicroSeconds_Motor);
  ESC3.attach(Motor3ESC , MinMicroSeconds_Motor , MaxicroSeconds_Motor);
  ESC4.attach(Motor4ESC , MinMicroSeconds_Motor , MaxicroSeconds_Motor);
ReadInitialAccValues=true;
}

void loop() {
//=====================================================================================================================================
   //             Code Provided By IMU Chip Creators
//=====================================================================================================================================
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }    
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
         //   Serial.print("ypr\t");
        //   Serial.print(ypr[0] * 180/M_PI);
         //   Serial.print("\t");
         //  Serial.print(ypr[1] * 180/M_PI);
           // Serial.print("\t");
         //   Serial.println(ypr[2] * 180/M_PI);


          
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
     //   Serial.print("a/g:\t");
        //Serial.print(ax); Serial.print("\t");
        //Serial.print(ay); Serial.print("\t");
      //  Serial.print(az); Serial.print("\t");
       // Serial.print(gx/131.0); Serial.print("\t");
       // Serial.print(gy/131.0); Serial.print("\t");
     //   Serial.println(gz/131.0);
    #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    mpu.resetFIFO();
//=====================================================================================================================================
    //            Code Provided By IMU Chip Creators
//=====================================================================================================================================

  ReadMessage(); //Read the Incomeing Message.
  if (SignalEnd==true) {
      HandleMessage(Data);
      SignalEnd=false;
     }
  if(ReadInitialAccValues==false){
     SetMotorsSpeed();
    }
  MessTime=millis();
  if(MessTime>interval && ReadInitialAccValues==true){
  digitalWrite(LedPin,HIGH);
  ReadInitialAccValues=false;
  ReadInitial_Yaw_Pitch_Roll();
   
    }
  }
  
  
 void ReadMessage(){
    char character;
    while(Serial.available()>0){
          SignalEnd=false;
          character = Serial.read();
          if(character=='-' ){ // || character==' ' || character=='\0' || character==""
              SignalEnd=true; 
              RecievedMessage[MessagePosition]='\0';
              MessagePosition=0;
              strncpy(Buff,RecievedMessage,4);
              Data=atoi(Buff);
          }
          else{RecievedMessage[MessagePosition]=character;
              MessagePosition++;  
          }
    }
}

//========== Set the Motor Speed =============
void SetMotorsSpeed(){

   Adjust();
   ESC.writeMicroseconds(Motor_1_Speed);
   ESC2.writeMicroseconds(Motor_2_Speed); 
   ESC3.writeMicroseconds(Motor_3_Speed); 
   ESC4.writeMicroseconds(Motor_4_Speed);  
}


//============ Handle Messages Recieved From From Master========
void HandleMessage(int RecievedData){
    if(RecievedData>1 && RecievedData<182){
        MotorPower=RecievedData;
       }
       else if( RecievedData<=1){ 
         Data=0;
         MotorPower=0;
       }
    HandleMessageLEDS(RecievedData);
    SignalEnd=false;
}

void HandleMessageLEDS(int RecievedData){
  if(RecievedData>190 && RecievedData<280){
       if(digitalRead(LedPin2)==HIGH){
          digitalWrite(LedPin2,LOW);
        }
       else{digitalWrite(LedPin2,HIGH);}
   }
  if(RecievedData>280 && RecievedData<320){
       if(digitalRead(LedPin3)==HIGH){
          digitalWrite(LedPin3,LOW);
       }
       else{digitalWrite(LedPin3,HIGH);}
   } 
   //mpu.resetFIFO();
}

void Adjust(){
    PidTime=millis();
    TimeError=(PidTime-LastPidTime)/1000;
    CurrentReading_Yaw=ypr[0] * 180/M_PI;;
    CurrentReading_Roll=ypr[2] * 180/M_PI;
    CurrentReading_Pitch=ypr[1]* 180/M_PI ;
    
    Desired_Roll=Initial_Roll;
    Desired_Pitch=Initial_Pitch;
    
    Error_Pitch=CurrentReading_Pitch-Desired_Pitch;
    Error_Roll=CurrentReading_Roll-Desired_Roll;
    
    Integral_Pitch+=(Integral_gain*(Error_Pitch*TimeError));
    Integral_Roll+=Integral_gain*(Error_Roll*TimeError);
    
    Derivative_Roll=Derivative_gain*((Error_Roll-Prev_Error_Roll)/TimeError);
    Derivative_Pitch=Derivative_gain*((Error_Pitch-Prev_Error_Pitch)/TimeError);
    
    Input_Pitch=(Porpotional_gain*Error_Pitch)+Integral_Pitch+Derivative_Pitch;
    Input_Roll=(Porpotional_gain*Error_Roll)+Integral_Roll+Derivative_Roll;
    
    Motor_1_Speed=((MotorPower*MotorPower_Scaler)+1000)-Input_Pitch;
    Motor_3_Speed=((MotorPower*MotorPower_Scaler)+1000)+Input_Pitch;
    Motor_2_Speed=((MotorPower*MotorPower_Scaler)+1000)-Input_Roll;
    Motor_4_Speed=((MotorPower*MotorPower_Scaler)+1000) +Input_Roll;
     
    Serial.print("Motor1: ");Serial.print(Motor_1_Speed);
    Serial.print("   Motor2: ");Serial.print(Motor_2_Speed);
    Serial.print("   Motor3: ");Serial.print(Motor_3_Speed);
    Serial.print("   Motor4: ");Serial.println(Motor_4_Speed);
       
    Prev_Error_Pitch=Error_Pitch;
    Prev_Error_Roll=Error_Roll;
    LastPidTime=PidTime;
}

void ReadInitial_Yaw_Pitch_Roll(){
    LastPidTime=millis();
    Initial_Yaw =ypr[0]* 180/M_PI;
    Initial_Roll=ypr[2] * 180/M_PI;
    Initial_Pitch=ypr[1] * 180/M_PI;
}
//============ Send Messages to The Master =====================
/*void SendMessage(){
 
  if(digitalRead(ButtonPin)==HIGH && Button1Bool==false){
    Serial.write(Signal,4);
    Button1Bool=true;
  }
  else if(digitalRead(ButtonPin)==LOW) {Button1Bool=false; 
  }
  if(digitalRead(ButtonPin2)==HIGH && Button2Bool==false){
    Serial.write(Signal2,4);
    Button2Bool=true;
  }
  else if(digitalRead(ButtonPin2)==LOW){
    Button2Bool=false;
  }
 SignalEnd=false;
  }*/
