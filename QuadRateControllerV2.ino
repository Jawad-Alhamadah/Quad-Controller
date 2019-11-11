//Bluee whit red
//this beset Version 2

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
int count=0;
double  pitch_Offset; 
double  roll_Offset;

bool High_ESC_setting=false;
bool Low_ESC_setting=false;
bool SecondHigh_ESC_setting=false;
int Motor1Analog_1=3;
int Motor1Analog_2=9;;
int Motor1Analog_3=10;
int Motor1Analog_4=11;

//===== Const ======================
const byte Size=10;// thee size of the buffer that the carries the meessage.
const int LedPin=8;
//const int LedPin2=3;
const int LedPin2=4;
//const int LedPin4=4;
float int_errorGain_Pitch;
float int_errorGain_Roll;
 float int_errorGain_Yaw;

float RCx=0.0;
float RCy=0.0;
float RCz=0.0;
//====== Motor Const ================

const int Motor1ESC=3;//5;//pins connected to the motors.
const int Motor2ESC=9;;//8;
const int Motor3ESC=10;//7;
const int Motor4ESC=11;//6;

//====== MessageVariables =========

char RecievedMessage[Size];
char Buff[Size];
int MessagePosition=0;//a variable that is used to determins the position of the char that we will store in the message arrsy
Servo ESC;
Servo ESC2;
Servo ESC3;
Servo ESC4;

//============ Speed of Each Motor after applying PID ==========

double MotorPower_Throttle=0;                 //Motor Power Scales with the Pot Reading from the Controller and is multiplied by the Scaler

double Motor_1_Speed=0;
double Motor_2_Speed=0;
double Motor_3_Speed=0;
double Motor_4_Speed=0;
double MinMicroSeconds_Motor=1000000;    //Max MicroSeconds write for the Servo.
double MaxMicroSeconds_Motor=2000000;     //Min Micro Seconds Write for the Servo.


//============== Initial , Desired,Current, Integral Compunent, Derivative Compunent,Error , Input For all Angles:  yaw,pitch,roll =======

float Initial_Yaw,Initial_Pitch,Initial_Roll=0;   //Ini Val for the Angles.Its read initially and is used as 0,0. We do this to Avoid balancing the Quad to the 0,0 of the MPU module because the Mpu wont be installed to be 100% level. there will be slight initial tilts.
float Desired_Yaw,Desired_Pitch,Desired_Roll=0;   // The Desired Angles at any given moment. initially they will equal the inital values because we want  the quad fly up with no tilts.
float Integral_Yaw,Integral_Pitch,Integral_Roll,Integral_Altitude=0; 
float Derivative_Yaw,Derivative_Pitch,Derivative_Roll=0;
float Input_Yaw,Input_Pitch,Input_Roll,Input_Altitude=0;    //the result of applying the PID to yaw,pitch and roll;

float Error_Yaw,Error_Pitch,Error_Roll=0;    // error between the current Angles and Previous angles.
float Prev_Error_Yaw,Prev_Error_Pitch,Prev_Error_Roll,Prev_Error_Altitude=0;



//float Desired_Yaw,Desired_Pitch,Desired_Roll=0;   // The Desired Angles at any given moment. initially they will equal the inital values because we want  the quad fly up with no tilts.

float Integral_Yaw_A,Integral_Pitch_A,Integral_Roll_A,Integral_Altitude_A=0; 
float Derivative_Yaw_A,Derivative_Pitch_A,Derivative_Roll_A=0;
float Input_Yaw_A,Input_Pitch_A,Input_Roll_A,Input_Altitude_A=0;    //the result of applying the PID to yaw,pitch and roll;

float Error_Yaw_A,Error_Pitch_A,Error_Roll_A=0;    // error between the current Angles and Previous angles.
float Prev_Error_Yaw_A,Prev_Error_Pitch_A,Prev_Error_Roll_A,Prev_Error_Altitude_A=0;

float Current_Altitude=0;
float CurrentReading_Yaw; // current values coming from the Chip.
float CurrentReading_Roll;
float CurrentReading_Pitch;
float CurrentPidTime;


//============= Signal Messages ==========

char Signal[5]="200-";
char Signal2[5]="300-";

// =========== Message Handeling Bools =====

bool SignalEnd=false;// bool to check if thee message ended
int MessTime=0;//timer.
int interval=9000;// interval for the timer. this is used because the few seconds at the begining, the chip isnt accurate. it gets better readings with a little bit of time.
bool ReadInitialAccValues=true;
int Data;

//============ PID vars====================
double MotorPower_Scaler=4000;

double maxInput=250;
double minInput=-250;

double maxInpu_Yaw=350;
double minInput_Yaw=-350;

float Porpotional_gain=0.095;    //best value so far
float Integral_gain=0.16399;     //best value so far
float Derivative_gain=0.606;   //best value so far

//float Porpotional_gain=0.095  ;      //0.121  //0.11044 //0.1071 //0.025//0.025 //0.02; //0.02 //0.1  //0.02 //0.04 //0.041; //0.051   /0.1  //0.1  //0.104  //0.104    //0.28
//float Integral_gain=0.16399;       //0.0582 //0.06234  //0.11  /0.11  //0.11  //0.18; //0.18 //0.6  //0.055//0.056 //0.0576 //0.1   /0.14  //0.145  //0.141  //0.15
//float Derivative_gain=0.606;    //0.213  //0.237545  //0.35 //0.38  //0.48  /0.8;  //1.7  //6    //0.48  //0.52  /0.52  //0.52  /0.52  //0.6 //0.58  //0.62





float Porpotional_gain_A=1; //0.05  /0.008    //0.02  //0.04
float Integral_gain_A=0.0005;//0.1    // 0.06  //0.002
float Derivative_gain_A=0.05; //0.41  // 0..4   //0.004  //0.0008

float Porpotional_gain_y=0.4; 
float Integral_gain_y=0.0;
float Derivative_gain_y=0; 


//float Velocity_Const=133.0;
float Velocity_Const=66.0;
//float Velocity_Const=33.0;
//float Velocity_Const=16.0;

uint32_t LastPidTime;
uint32_t PidTime;
uint32_t TimeError;


unsigned long HoldTime2=0;
unsigned long interval2=4;
unsigned long  MessTime2=0;;

unsigned long HoldTime3=0;
unsigned long interval3=4;
unsigned long  MessTime3=0;;
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
double gx_s,gy_s,gz_s;
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
int GyroCounter=0;;


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
       // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
   // Serial.begin(38400);
   //  Serial.begin(9600);
    
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
    mpu.setXGyroOffset(177);
    mpu.setYGyroOffset(-40);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(550); // 1688 factory default for my test chip
mpu.setXAccelOffset(771);
mpu.setYAccelOffset(3996);
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


  pinMode(LedPin,OUTPUT); 
  pinMode(LedPin2,OUTPUT);


   pinMode(Motor1ESC,OUTPUT); 
  pinMode(Motor2ESC,OUTPUT);
  pinMode(Motor3ESC,OUTPUT);
    pinMode(Motor4ESC,OUTPUT);
    
  digitalWrite(LedPin,LOW);
  digitalWrite(LedPin2,LOW);


 // ESC.attach(Motor1ESC , 1000 , 2000); //3
 //ESC2.attach(Motor2ESC , 1000 , 2000);//
 //ESC3.attach(Motor3ESC , 1000 , 2000);
 // ESC4.attach(Motor4ESC , 1000 , 2000);
ReadInitialAccValues=true;

  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
//    accelgyro.getAcceleration(&ax, &ay, &az);
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_20);
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
         //Serial.print("ypr\t");
         //Serial.print(ypr[0] * 180/M_PI);
         //Serial.print("\t");
        //  Serial.print(ypr[1] * 180/M_PI);
       //Serial.print("\t");
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
      //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
  // accelgyro.setDLPFMode(MPU6050_DLPF_BW_20);
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
  ReadMessage();

 /*
  delay(5000);
  ESC.writeMicroseconds(2000);
   ESC2.writeMicroseconds(2000); 
  ESC3.writeMicroseconds(2000); 
   ESC4.writeMicroseconds(2000);  
delay(5000);
  ESC.writeMicroseconds(1000);
   ESC2.writeMicroseconds(1000); 
  ESC3.writeMicroseconds(1000); 
   ESC4.writeMicroseconds(1000);  
  */
  //if()
 // if()
 // if() {OnMode=true}
//if(OnMode==true){
////////////////////////// MY CODE   ////////////////////////////////////////////////////////////
     MessTime=millis();
   //Read the Incomeing Message.
     HandleMessage(Data);
     HandleMessage_AnglesAndLED(Data);
  
     if (SignalEnd==true) {
  
        SignalEnd=false;
       }
    
    if(ReadInitialAccValues==false){
       SetMotorsSpeed();
      }
  
    if(MessTime>interval && ReadInitialAccValues==true){
    digitalWrite(LedPin,HIGH);
    ReadInitialAccValues=false;
    ReadInitial_Yaw_Pitch_Roll();
     
    }
     


  
  //}
}
  
  ///////////////// This method reads message from the ccontroller one char at a time. 
 void ReadMessage(){
    char character;
   //read a char if you recieve one.
    while(Serial.available()>0){
          SignalEnd=false;            // we recieved a signal.
          character = Serial.read();  //read the char
          if(character=='-' ){ // "-" is the char we determined to be the end signal char. it can be anything we want but we are used "-".
              SignalEnd=true; // meessage ended
              RecievedMessage[MessagePosition]='\0';  //end the array with the \0 . its a needed char in the C language.
              MessagePosition=0;// reset message position
              strncpy(Buff,RecievedMessage,5); // write the message in a buffer.
              Data=atoi(Buff); // change the type of data from A char string to an int.
          }
          else{RecievedMessage[MessagePosition]=character; // save current char in the buffer.
              MessagePosition++;  // move the position counter one step so that we dont overwrite our previously saved char.
          }
    }
}

//========== Set the Motor Speed =============


//this method sets the motor speeds through changing the Variables Motor_x_Speed. all these variabls will change inside the function call "adjust".
void SetMotorsSpeed(){



 //  accelgyro.getRotation(&gx, &gy, &gz);
 accelgyro.getRotation(&gx, &gy, &gz);
gx_s = (gx_s * 0.7) + ((gx / Velocity_Const) * 0.3);
gy_s = (gy_s * 0.7) + ((gy / Velocity_Const) * 0.3);
gz_s = (gz_s * 0.7) + ((gz / Velocity_Const) * 0.3);
 // GyroCounter++;
  
  MessTime2=millis();
  if(MessTime2-HoldTime2>interval2){
  //  gx_s =;
//gy_s /= GyroCounter;
//gz_s /=GyroCounter;
      HoldTime2=MessTime2;
   Adjust();
      }



Motor_1_Speed=map(Motor_1_Speed,0,2000,0,255);
Motor_2_Speed=map(Motor_2_Speed,0,2000,0,255);
Motor_3_Speed=map(Motor_3_Speed,0,2000,0,255);
Motor_4_Speed=map(Motor_4_Speed,0,2000,0,255);

analogWrite(Motor1ESC,Motor_1_Speed);
analogWrite(Motor2ESC,Motor_2_Speed);
analogWrite(Motor3ESC,Motor_3_Speed);
analogWrite(Motor4ESC,Motor_4_Speed);  


//ESC.writeMicroseconds(Motor_1_Speed);
//  ESC2.writeMicroseconds(Motor_2_Speed); 
//ESC3.writeMicroseconds(Motor_3_Speed); 
  //ESC4.writeMicroseconds(Motor_4_Speed);  
  
}


//============ Handle Messages Recieved From From Master========

//This handles the Data after its recieeved. All Data is changed from char array to int. Depeending on the int, we deteermine the appropriate action by the Quad.
void HandleMessage(int RecievedData){
    //this If handles the range that is used for the Motor speed.

    if(RecievedData>=1000 && RecievedData<2000){
        MotorPower_Throttle=RecievedData;
       }
       else if( RecievedData==1000){ 
         Data=1000;
         MotorPower_Throttle=1000;
  // we send the data to the smaller method HandleMesagesLED which.
    SignalEnd=false;
}

}
void HandleMessage_AnglesAndLED(int RecievedData){
  //Deteermine if the Message is within range to turn one of the leds on.
  if(RecievedData>190 && RecievedData<197){
       if(digitalRead(LedPin2)==HIGH){
          digitalWrite(LedPin2,LOW);
        }
       else{digitalWrite(LedPin2,HIGH);}

 //  Serial.print("1 : ");Serial.print(Motor_1_Speed);
  // Serial.print("   2: ");Serial.print(Motor_2_Speed);/
//Serial.print(" 3: ");Serial.print(Motor_3_Speed);
//Serial.print(" 4: ");Serial.println(Motor_4_Speed);

/*
analogWrite(Motor1ESC,0);
analogWrite(Motor2ESC,0);
analogWrite(Motor3ESC,0);
analogWrite(Motor4ESC,0); 
*/



 
//Motor_1_Speed=map(Motor_1_Speed,0,2000,0,255);
///Motor_2_Speed=map(Motor_2_Speed,0,2000,0,255);
//Motor_3_Speed=map(Motor_3_Speed,0,2000,0,255);
//Motor_4_Speed=map(Motor_4_Speed,0,2000,0,255);

analogWrite(Motor1ESC,0);
analogWrite(Motor2ESC,0);
analogWrite(Motor3ESC,0);
analogWrite(Motor4ESC,0);  

 //ESC.writeMicroseconds(1000);
 //  ESC2.writeMicroseconds(1000); 
 // ESC3.writeMicroseconds(1000); 
 //  ESC4.writeMicroseconds(1000);  
delay(200000);
   }


//HoldTime=MessTime;
 if(RecievedData>=400 && RecievedData<500){
            RCx=(RecievedData-400)*1;
       }
   else     if(RecievedData>=500 && RecievedData<600){
            RCx=(RecievedData-500)*-1;
       }
      else  if(RecievedData>=600 && RecievedData<700){
            RCy=(RecievedData-600)*-1;
        }
       else if(RecievedData>=700 && RecievedData<800){
            RCy=(RecievedData-700)*1;
        }
        else if(RecievedData>=800 && RecievedData<900){
            RCy=0.0;
        }
      else if( RecievedData>=900 && RecievedData<1000){
            RCx=0.0;
       }
        else if(RecievedData>=200 && RecievedData<300){
            RCz=(RecievedData-200)*-1;
        }
      else if(RecievedData>=300 && RecievedData<400){
            RCz=(RecievedData-300)*1;
        }
  //     Serial.print(" roll : ");Serial.print(RCx);
  // Serial.print("   itch: ");Serial.print(RCy);
// Serial.print("   yaw : ");Serial.println(RCz);
      
      

 


     // RCx=map(RCx,1000,2000,-30,30);
      //  RCy=map(RCx,1000,2000,-30,30);
       //  RCz=map(RCx,1000,2000,-50,50);
       
       //else if(RecievedData>=6000 && RecievedData<7000){
      // ''     RCy=(RecievedData-6000);
      //  }
     //  else if(RecievedData>=7000 && RecievedData<8000){
    //        RCy=(RecievedData-7000);
  //      }
       //else if(RecievedData>=8000 && RecievedData<9000){
        //    RCy=0.0;
       // }
      // else if( RecievedData>=9000){
      //      RCx=0.0;
      // }
    //Deteermine if the Message is within range to turn one of the leds on.

   //mpu.resetFIFO();


}



//This method is where we use PID control to adjust the motor Speed to what we think is appropriate.
void Adjust(){
   PidTime=micros();
 // accelgyro.setDLPFMode(MPU6050_DLPF_BW_20);         

       // HoldTime=MessTime;
        

       
   // gx=((gx/66.0));
   // gy=((gy/66.0));
   // gz=((gz/66.0));

   /*   
 
if(gx<minInput){gx=minInput;}
if(gy<minInput){gy=minInput;}
if(gz<minInput){gz=minInput;}


if(gx>maxInput){gx=maxInput;}


if(gy>maxInput){gy=maxInput;}
if(gz>maxInput){gz=maxInput;}
*/
//if(Motor_4_Speed>maxInput){Motor_4_Speed=maxInput;}
     //  Serial.print("x : ");Serial.print(gx);
  /// Serial.print("   y : ");Serial.print(gy);
  //Serial.print("   z : ");Serial.println(gz);
  // Serial.print("   Motor4: ");Serial.println(Motor_4_Speed);
   // we Read the Current time.
    TimeError=(PidTime-LastPidTime)/1000000; // determine an error based on the last time and the current time. we Divide by 1000 because we are using MicroSeconds to control motor speed.
         // Serial.print("Motor1: ");Serial.print(TimeError);
    CurrentReading_Yaw=gz_s;; // we read current Angles.ypr[0] * 180/M_PI;; // we ypr[2] * 180/M_PI;ad current Angles.
    CurrentReading_Roll=gx_s;
    CurrentReading_Pitch=gy_s;




  Desired_Roll=RCx; // the desired_ value are the acc/gyroscope Chip values that we want the quad to try to achive. Because right now we are only trying to get it to fly up, that value is just
    //the iInitial_Pitchial reading of the Acc.
   Desired_Pitch=RCy;
    Desired_Yaw=RCz;




    
    //int_errorGain_Pitch=(Error_Pitch);
    //int_errorGain_Roll=Error_Roll;
    //int_errorGain_Yaw=Error_Yaw;

     MessTime3=millis();
  if(MessTime3-HoldTime3>interval3){
  //  gx_s =;
//gy_s /= GyroCounter;
//gz_s /=GyroCounter;
      HoldTime3=MessTime3;
    Error_Pitch_A=Desired_Pitch-((ypr[1]*(180/M_PI))); // the error is determined by looking at the curreent reading and subttacing the desired value from it.
    Error_Roll_A=Desired_Roll-((ypr[2]*(180/M_PI)));
    Error_Yaw_A=Desired_Yaw-((ypr[0]*(180/M_PI)));
    // Serial.print("   Motor4:  ");Serial.println(ypr[1]*(180/M_PI));
    
    Integral_Pitch_A+=Integral_gain_A*Error_Pitch_A; //this is th integral portion useed for the PID
    Integral_Roll_A+=Integral_gain_A*Error_Roll_A;
    Integral_Yaw_A+=Integral_gain_y*Error_Yaw_A;
    
    Derivative_Roll_A=Derivative_gain_A*((Error_Roll_A-Prev_Error_Roll_A)); // Drevative Portion.
    Derivative_Pitch_A=Derivative_gain_A*((Error_Pitch_A-Prev_Error_Pitch_A));
    Derivative_Yaw_A=Derivative_gain_y*((Error_Yaw_A-Prev_Error_Yaw_A));
    
    Input_Pitch_A=(Porpotional_gain_A*Error_Pitch_A)+Integral_Pitch_A+Derivative_Pitch_A; //The result of suming all Portions together.
    Input_Roll_A=(Porpotional_gain_A*Error_Roll_A)+Integral_Roll_A+Derivative_Roll_A;
    Input_Yaw_A=(Porpotional_gain_y*Error_Yaw_A)+Integral_Yaw_A+Derivative_Yaw_A;
 
        Prev_Error_Pitch_A=Error_Pitch_A; // update values that we neeed to carry for the next loop call.
    Prev_Error_Roll_A=Error_Roll_A;
    Prev_Error_Yaw_A=Error_Yaw_A;
      }

Input_Pitch_A=Input_Pitch_A*-1;
    Error_Pitch=Desired_Pitch-CurrentReading_Pitch; // the error is determined by looking at the curreent reading and subttacing the desired value from it.
    Error_Roll=Desired_Roll-CurrentReading_Roll;
    Error_Yaw=Desired_Yaw-CurrentReading_Yaw;
    
    Integral_Pitch+=Integral_gain*Error_Pitch; 
    Integral_Roll+=Integral_gain*Error_Roll;
    Integral_Yaw+=Integral_gain_y*Error_Yaw;
    
    Derivative_Roll=Derivative_gain*((Error_Roll-Prev_Error_Roll)); // Drevative Portion.
    Derivative_Pitch=Derivative_gain*((Error_Pitch-Prev_Error_Pitch));
    Derivative_Yaw=Derivative_gain_y*((Error_Yaw-Prev_Error_Yaw));
    
    Input_Pitch=(Porpotional_gain*Error_Pitch)+Integral_Pitch+Derivative_Pitch; //The result of suming all Portions together.
    Input_Roll=(Porpotional_gain*Error_Roll)+Integral_Roll+Derivative_Roll;
    Input_Yaw=(Porpotional_gain_y*Error_Yaw)+Integral_Yaw+Derivative_Yaw;

int limit=100;
if(Input_Pitch>limit){Input_Pitch=limit;}
if(Input_Roll>limit){Input_Roll=limit;}
if(Input_Yaw>limit){Input_Yaw=limit;}
if(Input_Pitch<-1*limit){Input_Pitch=-1*limit;}
if(Input_Roll<-1*limit){Input_Roll=-1*limit;}
if(Input_Yaw<-1*limit){Input_Yaw=-1*limit;}

    // / Changing the Speeds of the Motor. MotorPower is coming from the controller, motor_scaler is deteermined manually by us.
    // since we are Using the "" + "" quad configuration then the Motor one is front and motor 2 is Back. and Since moving foward Decreases the Pitch, we subtract and moving backwards increases pitch so we Add.
    //in general, we want the speed to start at the Minimum and increase with the motor power that is increased by the controller + the Inputs which are Adjustments that we made above.

    Motor_1_Speed=(MotorPower_Throttle)+Input_Roll+Input_Pitch-Input_Yaw;//+ 
    Motor_3_Speed=(MotorPower_Throttle)-Input_Roll+Input_Pitch+Input_Yaw;//-
    Motor_2_Speed=(MotorPower_Throttle)-Input_Roll-Input_Pitch-Input_Yaw;//+
    Motor_4_Speed=(MotorPower_Throttle) +Input_Roll-Input_Pitch+Input_Yaw;//-

    int MotorMin=1000
    ;
   int MotorMax=2000;

 //   Motor_1_Speed=map(Motor_1_Speed,1000000,2000000,MotorMin,MotorMax);
  //  Motor_2_Speed=map(Motor_2_Speed,1000000,2000000,MotorMin,MotorMax);
  //  Motor_3_Speed=map(Motor_3_Speed,1000000,2000000,MotorMin,MotorMax);
   // Motor_4_Speed=map(Motor_4_Speed,1000000,2000000,MotorMin,MotorMax);
if(Motor_1_Speed<MotorMin){Motor_1_Speed=MotorMin;}
if(Motor_2_Speed<MotorMin){Motor_2_Speed=MotorMin;}
if(Motor_3_Speed<MotorMin){Motor_3_Speed=MotorMin;}
if(Motor_4_Speed<MotorMin){Motor_4_Speed=MotorMin;}

if(Motor_1_Speed>MotorMax){Motor_1_Speed=MotorMax;}
if(Motor_2_Speed>MotorMax){Motor_2_Speed=MotorMax;}
if(Motor_3_Speed>MotorMax){Motor_3_Speed=MotorMax;}
if(Motor_4_Speed>MotorMax){Motor_4_Speed=MotorMax;}



            //          Serial.print("RX ");Serial.println(Motor_1_Speed);

 //         Serial.print("Motor1: ");Serial.print(RCx);
 //Serial.print("   Motor2: ");Serial.print(RCy);
 // Serial.print("   Motor3: ");Serial.print(RCz);

 //   analogWrite(Motor1Analog_1,Motor_1_Speed);
   //analogWrite(Motor1Analog_2,Motor_2_Speed);
  // analogWrite(Motor1Analog_3,Motor_3_Speed);
  //  analogWrite(Motor1Analog_4,Motor_4_Speed);
 //delayMicroseconds(75);


//  Serial.print("1 : ");Serial.print(Motor_1_Speed);
 ///  Serial.print("   2: ");Serial.print(Motor_2_Speed);
//Serial.print(" 3: ");Serial.print(Motor_3_Speed);
//Serial.print(" 4: ");Serial.println(Motor_4_Speed);







      
   //Serial.print("Roll:   ");Serial.print(Input_Roll);
 // Serial.print("  ppitch: ");Serial.println(Input_Pitch);

       

//minInput=960+(MotorPower*MotorPower_Scaler);
//maxInput=1080+(MotorPower*MotorPower_Scaler);

 

       
    Prev_Error_Pitch=Error_Pitch; // update values that we neeed to carry for the next loop call.
    Prev_Error_Roll=Error_Roll;
    Prev_Error_Yaw=Error_Yaw;
    LastPidTime=PidTime;
      
     

}

//this function reads the initial Acc Values.




void ReadInitial_Yaw_Pitch_Roll(){
    LastPidTime=micros();
   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Initial_Yaw =ypr[0]*(180/M_PI);
    Initial_Roll=ypr[2]*(180/M_PI);
    Initial_Pitch=ypr[1]*(180/M_PI);
}
//If the Errors are high than we switch to a stronger contolls

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














///////////////////////////            SOURCES               ......................................
//  https://en.wikipedia.org/wiki/PID_controller
// https://www.wilselby.com/research/arducopter/modeling/
// https://www.youtube.com/watch?v=hyME1osgr7s
//https://www.youtube.com/watch?v=AN3yxIBAxTA&t=2s
//https://en.wikipedia.org/wiki/Moving_average
//https://www.youtube.com/watch?v=uOQk8SJso6Q&t=604s









  
