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
float pitch_level_adjust=0;
float roll_level_adjust=0;
int Motor1Analog_1=3;
int Motor1Analog_2=9;;
int Motor1Analog_3=10;
int Motor1Analog_4=11;
float xAngle;
float yAngle;
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

float Porpotional_gain=0.0;    //best value so far // .0.095  //.095  //.095   //.085  /084
float Integral_gain=0.0;     //best value so far  /.16399  //.16399   /.1699   //.184 / .182
float Derivative_gain=0.0;   //best value so far  /0.606 //.68   /.688    //.670    /668

//float Porpotional_gain=0.095  ;      //0.121  //0.11044 //0.1071 //0.025//0.025 //0.02; //0.02 //0.1  //0.02 //0.04 //0.041; //0.051   /0.1  //0.1  //0.104  //0.104    //0.28
//float Integral_gain=0.16399;       //0.0582 //0.06234  //0.11  /0.11  //0.11  //0.18; //0.18 //0.6  //0.055//0.056 //0.0576 //0.1   /0.14  //0.145  //0.141  //0.15
//float Derivative_gain=0.606;    //0.213  //0.237545  //0.35 //0.38  //0.48  /0.8;  //1.7  //6    //0.48  //0.52  /0.52  //0.52  /0.52  //0.6 //0.58  //0.62

float Porpotional_gain_A=0.7326999; //0.72
float Integral_gain_A=0.032; //0.02 
float Derivative_gain_A=17.1; //15.3

float Porpotional_gain_y=1.2;  //1.5;  //0.2
float Integral_gain_y=0.0;
//Elimmminate the I.  try PD contorlleerr

float Derivative_gain_y=0.02; 


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

#include <Wire.h>

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long acc_x_cal, acc_y_cal, acc_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

void setup() {
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(115200);                                               //Use only for debugging
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  delay(6000);
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
                                                 //Set the LCD cursor to position to position 0,1
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0)Serial.print('.' );                         //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;       
    acc_x_cal += acc_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    acc_y_cal += acc_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    acc_z_cal += acc_z;    //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  acc_x_cal /=  2000;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
  acc_y_cal /=  2000;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
  acc_z_cal /=  2000;
 
  loop_timer = micros();       

  pinMode(LedPin,OUTPUT); 
  pinMode(LedPin2,OUTPUT);
  pinMode(Motor1ESC,OUTPUT); 
  pinMode(Motor2ESC,OUTPUT);
  pinMode(Motor3ESC,OUTPUT);
  pinMode(Motor4ESC,OUTPUT);
    
  digitalWrite(LedPin,LOW);
  digitalWrite(LedPin2,LOW);

ReadInitialAccValues=true;

}

void loop() {
     ReadMessage();
     MessTime=millis();
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

      MessTime2=millis();
      if(MessTime2-HoldTime2>interval2){
    
      read_mpu_6050_data();                                              
    
      gyro_x -= gyro_x_cal;                                               
      gyro_y -= gyro_y_cal;  
      acc_x  -= acc_x_cal;
      acc_y  -= acc_y_cal;
      acc_z  -= acc_z_cal;
                                                    

      angle_pitch += gyro_y * 0.0000611;                                    
      angle_roll += gyro_x * 0.0000611;                                      

      angle_roll_acc=atan2(acc_y,acc_z)*57.296;
      angle_pitch_acc=atan2(-1*acc_x,sqrt((acc_y*acc_y)+(acc_z*acc_z)))*57.296;
                                                   
      xAngle = angle_roll * 0.9996 + angle_roll_acc * 0.0004; 
      yAngle = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     

      Adjust();
      Motor_1_Speed=map(Motor_1_Speed,0,2000,0,255);
      Motor_2_Speed=map(Motor_2_Speed,0,2000,0,255);
      Motor_3_Speed=map(Motor_3_Speed,0,2000,0,255);
      Motor_4_Speed=map(Motor_4_Speed,0,2000,0,255);

      analogWrite(Motor1ESC,Motor_1_Speed);
      analogWrite(Motor2ESC,Motor_2_Speed);
      analogWrite(Motor3ESC,Motor_3_Speed);
      analogWrite(Motor4ESC,Motor_4_Speed);  
     }
  
}


//============ Handle Messages Recieved From From Master========

//This handles the Data after its recieeved. All Data is changed from char array to int. Depeending on the int, we deteermine the appropriate action by the Quad.
void HandleMessage(int RecievedData){
    
    if(RecievedData>=1000 && RecievedData<2000){ ////this If handles the range that is used for the Motor speed.
        MotorPower_Throttle=RecievedData;
       }
    else if( RecievedData==1000){ 
       Data=1000;
       MotorPower_Throttle=1000;
       SignalEnd=false;
        }
}
void HandleMessage_AnglesAndLED(int RecievedData){
 
        if(RecievedData>190 && RecievedData<197){  //Deteermine if the Message is within range to turn one of the leds on.
               if(digitalRead(LedPin2)==HIGH){
                  digitalWrite(LedPin2,LOW);
                }
               else{digitalWrite(LedPin2,HIGH);}
        
           analogWrite(Motor1ESC,0);
           analogWrite(Motor2ESC,0);
           analogWrite(Motor3ESC,0);
           analogWrite(Motor4ESC,0);           
           delay(200000);
         }
      // int Base=20;
       int Scale=1;
       if(RecievedData>=400 && RecievedData<500){
           RCx=(RecievedData-400)*Scale;
       //    RCx=map(RCx,-45,45,-1*Base,Base);
        }
        else  if(RecievedData>=500 && RecievedData<600){
            RCx=(RecievedData-500)*(-1*Scale);
      //      RCx=map(RCx,-45,45,-1*Base,Base);
        }
        else  if(RecievedData>=600 && RecievedData<700){
            RCy=(RecievedData-600)*(-1*Scale);
      //      RCy=map(RCy,-45,45,-1*Base,Base);
        }
        else if(RecievedData>=700 && RecievedData<800){
            RCy=(RecievedData-700)*Scale;
        //    RCy=map(RCy,-45,45,-1*Base,Base);
        }
        else if(RecievedData>=800 && RecievedData<900){
            RCy=0.0;
        }
        else if(RecievedData>=900 && RecievedData<1000){
            RCx=0.0;
        }
        else if(RecievedData>=200 && RecievedData<300){
            RCz=(RecievedData-200)*(-1*Scale);
       //     RCz=map(RCz,-45,45,-1*Base,Base);
        }
        else if(RecievedData>=300 && RecievedData<400){
            RCz=(RecievedData-300)*Scale;
     //       RCz=map(RCz,-45,45,-1*Base,Base);
        }
}


void Adjust(){  //This method is where we use PID control to adjust the motor Speed to what we think is appropriate.

    Desired_Roll=RCx; 
    Desired_Pitch=RCy;
    Desired_Yaw=RCz;
      
    Error_Pitch_A=(Desired_Pitch-(yAngle*2))-(yAngle-Initial_Pitch); // the error is determined by looking at the curreent reading and subttacing the desired value from it.
    Error_Roll_A=(Desired_Roll-(xAngle*2))-(xAngle-Initial_Roll);

   // Error_Pitch_A=(Desired_Pitch-(yAngle*1.7))-(yAngle); // the error is determined by looking at the curreent reading and subttacing the desired value from it.
  //  Error_Roll_A=(Desired_Roll-(xAngle*1.7))-(xAngle);
    Integral_Pitch_A+=Integral_gain_A*Error_Pitch_A; //this is th integral portion useed for the PID
    Integral_Roll_A+=Integral_gain_A*Error_Roll_A;

    Derivative_Roll_A=Derivative_gain_A*((Error_Roll_A-Prev_Error_Roll_A)); // Drevative Portion.
    Derivative_Pitch_A=Derivative_gain_A*((Error_Pitch_A-Prev_Error_Pitch_A));

    Input_Pitch_A=(Porpotional_gain_A*Error_Pitch_A)+Integral_Pitch_A+Derivative_Pitch_A; //The result of suming all Portions together.
    Input_Roll_A=(Porpotional_gain_A*Error_Roll_A)+Integral_Roll_A+Derivative_Roll_A;

    Prev_Error_Pitch_A=Error_Pitch_A; // update values that we neeed to carry for the next loop call.
    Prev_Error_Roll_A=Error_Roll_A;

    Error_Yaw=Desired_Yaw-(gyro_z/66);
    Integral_Yaw+=Integral_gain_y*Error_Yaw;
    Derivative_Yaw=Derivative_gain_y*((Error_Yaw-Prev_Error_Yaw));
    Input_Yaw=(Porpotional_gain_y*Error_Yaw)+Integral_Yaw+Derivative_Yaw;
    Input_Roll=Input_Roll_A;
    Input_Pitch=Input_Pitch_A;
    int MotorMin=1000;
    int MotorMax=2000;
    int limit=400;
    if(Input_Pitch>limit){Input_Pitch=limit;}
    if(Input_Roll>limit){Input_Roll=limit;}
    if(Input_Yaw>limit){Input_Yaw=limit;}
    if(Input_Pitch<-1*limit){Input_Pitch=-1*limit;}
    if(Input_Roll<-1*limit){Input_Roll=-1*limit;}
    if(Input_Yaw<-1*limit){Input_Yaw=-1*limit;}
   
    Motor_1_Speed=(MotorPower_Throttle)+Input_Roll+Input_Pitch-Input_Yaw;//+ 
    Motor_3_Speed=(MotorPower_Throttle)-Input_Roll+Input_Pitch+Input_Yaw;//-
    Motor_2_Speed=(MotorPower_Throttle)-Input_Roll-Input_Pitch-Input_Yaw;//+
    Motor_4_Speed=(MotorPower_Throttle)+Input_Roll-Input_Pitch+Input_Yaw;//-

    if(Motor_1_Speed<MotorMin){Motor_1_Speed=MotorMin;}
    if(Motor_2_Speed<MotorMin){Motor_2_Speed=MotorMin;}
    if(Motor_3_Speed<MotorMin){Motor_3_Speed=MotorMin;}
    if(Motor_4_Speed<MotorMin){Motor_4_Speed=MotorMin;}
    
    if(Motor_1_Speed>MotorMax){Motor_1_Speed=MotorMax;}
    if(Motor_2_Speed>MotorMax){Motor_2_Speed=MotorMax;}
    if(Motor_3_Speed>MotorMax){Motor_3_Speed=MotorMax;}
    if(Motor_4_Speed>MotorMax){Motor_4_Speed=MotorMax;}
        
    Prev_Error_Pitch=Error_Pitch; // update values that we neeed to carry for the next loop call.
    Prev_Error_Roll=Error_Roll;
    Prev_Error_Yaw=Error_Yaw;
    LastPidTime=PidTime;
      
}

//this function reads the initial Acc Values.

void ReadInitial_Yaw_Pitch_Roll(){

      read_mpu_6050_data();                                            
    
      gyro_x -= gyro_x_cal;                                           
      gyro_y -= gyro_y_cal;                                            
      acc_x  -= acc_x_cal;
      acc_y  -= acc_y_cal;
      acc_z  -= acc_z_cal;
      
      angle_pitch += gyro_y * 0.0000611;                                   
      angle_roll += gyro_x * 0.0000611;                                   

      angle_roll_acc=atan2(acc_y,acc_z)*57.296;
      angle_pitch_acc=atan2(-1*acc_x,sqrt((acc_y*acc_y)+(acc_z*acc_z)))*57.296;

      xAngle = angle_roll * 0.9996 + angle_roll_acc * 0.0004; 
      yAngle = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     

      Initial_Pitch=yAngle;
      Initial_Roll=xAngle;
      analogWrite(Motor1ESC,0);
      analogWrite(Motor2ESC,0);
      analogWrite(Motor3ESC,0);
      analogWrite(Motor4ESC,0);  
    
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}


void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}


///////////////////////////            SOURCES               ......................................
//  https://en.wikipedia.org/wiki/PID_controller
// https://www.wilselby.com/research/arducopter/modeling/
// https://www.youtube.com/watch?v=hyME1osgr7s
//https://www.youtube.com/watch?v=AN3yxIBAxTA&t=2s
//https://en.wikipedia.org/wiki/Moving_average
//https://www.youtube.com/watch?v=uOQk8SJso6Q&t=604s















  
