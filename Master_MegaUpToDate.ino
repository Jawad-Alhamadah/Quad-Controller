#include <Servo.h>

// Vars for an Equation called the "exponitialy moving average". it evens out the Pot readings.
int EMA_S=analogRead(A5);
float EMA_Const=0.1;
int EMA_PotValue=0;
int Prev_EMA=EMA_S;

//===== Const ======================
const int LedPin=5;
const int LedPin2=4;
const int RedButtonPin=12;
const int GreenButtonPin=7;
const int Anlg4=A4;
const int Anlg5=A5;
const byte Size=5;
char Signal[4]="200-";
char Signal2[4]="300-";
const int Analg6=A6;
const int Analg7=A7;



const int JoyStickAnlg_Y=A1;
const int JoyStickAnlg_X=A2;
//======= Potentiometer Variables ==========
int Xval;
int Yval;

int CurrentJoyStick_X=0;
int CurrentJoyStick_Y=0;
int CurrentPotReadingState=0;
char state3[4];
int MessagePosition=0;
String content = "";
int RedButtonBool=false;
int GreenButtonBool=false;
int Breaktimer=0;
int RepeatedMesssageCounter=0;
int JoyStick_RepeatedMesssageCounter=0;
int JoyStickAnlgValue_X=0;
int JoyStickAnlgValue_Y=0;
//========= Message Variables ============

char RecievedMessage[Size];
char Buff[Size];
char  JoyStick_Buff[Size];
bool SignalEnd=false;
int MotorPowerPotValue=0;
//int PotSum=0;
int CurrentPower=0;
int Data;

//======== Timer Variables =============
int HoldTime=0;
int interval=150;
long  MessTime=0;;

double LoopSpeed=100;
int LedLoop=0;

int LedPin22=22;
int LedPin24=24;
int LedPin26=26;
int LedPin28=28;


void setup() {
//MotorPowerPotValue = analogRead(Anlg5);
  pinMode(LedPin,OUTPUT);
  pinMode(LedPin2,OUTPUT);
    pinMode(LedPin22,OUTPUT);
  pinMode(LedPin24,OUTPUT);
    pinMode(LedPin26,OUTPUT);
  pinMode(LedPin28,OUTPUT);
  pinMode(RedButtonPin,INPUT);
    pinMode(GreenButtonPin,INPUT);
  Serial.begin(9600);
  digitalWrite(LedPin,LOW);

 EMA_S=analogRead(A5);
}

void loop() {
// we call Send Button Messages everyloop. it checks if the button is pressed so that we send a message.
  SendButtonMessages();
  ReadMessage();
  if (SignalEnd==true) {
     HandleData();
   }
   //this timer is for sending messages. i found that sending message everyloop was too much to handle for the quad so we need every 100 loops or so.
  MessTime=millis();
  if(MessTime-HoldTime>interval){
      HoldTime=MessTime;
   if(Breaktimer==70){delay(15);Breaktimer=0;}
   Breaktimer++;
    SendMotorsSpeed();
    SendQuadDirection();
      }
  LightLed();
 
 // PotFilter();
  }
  


// this is for handeling data that i recieveed. Right now unused since we dont recieve data for the controller But its here just incase we change that.
void HandleData(){
 
   if(Data>190 && Data<280){
       if(digitalRead(LedPin)==HIGH){
         digitalWrite(LedPin,LOW);
        }
       else{digitalWrite(LedPin,HIGH);}
   }
   if(Data>280 && Data<320){
       if(digitalRead(LedPin2)==HIGH){
        digitalWrite(LedPin2,LOW);
       }
       else{digitalWrite(LedPin2,HIGH);}
   }  
  SignalEnd=false;
}
//Reads messages. unseded for the controller but here incase we do want to read a message at some point.
  void ReadMessage(){
      char character;
  while(Serial.available()>0){
      SignalEnd=false;
      character = Serial.read();
      if(character=='-'){
        SignalEnd=true;
        RecievedMessage[MessagePosition]='\0';
        MessagePosition=0;
        Data=atoi(RecievedMessage);
      }
      else{
        RecievedMessage[MessagePosition]=character;
        MessagePosition++;  
      }
  }  
}

void  SendMotorsSpeed(){
      
      PotMovingAve();// call the Averaging method to get smooth Pot reading.
      MotorPowerPotValue=EMA_S;// EMA_S is a global and its changed in the PotMovingAve method.
 
    //this If checks if the Current power is the same as the Pot value we just read. If we the Power is the Same as the Pot value we just read than there is not Need to Send a signal so we just increae
    //a counter. we only send that repeated signal if the Counter has reached 500 just incase. This is just another way to reduced the heavy load on the Quad which gives it room to breath.
      if(CurrentPower==MotorPowerPotValue){
        RepeatedMesssageCounter++;
        if(RepeatedMesssageCounter>500){
                 RepeatedMesssageCounter=0;
          CurrentPower=MotorPowerPotValue;
         //sprintf(Buff, "%3d-", MotorPowerPotValue);
         sprintf(Buff, "%3d-", MotorPowerPotValue); //fill the buffer with the the potValue. what the "%3d-" does is format  the PotValue to a chars that are of length 3 at the maximum 
         //that are Ints and are ending in "-" which is our ending signal.
         Buff[4]='\0';
         Serial.write(Buff,4);// write the buffer into the serial which is how the Bluetooth module detect messages from eachother.
          }
      }
      else{
        RepeatedMesssageCounter=0;
        CurrentPower=MotorPowerPotValue;
       //sprintf(Buff, "%3d-", MotorPowerPotValue);
       sprintf(Buff, "%3d-", MotorPowerPotValue);
       Buff[4]='\0';
       Serial.write(Buff,4);
  
  }

}


// Send the X and Y of the joystick of the controller. 
void  SendQuadDirection(){
    int Current_X_Reading=analogRead(JoyStickAnlg_X);//read the X of the joystick
     int Current_Y_Reading=analogRead(JoyStickAnlg_Y);
    
    int MessageMapValue=0;
Current_X_Reading=MapJoyStick_X(Current_X_Reading); //the Map method returns the mapping of the Joystick
Current_Y_Reading=MapJoyStick_Y(Current_Y_Reading);

    JoyStickAnlgValue_Y=Current_Y_Reading;
  
      JoyStickAnlgValue_X=Current_X_Reading;
 
// the joystick sometimes doesnt return  to its 0,0 poition accurately after releasing it. to avoid reading non zeros when its suppose to be zero,we set 1s to also be zeros.
   if( JoyStickAnlgValue_Y==1||JoyStickAnlgValue_Y==-1){
      JoyStickAnlgValue_Y=0;
    
      }

       if( JoyStickAnlgValue_X==1||JoyStickAnlgValue_X==-1){
      JoyStickAnlgValue_X=0;
    
      }


//because we want to stick to max 3 chars (not counting the ending signal char) we need to map the joystick values from 220, 720 to -9 to 9. that way, we can use the first digit to determine direction.
// and other two digits for X and y . the first digit will tell us which of the other two digits is negative and which is positive. for example:
//lets say we are moving down left, down left means that both x and y are Negative. for that, we use 500. the 5 will be read in the quad as a signal that the values of x and y are negative and the 00 will 
//tell the quad that the position of x is 0 and y is 0.  So 528  means  x=-2 , y=-8.   600 Is used for when both are poitive so 695  means x=9 and y=5. and so on .



   if( JoyStickAnlgValue_X==0  && JoyStickAnlgValue_Y==0){MessageMapValue=400;}//downan left==-
  else  if( JoyStickAnlgValue_X<1 && JoyStickAnlgValue_Y<1){MessageMapValue=500;  }
    else if( JoyStickAnlgValue_X>0  && JoyStickAnlgValue_Y>0){MessageMapValue=600;}
    else if( JoyStickAnlgValue_X>0  && JoyStickAnlgValue_Y<1){MessageMapValue=700;  }
     else  if( JoyStickAnlgValue_X<1  && JoyStickAnlgValue_Y>0){MessageMapValue=800;  }

     //USE ABS 
     //USE ABS
       
        // a check to makee sure we dont send too many reepeats.
        if(CurrentJoyStick_X == JoyStickAnlgValue_X &&  CurrentJoyStick_Y == JoyStickAnlgValue_Y ){
        JoyStick_RepeatedMesssageCounter++;
        if(JoyStick_RepeatedMesssageCounter>150){
                 JoyStick_RepeatedMesssageCounter=0;
        int Message=MessageMapValue+abs(JoyStickAnlgValue_X*10)+abs(JoyStickAnlgValue_Y);// here we change the negative values  to positive with abs. we do this so we can create the number we discussed above.
         //sprintf(Buff, "%3d-", MotorPowerPotValue);
         CurrentJoyStick_X=JoyStickAnlgValue_X;
         CurrentJoyStick_Y=JoyStickAnlgValue_Y;
         sprintf(JoyStick_Buff, "%3d-", Message);
         JoyStick_Buff[4]='\0';
         Serial.write(JoyStick_Buff,4);
        // Serial.println(" ");
          }
      }
      else{
        JoyStick_RepeatedMesssageCounter=0;
         int Message=MessageMapValue+abs(JoyStickAnlgValue_X*10)+abs(JoyStickAnlgValue_Y);
     
        CurrentJoyStick_X=JoyStickAnlgValue_X;
         CurrentJoyStick_Y=JoyStickAnlgValue_Y;
       //sprintf(Buff, "%3d-", MotorPowerPotValue);
       sprintf(JoyStick_Buff, "%3d-", Message);
       JoyStick_Buff[4]='\0';
       Serial.write(JoyStick_Buff,4);

  }

}

   //method to send the Button messages. one click would send a singal signal.
   void SendButtonMessages(){
 
  if(digitalRead(RedButtonPin)==HIGH && RedButtonBool==false){
    Serial.write(Signal,4);
    
    RedButtonBool=true;
  }
  else if(digitalRead(RedButtonPin)==LOW) {RedButtonBool=false; 
  }
  if(digitalRead(GreenButtonPin)==HIGH && GreenButtonBool==false){
    Serial.write(Signal2,4);
    GreenButtonBool=true;
  }
  else if(digitalRead(GreenButtonPin)==LOW){
    GreenButtonBool=false;
  }
 SignalEnd=false;
  
  } 



// this is a small light display in the controller. Not rquired or necissary But its useful for testing and For A nice looking controller.
  void LightLed(){
    if(LedLoop<LoopSpeed*1){
      LedLoop++;
      digitalWrite(LedPin22,HIGH);
      digitalWrite(LedPin24,LOW);
      digitalWrite(LedPin26,LOW);
      digitalWrite(LedPin28,LOW);
      }
      else if(LedLoop<2*LoopSpeed){
           LedLoop++;
              digitalWrite(LedPin22,LOW);
      digitalWrite(LedPin24,HIGH);
      digitalWrite(LedPin26,LOW);
      digitalWrite(LedPin28,LOW);
        }
        else if(LedLoop<3*LoopSpeed){
             LedLoop++;
              digitalWrite(LedPin22,LOW);
      digitalWrite(LedPin24,LOW);
      digitalWrite(LedPin26,HIGH);
      digitalWrite(LedPin28,LOW);
        }
        else if(LedLoop<4*LoopSpeed){
             LedLoop++;
              digitalWrite(LedPin22,LOW);
      digitalWrite(LedPin24,LOW);
      digitalWrite(LedPin26,LOW);
      digitalWrite(LedPin28,HIGH);
        }

         else if(LedLoop<12*LoopSpeed){
             LedLoop++;
              digitalWrite(LedPin22,LOW);
      digitalWrite(LedPin24,LOW);
      digitalWrite(LedPin26,LOW);
      digitalWrite(LedPin28,LOW);
        }
        if(LedLoop+1>12*LoopSpeed){LedLoop=0;}
    }

//maps the Values of joystick
   int MapJoyStick_X(int x){

   
      x= map(x,240,770,-9,9);//x
      return x;
 
  }

  
   int MapJoyStick_Y(int y){

   
      y= map(y,250,775,-9,9);//x
      return y;
 
  }
// method that evens out the signal of the pot value.
  void PotMovingAve(){

EMA_PotValue=analogRead(Anlg5);
EMA_PotValue= EMA_PotValue>>3; // we shift instead of map beecause its faster.
EMA_S=EMA_Const*EMA_PotValue+(1-EMA_Const)*Prev_EMA; /// mathmatical equation from Wikipeedia. source below
Prev_EMA=EMA_S;


      
    }

///// Sources////
//https://www.wilselby.com/research/arducopter/modeling/
//https://www.youtube.com/watch?v=uOQk8SJso6Q&t=604s
//https://www.youtube.com/watch?v=AN3yxIBAxTA&t=2s
//https://en.wikipedia.org/wiki/Moving_average
//https://www.youtube.com/watch?v=hyME1osgr7s
//https://en.wikipedia.org/wiki/PID_controller
