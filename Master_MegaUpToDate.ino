#include <Servo.h>
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

  SendButtonMessages();
  ReadMessage();
  if (SignalEnd==true) {
     HandleData();
   }
   
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
  
    // reads the value of the potentiometer (value between 0 and 1023)
   // scale it to use it with the servo library (value between 0 and 180)
    // Send the signal to the ESC



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
      PotMovingAve();
  
      MotorPowerPotValue=EMA_S;
 
    
      if(CurrentPower==MotorPowerPotValue){
        RepeatedMesssageCounter++;
        if(RepeatedMesssageCounter>500){
                 RepeatedMesssageCounter=0;
          CurrentPower=MotorPowerPotValue;
         //sprintf(Buff, "%3d-", MotorPowerPotValue);
         sprintf(Buff, "%3d-", MotorPowerPotValue);
         Buff[4]='\0';
         Serial.write(Buff,4);
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



void  SendQuadDirection(){
    int Current_X_Reading=analogRead(JoyStickAnlg_X);
     int Current_Y_Reading=analogRead(JoyStickAnlg_Y);
    int MessageMapValue=0;
Current_X_Reading=MapJoyStick_X(Current_X_Reading);
Current_Y_Reading=MapJoyStick_Y(Current_Y_Reading);

    JoyStickAnlgValue_Y=Current_Y_Reading;
  
      JoyStickAnlgValue_X=Current_X_Reading;
 

   if( JoyStickAnlgValue_Y==1||JoyStickAnlgValue_Y==-1){
      JoyStickAnlgValue_Y=0;
    
      }

       if( JoyStickAnlgValue_X==1||JoyStickAnlgValue_X==-1){
      JoyStickAnlgValue_X=0;
    
      }


   if( JoyStickAnlgValue_X==0  && JoyStickAnlgValue_Y==0){MessageMapValue=400;}//downan left==-
  else  if( JoyStickAnlgValue_X<1 && JoyStickAnlgValue_Y<1){MessageMapValue=500;  }
    else if( JoyStickAnlgValue_X>0  && JoyStickAnlgValue_Y>0){MessageMapValue=600;}
    else if( JoyStickAnlgValue_X>0  && JoyStickAnlgValue_Y<1){MessageMapValue=700;  }
     else  if( JoyStickAnlgValue_X<1  && JoyStickAnlgValue_Y>0){MessageMapValue=800;  }

     //USE ABS 
     //USE ABS
        if(CurrentJoyStick_X == JoyStickAnlgValue_X &&  CurrentJoyStick_Y == JoyStickAnlgValue_Y ){
        JoyStick_RepeatedMesssageCounter++;
        if(JoyStick_RepeatedMesssageCounter>150){
                 JoyStick_RepeatedMesssageCounter=0;
        int Message=MessageMapValue+abs(JoyStickAnlgValue_X*10)+abs(JoyStickAnlgValue_Y);
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


   int MapJoyStick_X(int x){

   
      x= map(x,240,770,-9,9);//x
      return x;
 
  }

  
   int MapJoyStick_Y(int y){

   
      y= map(y,250,775,-9,9);//x
      return y;
 
  }

  void PotMovingAve(){

EMA_PotValue=analogRead(Anlg5);
EMA_PotValue= EMA_PotValue>>3;
EMA_S=EMA_Const*EMA_PotValue+(1-EMA_Const)*Prev_EMA;
Prev_EMA=EMA_S;


      
    }
