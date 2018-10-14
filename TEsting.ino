#include <PS2X_lib.h>
#include "configuration.h"
/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        11      
#define PS2_CMD        12  
#define PS2_SEL        10  
#define PS2_CLK        13  

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   true
//#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;
int LY=0;
int RY=0;
int LY1=0;
int RY1=0;
int Speed=0;
void stop_Stop()    //Stop
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,LOW);
}

void init_GPIO()
{
  pinMode(dir1PinL, OUTPUT); 
  pinMode(dir2PinL, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(dir1PinR, OUTPUT);
  pinMode(dir2PinR, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();
}


void go_AdvanceL(int L, int R)
{

  if(L>70){
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL,HIGH);
    //Serial.print("LEFT_DOWN=");
    //Serial.println(L);
  }
  if(L<-70){
    digitalWrite(dir1PinL, HIGH);
    digitalWrite(dir2PinL,LOW);
    //Serial.print("LEFT_UP=");
    //Serial.println(L);
  }
  
  if(R>70){
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR,HIGH);
    //Serial.print("RIGHT_DOWN=");
    //Serial.println(R);
  }
  if(R<-70){
    digitalWrite(dir1PinR, HIGH);
    digitalWrite(dir2PinR,LOW);
    //Serial.print("RIGHT_UP=");
    //Serial.println(R);
  }
  if(ps2x.Button(PSB_R1))Speed=100;else Speed=220;
  analogWrite(speedPinL,Speed);
  analogWrite(speedPinR,Speed);
}




void setup(){
  Serial.begin(9600);
  init_GPIO();
  delay(3);  //added delay to give wireless ps2 module some time to startup, before configuring it
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  //Serial.print(ps2x.Analog(1), HEX);
  
}

void loop() { 
  
  ps2x.read_gamepad(false, vibrate);
  LY=ps2x.Analog(PSS_LY)-128;
  RY=ps2x.Analog(PSS_RY)-128;
  delay(5);
  LY1=ps2x.Analog(PSS_LY)-128;
  RY1=ps2x.Analog(PSS_RY)-128;
  if((ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) && ((abs(LY)>50) || (abs(RY)>50))){
    if(LY==LY1 &&RY==RY1)
    go_AdvanceL(LY,RY);
  }else{
    stop_Stop();
  }
  delay(25);  
}
