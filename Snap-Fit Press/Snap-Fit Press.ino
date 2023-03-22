//Full step (1.8 degrees)
//Used 1/2 Step
//Uses CNC shield with X-slot
//End-stop connected to X

#include <Wire.h> 

#define MCU1 9 //I2C address (MCU1)
#define SCARA_MCU 8 //I2C address (SCARA)
#define Press 10 //I2C address (This MCU)
#define stepMotor  4 
#define dirMotor  7
#define limitSwitch 11

//list of commands recieved from MCU1
const char doPress = 0x01;
char commandFlag = 0x00; //Set to 0x01 if command is being executed. Is set when command is recieved/before data is recieved
char command = 0x00; //Stores incoming command. Keep at 0x00 if no commands are being executed

union Coordinates{ //Coordinates kept as union for I2C transmission as bytes
   byte array[4];
   double num;
};

union Coordinates grabCord;
int N = 0; //total motor turns from end-stop
int delayTime = 1000; //delay between motor steps (microseconds)
enum States {Start, Wait, PressState} state;

void Tick() {
    switch (state) { //State Transitions
      case Start:
        ZeroOut();
        command = 0x00;
        commandFlag = 0x00;
        state = Wait;
        break;
      case Wait:
        if((command == doPress) && (commandFlag == 0x01)) {
              state = PressState; }
        break;
      case PressState:
        if((command == 0x00) && (commandFlag == 0x00)) {
              state = Wait; }
            else
              state = PressState;
            break;
      default:
        break;
    }
    switch (state) { //State Actions
      case Start:
        break;
      case Wait:
        break;
      case PressState:
        Pressing();
        command = 0x00;
        commandFlag = 0x00;
        break;
      default:
        break;
    }
    
    
}

void setup() {
  pinMode(stepMotor,OUTPUT);  
  pinMode(dirMotor,OUTPUT);
  pinMode(limitSwitch, INPUT_PULLUP);
  grabCord.num = 0; //We used 33mm
  Wire.begin(Press); //This address 
  Serial.begin(9600);  
  Wire.onRequest(requestEvent); 
  Wire.onReceive(receiveEvent);
  Serial.print("Begin Test for Press\n"); 
  //ZeroOut();
  //Pressing();
}

void loop() {
  Tick();
}

//Not used (yet)
void requestEvent() {}

//Function interprets command then reads next packet of bytes
void receiveEvent(int bytes) { 
  if(command == 0x00) {
    command = Wire.read(); }
  else if(command == doPress) {
      while(Wire.available() ) {
          for (int i = 0; i < sizeof(double); i++) {
              grabCord.array[i] = Wire.read(); } 
      }
      commandFlag = 0x01; }
  else
    command = 0x00;
}

void ZeroOut() { 
  digitalWrite(dirMotor,LOW); 
  while(!digitalRead(limitSwitch)) { 
    if(!digitalRead(limitSwitch)) { digitalWrite(stepMotor,HIGH); } 
    delayMicroseconds(delayTime);
    if(!digitalRead(limitSwitch)) { digitalWrite(stepMotor,LOW); } 
    delayMicroseconds(delayTime);
  }
  N = 0;
  Serial.print("Done with Zero Out\n");
}

void Pressing() {
  Serial.print("Starting Press\n");
  if(grabCord.num < 35) { //Max press distance is 35mm
    int tmpN = grabCord.num*50;
    digitalWrite(dirMotor,HIGH);
    for(int i = 0; i <= tmpN; i++) {
      digitalWrite(stepMotor,HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(stepMotor,LOW);
      delayMicroseconds(delayTime);
    }
    delay(2000);
    digitalWrite(dirMotor,LOW);
    for(int i = 0; i <= 7*50; i++) {
      digitalWrite(stepMotor,HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(stepMotor,LOW);
      delayMicroseconds(delayTime);
    }
    delay(500);
    digitalWrite(dirMotor,HIGH);
    for(int i = 0; i <= 7*50; i++) {
      digitalWrite(stepMotor,HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(stepMotor,LOW);
      delayMicroseconds(delayTime);
    }
    delay(2000);
    ZeroOut();
   
  }
}
