
//I2C MCU1
//Full Step is 1.8 degrees
//One-eight step is 0.225 degrees
//Elbow-end stop at -135 degrees (theta2 value), this is only an estimate
//L1 = 269.107mm
//L2 = 190mm
//Z-height = 224mm (max for movement)
//Gear ratios: 
//Z-Axis: 20:80 reduction then 22:92 (motor turns)*(11/184) = z turns
//Elbow: 20:92 (motor turns)*(5/23) = elbow turns
//End-Effector: 20:92 (motor turns)*(5/23) = end turns + another 20:92 when elbow moves
//Z-Axis (vertical): 2mm/(full rotation)
//When sending command, use if(SCARA_ready) { myCommand(); SCARA_ready = 0x00; }


// included librarys:
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "Nextion.h"

//I2C Address declaration
#define MCU1 9 //I2C address (This MCU)
#define Press 10 //I2C address for snap fit press
#define SCARA_MCU 8 //I2C address

// Declare your Nextion objects - Example (page id = 0, component id = 1, component name = "b0") 
NexButton button_manual = NexButton(0, 3, "b0");
NexButton button_auto = NexButton(0, 4, "b1");
NexButton button_Xplus = NexButton(1, 7, "b0");
NexButton button_Xminus = NexButton(1, 3, "b1");
NexButton button_Yplus = NexButton(1, 2, "b2");
NexButton button_Yminus = NexButton(1, 4, "b3");
NexButton button_Zplus = NexButton(1, 5, "b4");
NexButton button_Zminus = NexButton(1, 6, "b5");
NexButton button_Rplus = NexButton(1, 12, "b9");
NexButton button_Rminus = NexButton(1, 13, "b10");
NexButton button_Gplus = NexButton(1, 11, "b7");
NexButton button_Gminus = NexButton(1, 10, "b6");
NexButton button_Back1 = NexButton(1, 8, "b8");
NexButton Script1 = NexButton(2, 3, "b1");
NexButton Script2 = NexButton(2, 5, "b2");
NexButton Script3 = NexButton(2, 4, "b3");
NexButton button_Back2 = NexButton(1, 2, "b0");
NexButton button_Cancel = NexButton(1, 6, "b4");


// Register a button object to the touch event list.  
NexTouch *nex_listen_list[] = {
  &button_manual,
  &button_auto,
  &button_Xplus,
  &button_Xminus,
  &button_Yplus,
  &button_Yminus,
  &button_Zplus,
  &button_Zminus,
  &button_Rplus,
  &button_Rminus,
  &button_Gplus,
  &button_Gminus,
  &button_Back1,
  &Script1,
  &Script2,
  &Script3,
  &button_Back2,
  &button_Cancel,
  NULL
};


//Function Prototypes
void requestEvent(); void receiveEvent(int bytes);
void sendXYZ(); void sendGrab(); void sendManual();
void getCmd(String file); void exacuteCmd(); void sendPress();
void menuFSM();

struct command{
int line;
int type;
float p1;
float p2;
float p3;
float p4;
int nxtLine;
} cmd;

int menuState;

//Flags send from SCARA to MCU1
char SCARA_ready = 0x00; //Flag to let MCU1 know SCARA is ready, see receiveEvent() 0x00 for not ready, 0x01 for ready
char Error = 0x00; //Flag set to 0x01 if error is returned

//Commands to send SCARA MCU1
const char moveXYZ = 0x01;     //Send XYZ cords and End Effector Values
const char grab = 0x02;        //Send grab command with char value
const char moveCE = 0x03;      //Send XYZ while End Effector maintains orientation
const char manualMode =0x04;  //Manual move motors
const char goHome = 0x05;     //Move to home position
char manualInput = 0x00;     //Array to hold button values

const double L1 = 269.107;
const double L2 = 190;
union Coordinates{ //Coordinates kept as union for I2C transmission as bytes
   byte array[4];
   double num;
};

//SCARA Coordinates
union Coordinates xCord; //forward KM (mm) 
union Coordinates yCord; //forward KM (mm) 
union Coordinates zCord; // (mm)

union Coordinates theta1; //Inverse KM Z-rotation (degrees)
union Coordinates theta2; //Inverse KM Elbow (degrees)
union Coordinates theta3; //End Effector rotation (degrees)
int endEffector = 0; //50 for fully open, ~170 for closed on Pad Pal

//Global variables for adafruit SD card module
File myFile;


//====================Nextion Functions================================//
//PushCallback for press event
//PopCallback for release event

void button_auto_PushCallback(void *ptr){
  menuState = 3;
}

void button_manual_PushCallback(void *ptr){
  menuState = 2;
  Serial.print("Reached Manual\n");
}

void button_Xplus_PushCallback(void *ptr) {
  //Add flag, change state, or call function
  cmd.p1 = cmd.p1+1;
  Serial.print("X-Plus\n");

  if (SCARA_ready) {
        xCord.num = cmd.p1;
        
        sendXYZ();
        SCARA_ready = 0;
  }
}

//Example for pop callback. Use for button release
void button_Xplus_PopCallback(void *ptr) {
}

void button_Xminus_PushCallback(void *ptr) {
  cmd.p1 = cmd.p1-1;
  Serial.print("X-minus\n");

  if (SCARA_ready) {
        xCord.num = cmd.p1;
        
        sendXYZ();
        SCARA_ready = 0;
  }
}

void button_Yplus_PushCallback(void *ptr) {
  cmd.p2 = cmd.p2+1;
  Serial.print("Y-Plus\n");

  if (SCARA_ready) {
        yCord.num = cmd.p2;

        sendXYZ();
        SCARA_ready = 0;
  }
}

void button_Yminus_PushCallback(void *ptr) {
  cmd.p2 = cmd.p2-1;
  Serial.print("Y-minus\n");

  if (SCARA_ready) {
        yCord.num = cmd.p2;

        sendXYZ();
        SCARA_ready = 0;
  }
}

void button_Zplus_PushCallback(void *ptr) {
  Serial.print("Z-Plus\n");
  if(cmd.p3 < 223) {
    cmd.p3 = cmd.p3+1;
    if (SCARA_ready) {
        zCord.num = cmd.p3;

        sendXYZ();
        SCARA_ready = 0;
  }
    }
  else
    cmd.p3 = 224;
    if (SCARA_ready) {
        zCord.num = cmd.p3;
        
        sendXYZ();
        SCARA_ready = 0;
  }
}

void button_Zminus_PushCallback(void *ptr) {
  Serial.print("Z-minus\n");
  if(cmd.p3 > 1) {
    cmd.p3 = cmd.p3-1; 
    if (SCARA_ready) {
        zCord.num = cmd.p3;

        sendXYZ();
        SCARA_ready = 0;
  }
    }
  else
    cmd.p3 = 0;
    if (SCARA_ready) {
        zCord.num = cmd.p3;

        sendXYZ();
        SCARA_ready = 0;
  }
}

void button_Rplus_PushCallback(void *ptr) {
  cmd.p4 = cmd.p4+1;
  Serial.print("R-Plus\n");

  if (SCARA_ready) {
        theta3.num = cmd.p4;
        
        sendXYZ();
        SCARA_ready = 0;
  }
}

void button_Rminus_PushCallback(void *ptr) {
  cmd.p4 = cmd.p4-1;
  Serial.print("R-minus\n");

  if (SCARA_ready) {
        theta3.num = cmd.p4;
        
        sendXYZ();
        SCARA_ready = 0;
  }
}

void button_Gplus_PushCallback(void *ptr) {
  endEffector++;
  Serial.print("G-Plus\n");

  while(SCARA_ready == 0){
    delay(50);
    }
  sendGrab(); 
  delay(500);
}

void button_Gminus_PushCallback(void *ptr) {
  endEffector--;
  Serial.print("G-minus\n");

  
  while(SCARA_ready == 0){
    delay(50);
    }
  sendGrab(); 
  delay(500);
}

void button_Back1_PushCallback(void *ptr) {
  menuState = 1;
}

void button_Script1_PushCallback(void *ptr) {
  menuState = 4;
}

void button_Script2_PushCallback(void *ptr) {
  
}

void button_Script3_PushCallback(void *ptr) {
  
}

void button_Back2_PushCallback(void *ptr) {
  menuState = 1;
}

void button_Cancel_PushCallback(void *ptr) {
  menuState = 3;
}
//=====================================================================//


void setup() {
  Serial.begin(9600);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Austin's Setup~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xCord.num = 140.61; yCord.num = -80.18; zCord.num = 10;
  theta1.num = 0;     theta2.num = 0;     theta3.num = 5;
  endEffector = 80;
  cmd.p1 = xCord.num; cmd.p2 = yCord.num; cmd.p3 = zCord.num; cmd.p4 = theta3.num;
  Wire.begin(MCU1);     //This address   
  Wire.onRequest(requestEvent); //see requestEvent() below
  Wire.onReceive(receiveEvent); //See recieveEnvent() below
  Serial.print("Begin Test\n");
  
  //===============================SD Card File IO setup=============================================================//
  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    return;
    } 
             
  Serial.println("initialization done.");

  //initialize SD command Loactions
  cmd.nxtLine = 1;
//===============================Nextion LCD IO setup=============================================================//
  nexInit();
  button_Xplus.attachPop(button_Xplus_PopCallback, &button_Xplus); //Left as an example for button pop
  button_Xplus.attachPush(button_Xplus_PushCallback, &button_Xplus);
  button_Xminus.attachPush(button_Xminus_PushCallback, &button_Xminus);
  button_Yplus.attachPush(button_Yplus_PushCallback, &button_Yplus);
  button_Yminus.attachPush(button_Yminus_PushCallback, &button_Yminus);
  button_Zplus.attachPush(button_Zplus_PushCallback, &button_Zplus);
  button_Zminus.attachPush(button_Zminus_PushCallback, &button_Zminus);
  button_Rplus.attachPush(button_Rplus_PushCallback, &button_Rplus);
  button_Rminus.attachPush(button_Rminus_PushCallback, &button_Rminus);
  button_Gplus.attachPush(button_Gplus_PushCallback, &button_Gplus);
  button_Gminus.attachPush(button_Gminus_PushCallback, &button_Gminus);
  button_Back1.attachPush(button_Back1_PushCallback, &button_Back1);
  Script1.attachPush(button_Script1_PushCallback, &Script1);
  Script2.attachPush(button_Script2_PushCallback, &Script2);
  Script3.attachPush(button_Script3_PushCallback, &Script3);
  button_Back2.attachPush(button_Back2_PushCallback, &button_Back2);
  button_Cancel.attachPush(button_Cancel_PushCallback, &button_Cancel);
  button_auto.attachPush(button_auto_PushCallback, &button_auto);
  button_manual.attachPush(button_manual_PushCallback, &button_manual);
  
}

void loop() {
  nexLoop(nex_listen_list);
  menuFSM();
}

void menuFSM(){
  switch (menuState) {
    case 1:{

    }
    break;

    case 2:{ //Manual mode
//      if (SCARA_ready) {
//        xCord.num = cmd.p1;
//        yCord.num = cmd.p2;
//        zCord.num = cmd.p3;
//        theta3.num = cmd.p4;
//        
//        sendXYZ();
//        SCARA_ready = 0;
//        while(SCARA_ready == 0){delay(50);}
//        endEffector = (int)cmd.p1;
//        sendGrab();  
//      } 
    }
    break;

    case 3:{ //Auto mode

    }
    break;

    case 4:{ //Script-1 on auto mode
      getCmd("myFile.txt");
      exacuteCmd();
    }
    break;

    default:

    break;
  }
}

void exacuteCmd(){
  Serial.println("=================Exacuting Command====================");
  Serial.println(cmd.type);
  switch (cmd.type) {
    case 1:{ //move scara
      Serial.print("Exacuting Move: "); Serial.println(cmd.line);
      while(SCARA_ready == 0){delay(50);}
      if(SCARA_ready){
        xCord.num = cmd.p1;
        yCord.num = cmd.p2;
        zCord.num = cmd.p3;
        theta3.num = cmd.p4;
    
        sendXYZ();
        SCARA_ready = 0;
      }
      
    }
    break;

    case 2:{ //SCARA  grab
      while(SCARA_ready == 0){delay(50);}
      if(SCARA_ready) {
        sendGrab();  
        SCARA_ready = 0x00; }
      
    }
    break;

    case 3:{}
    break;

    case 4:{}
    break;
    
    case 5:{ //Delay before next command
      //cmd for delaying
      int tmp1 = ((int)cmd.p1)*1000;
      delay(tmp1);

      Serial.print("Line: = "); Serial.print(cmd.line);
      Serial.print("  cmd: = "); Serial.print(cmd.type);
      Serial.print("  p1: = "); Serial.print(cmd.p1);
      Serial.print("  p2: = "); Serial.print(cmd.p2);
      Serial.print("  p3: = "); Serial.print(cmd.p3);
      Serial.print("  p4: = "); Serial.print(cmd.p4);
      Serial.print("  nxtLine: = "); Serial.println(cmd.nxtLine);
    }
    break;

    case 6: {
      //cmd for press activation
      //float tmp = zCord.num; //33mm with SD card for Pad_pal
      zCord.num = cmd.p1;
      sendPress();
      //zCord.num = tmp; 

      Serial.print("Line: = "); Serial.print(cmd.line);
      Serial.print("  cmd: = "); Serial.print(cmd.type);
      Serial.print("  p1: = "); Serial.print(cmd.p1);
      Serial.print("  p2: = "); Serial.print(cmd.p2);
      Serial.print("  p3: = "); Serial.print(cmd.p3);
      Serial.print("  p4: = "); Serial.print(cmd.p4);
      Serial.print("  nxtLine: = "); Serial.println(cmd.nxtLine);
    }
    break;

    default:{
      Serial.println("Unrecognized command!");
      Serial.print("Line: = "); Serial.print(cmd.line);
      Serial.print("  cmd: = "); Serial.print(cmd.type);
      Serial.print("  p1: = "); Serial.print(cmd.p1);
      Serial.print("  p2: = "); Serial.print(cmd.p2);
      Serial.print("  p3: = "); Serial.print(cmd.p3);
      Serial.print("  p4: = "); Serial.print(cmd.p4);
      Serial.print("  nxtLine: = "); Serial.println(cmd.nxtLine);
    }
    break;
  }
  Serial.println("=================done====================");
}

void getCmd(String file){
  myFile = SD.open(file);
  Serial.println("File opened");

  command tmpCmd;
  
  if(myFile){

    while(myFile.available()){

      tmpCmd.line = myFile.parseInt();
      tmpCmd.type = myFile.parseInt();
      tmpCmd.p1 = myFile.parseFloat();
      tmpCmd.p2 = myFile.parseFloat();
      tmpCmd.p3 = myFile.parseFloat();
      tmpCmd.p4 = myFile.parseFloat();
      tmpCmd.nxtLine = myFile.parseInt();

      if(tmpCmd.line == cmd.nxtLine) {
        cmd = tmpCmd;
        break;
      }
    }
  }

  else {Serial.println("File failed to opened");}

  Serial.println("grabbed command!");
  Serial.print("Line: = "); Serial.print(cmd.line);
  Serial.print("  cmd: = "); Serial.print(cmd.type);
  Serial.print("  p1: = "); Serial.print(cmd.p1);
  Serial.print("  nxtLine: = "); Serial.println(cmd.nxtLine);

  myFile.close();
  Serial.println("File closed");
}

void requestEvent() { } //Not used

//Can expand to recieve more information
void receiveEvent(int bytes) {
    char tmp = Wire.read();
    if(tmp == 0x01)
      SCARA_ready = 0x01;
    else if(tmp = 0x02)
      Error = 0x01;
    //Add coordinates to be sent back
    
}

//Sends coordinates to SCARA (x, y, z, theta)
void sendXYZ() {
  Wire.beginTransmission(SCARA_MCU); 
  Wire.write(moveXYZ);
  Wire.endTransmission();
  Wire.beginTransmission(SCARA_MCU);
  for (int i = 0; i < sizeof(double); i++) { //Send 4 byte double
      Wire.write(xCord.array[i]);   }  
  for (int i = 0; i < sizeof(double); i++) { //Send 4 byte double
      Wire.write(yCord.array[i]);   }  
  for (int i = 0; i < sizeof(double); i++) { //Send 4 byte double
      Wire.write(zCord.array[i]);   }
  for (int i = 0; i < sizeof(double); i++) { //Send 4 byte double
      Wire.write(theta3.array[i]);   }            
  Wire.endTransmission();    
}


void sendPress() {
  Wire.beginTransmission(Press); 
  Wire.write(0x01); //<-potential command expansion. generic only command ATM
  Wire.endTransmission();
  Wire.beginTransmission(Press);
  for (int i = 0; i < sizeof(double); i++) { //Send 4 byte double
      Wire.write(zCord.array[i]);   }          
  Wire.endTransmission(); 
}

//Send End-effector Grab value
void sendGrab() {
  endEffector = (int) cmd.p1;
  if((endEffector) >= 0 || (endEffector <= 180)) {
    Wire.beginTransmission(SCARA_MCU); 
    Wire.write(grab);
    Wire.endTransmission();
    Wire.beginTransmission(SCARA_MCU); 
    Wire.write(endEffector);
    Wire.endTransmission();
  }
}

double forward_x(double theta1, double theta2) {
  double theta1_d = (theta1 * 3.14) / 180;
  double theta2_d = (theta2 * 3.14) / 180;
  double x = L1*cos(theta1_d) + L2*cos(theta1_d+ theta2_d);
  return x;
}

double forward_y(double theta1, double theta2) {
  double theta1_d = (theta1 * 3.14) / 180;
  double theta2_d = (theta2 * 3.14) / 180;
  double y = L1*sin(theta1_d)+ L2*sin(theta1_d+ theta2_d);
  return y;
}
