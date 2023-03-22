//I2C SCARA MCU
//Full Step is 1.8 degrees
//One-eight step is 0.225 degrees
//Z-Rotation is in 1/8 step, Z-Vertical is in 1/4 step, Elbow is in 1/16 step, End-Effector in 1/8 Step
//Elbow-end stop at -145 degrees (theta2 value), (this is only an estimate!)
//Elbow-end stop can't go past +135 degrees
// x^2+y^2 < (L1+L2)^2 and for LH x^2+y^2 > L2*cos(35) = 155.6 mm
//L1 = 269.107mm
//L2 = 190mm
//Z-height = 224mm (max for movement)
//Gear ratios: 
//Z-Axis (Rotation): 20:80 reduction then 22:92 (motor turns)*(11/184) = z turns
//Elbow: 20:92 (motor turns)*(5/23) = elbow turns
//End-Effector: 20:92 (motor turns)*(5/23) = end turns + another 20:92 when elbow moves
//Z-Axis (vertical): 2mm/(full rotation)

#include <AccelStepper.h>
#include <Wire.h> 
#include <Servo.h>

#define MCU1 9 //I2C address (MCU1)
#define SCARA_MCU 8 //I2C address (This MCU)

#define stepPinX  2 //Z-rotation
#define dirPinX   5
AccelStepper stepperX(1, 2, 5); // (Type:driver, STEP, DIR)

#define stepPinY  3 //Elbow Rotation
#define dirPinY   6 
AccelStepper stepperY(1, 3, 6);

#define stepPinZ  4 //Z Vertical
#define dirPinZ   7 
AccelStepper stepperZ(1, 4, 7);

#define stepPinA  12 //End Effector Rotate
#define dirPinA   13
AccelStepper stepperA(1, 12, 13);

//Assigning pin connections to limit switches
#define limitSwitchX 9
#define limitSwitchY 10
#define limitSwitchZ 11 
#define limitSwitchA A3 //CoolEN pin

Servo myServo; //for end-effector grabber

//Function Prototypes
void sendReadyStatus(); void requestEvent(); void receiveEvent(int bytes);
void manual_Rotate(); void ZeroOut(); void InvereKM(); char checkXYZ();

//List of Commands from SCARA to MCU1
const char SCARA_ready = 0x01;  //Lets MCU1 know that new commands can be recieved. Only changes on MCU1.
const char Error = 0x02; //Return error if xyz-coordinates are invalid

//List of Commands from MCU1 to SCARA
const char moveXYZ = 0x01;     //Send XYZ cords and End Effector Values
const char grab = 0x02;        //Send grab command with char value
const char moveCE = 0x03;      //Send XYZ while End Effector maintains orientation
const char manualMode =0x04;  //Manual move motors
char command = 0xFF; //Stores incoming command. Keep at 0x00 if no commands are being executed
char commandFlag = 0x01; //Set to 0x01 if command is being executed. Is set when command is recieved/before data is recieved
char errorFlag = 0x00; //Set to 0x01 if there is an error, send error message, clear flag then return to Wait state
char manualInput = 0x00;

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
int endEffector = 0;      //Value for servo motor in degrees

//Total motor rotations measured from end stop
long N1 = 0; //Motor rotations for Z-axis rotation
long N2 = 0; //Motor rotations for elbow rotation
long N3 = 0; //Motor rotations for end-effector rotation
long NZ = 0; //Motor rotations for Z-axis (vertical) rotation
const double zMax = 224; //mm


enum States {Start, Wait, Move, Grab, MoveCE, Manual} state;

void Tick() {
    switch (state) { //Transitions
        case Start:
            if((command == 0x00) && (commandFlag == 0x00)) {
              state = Wait;
              sendReadyStatus(); }
            else 
              state = Start;
            break;
        case Wait:
            if((command == moveXYZ) && (commandFlag == 0x01)) {
              state = Move; }
            else if((command == manualMode) && (commandFlag == 0x01)) {
              state = Manual; }
            else if((command == grab) && (commandFlag == 0x01)) {
              state = Grab; }
            else
              state = Wait;
            break;
        case Move:
            if((command == 0x00) && (commandFlag == 0x00)) {
              state = Wait;
              sendReadyStatus(); }
            else
              state = Move;
            break;
        case Grab:
            if((command == 0x00) && (commandFlag == 0x00)) {
              state = Wait;
              sendReadyStatus(); }
            else
              state = Grab;
            break;
        case Manual:
          if((command == 0x00) && (commandFlag == 0x00)) {
              state = Wait;
              sendReadyStatus(); }
            else
              state = Manual;
          break;
        default:
          break;
    }
     switch (state) { //State Actions
        case Start:
          ZeroOut(); //Moves all motors to End Stops
          command = 0x00;
          commandFlag = 0x00;
          break;
        case Wait:
          break;
        case Move:
          if(checkXYZ()) { //Check if XYZ are valid
            sendError();
            command = 0x00;
            commandFlag = 0x00; }
          else { 
            moveSCARA();
            //moveArm();
            command = 0x00;
            commandFlag = 0x00; }
          break; 
        case Grab:
          doGrabby();
          command = 0x00;
          commandFlag = 0x00;
          break;
        case Manual:
          manual_Rotate();
          command = 0x00;
          commandFlag = 0x00;
          break;
        default:
            break;
     }
}

void setup() {
//  pinMode(stepPinX,OUTPUT);  pinMode(dirPinX,OUTPUT);
//  pinMode(stepPinY,OUTPUT);  pinMode(dirPinY,OUTPUT);
//  pinMode(stepPinZ,OUTPUT);  pinMode(dirPinZ,OUTPUT);
//  pinMode(stepPinA,OUTPUT);  pinMode(dirPinA,OUTPUT);
  // Stepper motors max speed
  stepperX.setMaxSpeed(2000);
  stepperX.setAcceleration(2000);
  stepperY.setMaxSpeed(2000);
  stepperY.setAcceleration(2000);
  stepperZ.setMaxSpeed(4000);
  stepperZ.setAcceleration(2000);
  stepperA.setMaxSpeed(2000);
  stepperA.setAcceleration(2000);

  myServo.attach(A0, 600, 2500);
  myServo.write(50);

  pinMode(limitSwitchX, INPUT_PULLUP);  pinMode(limitSwitchY, INPUT_PULLUP);
  pinMode(limitSwitchZ, INPUT_PULLUP);  pinMode(limitSwitchA, INPUT_PULLUP);
  
  xCord.num =  0; yCord.num =  0; zCord.num =  0;
  theta1.num = 0; theta2.num = 0; theta3.num = 0;
  Wire.begin(SCARA_MCU); //This address 
  Serial.begin(9600);  
  Wire.onRequest(requestEvent); 
  Wire.onReceive(receiveEvent);
  Serial.print("Begin Test for SCARA\n"); 
}

void loop() {
  Tick();
}

//Tell MCU1 that Scara is Ready for next command
void sendReadyStatus() {
  Wire.beginTransmission(MCU1); 
  Wire.write(SCARA_ready); //This is a constant
  Wire.endTransmission();
}

void sendError(){
  Wire.beginTransmission(MCU1); 
  Wire.write(Error); //This is a constant
  Wire.endTransmission();
}

//Not used (yet)
void requestEvent() {}

//Function interprets command then reads next packet of bytes
void receiveEvent(int bytes) { 
  if(command == 0x00) {
    command = Wire.read(); }
  else if(command == moveXYZ) {
      while(12 < Wire.available() ) {
          for (int i = 0; i < sizeof(double); i++) {
              xCord.array[i] = Wire.read(); } }
      while(8 < Wire.available() ) {
           for (int i = 0; i < sizeof(double); i++) {
              yCord.array[i] = Wire.read(); } }
      while(4 < Wire.available() ) {
           for (int i = 0; i < sizeof(double); i++) {
              zCord.array[i] = Wire.read(); } }
      while(Wire.available() ) {
           for (int i = 0; i < sizeof(double); i++) {
              theta3.array[i] = Wire.read(); } 
      }
      commandFlag = 0x01; }
  else if(command == grab) {
      endEffector = Wire.read(); 
      commandFlag = 0x01; }
  else if(command == manualMode) {
    manualInput = Wire.read();
    commandFlag = 0x01; }
  else
    command = 0x00;
}

void ZeroOut() { 
  double tmpN1 = 0; double tmpN2 = 0; double tmpN3 = 0; double tmpNZ = 0;
  double tmpTheta1 = 0; double tmpTheta2 = 0;
  endEffector = 80;
  doGrabby();
  digitalWrite(dirPinX,LOW); digitalWrite(dirPinY,LOW);
  digitalWrite(dirPinZ,LOW); digitalWrite(dirPinA,LOW);
  while(!digitalRead(limitSwitchA)) { 
    if(!digitalRead(limitSwitchA)) { digitalWrite(stepPinA,HIGH); ++tmpN3;}
    delayMicroseconds(300);
    if(!digitalRead(limitSwitchA)) { digitalWrite(stepPinA,LOW); }
    delayMicroseconds(300);
  } 

  while(!digitalRead(limitSwitchX) || !digitalRead(limitSwitchY) || !digitalRead(limitSwitchZ) || !digitalRead(limitSwitchA)) { 
    if(!digitalRead(limitSwitchX)) { digitalWrite(stepPinX,HIGH); ++tmpN1;} 
    if(!digitalRead(limitSwitchY)) { digitalWrite(stepPinY,HIGH); ++tmpN2;}
    if(!digitalRead(limitSwitchZ)) { digitalWrite(stepPinZ,HIGH); ++tmpNZ;}
    if(!digitalRead(limitSwitchA)) { digitalWrite(stepPinA,HIGH);}
    delayMicroseconds(300);
    if(!digitalRead(limitSwitchX)) { digitalWrite(stepPinX,LOW); } 
    if(!digitalRead(limitSwitchY)) { digitalWrite(stepPinY,LOW); }
    if(!digitalRead(limitSwitchZ)) { digitalWrite(stepPinZ,LOW); }
    if(!digitalRead(limitSwitchA)) { digitalWrite(stepPinA,LOW); }
    delayMicroseconds(300);
  }
  tmpTheta1 = (tmpN1*11*0.225)/184;
  tmpTheta2 = (tmpN2*5*0.1125)/23 - 145;
  Serial.print("Theta 1: "); Serial.print(tmpTheta1); Serial.print("\n");
  Serial.print("Theta 2: "); Serial.print(tmpTheta2); Serial.print("\n");
  Serial.print("X-Coordinate: ");
  Serial.print(forward_x(tmpTheta1, tmpTheta2) );
  Serial.print("\n");
  Serial.print("Y-Coordinate: ");
  Serial.print(forward_y(tmpTheta1, tmpTheta2) );
  Serial.print("\n");
  Serial.print("Z-Coordinate: ");
  Serial.print( (tmpNZ*0.45)/180);
  Serial.print("\n");
   Serial.print("End-Angle: ");
  Serial.print( (tmpN3*5*0.225)/23);
  Serial.print("\n");
  //Total motor rotations measured from end stop
  N1 = 0; N2 = 0; N3 = 0; NZ = 0; 

  //Zero out all stepper location
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);
  stepperA.setCurrentPosition(0);

  //previouse function block resets speed so these change it back
  stepperX.setMaxSpeed(2000);
  stepperX.setAcceleration(2000);
  stepperY.setMaxSpeed(2000);
  stepperY.setAcceleration(2000);
  stepperZ.setMaxSpeed(4000);
  stepperZ.setAcceleration(2000);
  stepperA.setMaxSpeed(2000);
  stepperA.setAcceleration(2000);  
}

void manual_Rotate() {
  if(0x10 & manualInput) { digitalWrite(dirPinX,HIGH); } 
  else {                   digitalWrite(dirPinX,LOW);  } 
  if(0x20 & manualInput) { digitalWrite(dirPinY,HIGH); } 
  else {                   digitalWrite(dirPinY,LOW);  } 
  if(0x40 & manualInput) { digitalWrite(dirPinZ,HIGH); } 
  else {                   digitalWrite(dirPinZ,LOW);  } 
  if(0x80 & manualInput) { digitalWrite(dirPinA,HIGH); } 
  else {                   digitalWrite(dirPinA,LOW);  } 
  for(int x = 0; x < 92; x++) { //keep at x<92 for manual (92 to ensure accuracy in counting step correction for end-effector)
    if(0x01 & manualInput) { if(!digitalRead(limitSwitchX)){ digitalWrite(stepPinX,HIGH); } }
    else if(0x10 & manualInput)                            { digitalWrite(stepPinX,HIGH); } 
    if(0x02 & manualInput) {if(!digitalRead(limitSwitchY)) { digitalWrite(stepPinY,HIGH); } }
    else if(0x20 & manualInput)                            { digitalWrite(stepPinY,HIGH); } 
    if(0x04 & manualInput) {if(!digitalRead(limitSwitchZ)) { digitalWrite(stepPinZ,HIGH); } }
    else if(0x40 & manualInput)                            { digitalWrite(stepPinZ,HIGH); } 
    if(0x08 & manualInput) {if(!digitalRead(limitSwitchA)) { digitalWrite(stepPinA,HIGH); } }
    else if(0x80 & manualInput)                            { digitalWrite(stepPinA,HIGH); } 
    delayMicroseconds(300);
    if(0x01 & manualInput)      { digitalWrite(stepPinX,LOW); }
    else if(0x10 & manualInput) { digitalWrite(stepPinX,LOW); } 
    if(0x02 & manualInput)      { digitalWrite(stepPinY,LOW); }
    else if(0x20 & manualInput) { digitalWrite(stepPinY,LOW); }
    if(0x04 & manualInput)      { digitalWrite(stepPinZ,LOW); }
    else if(0x40 & manualInput) { digitalWrite(stepPinZ,LOW); } 
    if(0x08 & manualInput)      { digitalWrite(stepPinA,LOW); }
    else if(0x80 & manualInput) { digitalWrite(stepPinA,LOW); }
    delayMicroseconds(300);
  }
  //Update turn count number
  if(0x01 & manualInput) {
    if(0x10 & manualInput)
      N1 = N1 + 92;
    else
      N1 = N1 - 92;
  }
  if(0x02 & manualInput) {
    if(0x20 & manualInput)
      N2 = N2 + 92;
    else
      N2 = N2 - 92;
  }
  if(0x04 & manualInput) {
    if(0x40 & manualInput)
      NZ = NZ + 92;
    else
      NZ = NZ - 92;
  }
  if(0x08 & manualInput) {     //Does N3 need to change
    if(0x80 & manualInput) {   //What direction motor is turning
      if(0x02 & manualInput) { //Did Elbow move?
        if(0x20 & manualInput) //What direction for elbow
          N3 = N3 + 92 - 10;
        else 
          N3 = N3 + 92 + 10;
      }
      else
        N3 = N3 + 92;
    }
    else {
      if(0x02 & manualInput) {  //Did Elbow move?
        if(0x20 & manualInput)  //What direction for elbow
          N3 = N3 - 92 + 10;
        else 
          N3 = N3 - 92 - 10;
      }
      else
        N3 = N3 - 92;
      }
  }
  if(N1 < 0) N1 = 0; if(N2 < 0) N2 = 0; if(N3 < 0) N3 = 0; if(NZ < 0) NZ = 0;
  manualInput = 0x00; //Reset input from MCU1
}

void doGrabby() {
    Serial.print("Doing grab\n");
    //Serial.print("Servo Angle: ");
    //Serial.print(endEffector);
    //Serial.print("\n");
    myServo.write(endEffector); //0 open, 180 closed
    delay(800);
}

double forward_x(double theta1, double theta2) {
  double theta1_d = (theta1 * 3.14) / 180;
  double theta2_d = (theta2 * 3.14) / 180;
  double x = L1*cos(theta1_d)+ L2*cos(theta1_d+ theta2_d);
  return x;
}

double forward_y(double theta1, double theta2) {
  double theta1_d = (theta1 * 3.14) / 180;
  double theta2_d = (theta2 * 3.14) / 180;
  double y = L1*sin(theta1_d)+ L2*sin(theta1_d + theta2_d);
  return y;
}

double inverseR_theta1(double x, double y) {
  double c = x*x+y*y-L1*L1-L2*L2;
  double theta = atan2(y,x) - atan2(sqrt(4*sq(L1*L2)-sq(c)),sq(x)+sq(y)+sq(L1)-sq(L2));
  return (theta*180)/3.1415;
}

double inverseR_theta2(double x, double y) {
  double c = x*x+y*y-L1*L1-L2*L2;
  double theta = acos(c/(2*L1*L2));
  return (theta*180)/3.1415;
}

double inverseL_theta1(double x, double y) {
  double c = x*x+y*y-L1*L1-L2*L2;
  double theta = atan2(y,x) + atan2(sqrt(4*sq(L1*L2)-sq(c)),sq(x)+sq(y)+sq(L1)-sq(L2));
  return (theta*180)/3.1415;
}

double inverseL_theta2(double x, double y) {
  double c = x*x+y*y-L1*L1-L2*L2;
  double theta = -1*acos(c/(2*L1*L2));
  return (theta*180)/3.1415;
}

//Check if XYZ are valid. If valid, assigns values to theta1.num and theta2.num
//Currently only used for LH Kinematics
char checkXYZ(){  
  double tempTheta1 = 0;
  double tempTheta2 = 0;
  if(zMax < zCord.num)
    return 0x01;
  else if((sq(xCord.num)+sq(yCord.num)) > sq(L1+L2))
    return 0x01;
  else if((sq(xCord.num)+sq(yCord.num)) < sq(L1-L2*cos( (35*3.14)/180) ) )
    return 0x01;
  else {
    tempTheta1 = inverseL_theta1(xCord.num, yCord.num);
    tempTheta2 = inverseL_theta2(xCord.num, yCord.num);
    if((tempTheta2 < -145) || (tempTheta2 > 135))
      return 0x00;
    else if((tempTheta1 < 0) || (tempTheta1 > 210))
      return 0x00;
    else {
      theta1.num = tempTheta1;
      theta2.num = tempTheta2;
      return 0x00; }
  }
}

//Move Scara to a new possition
void moveSCARA(){
  Serial.print("Moving arm");
  //Turn count motors need to move to
  long tmpN1 = (184*theta1.num)/(11*0.225);
  long tmpN2 = (23 * (theta2.num + 145) )/(5*0.1125); //-145 degress should make N2=0 turns
  long tmpN3 = (23*theta3.num)/(5*0.225); 
  long tmpNZ = (zCord.num*360)/(2*0.45);

  //compute the difference between old possition and new positon
  long diffN2 = abs(tmpN2-N2);
  
  //Correction factor for elbow moving end-effector
  long N3_CorrectionFactor = (5*diffN2)/46;

  //Checack which direction elbow is moving, and + or - the factor.
  if(tmpN2 > N2) {//minus if both moving same direction
        tmpN3 = tmpN3 - N3_CorrectionFactor; 
        } 
      else
        tmpN3 = tmpN3 + N3_CorrectionFactor;

  //Set Theta1 (hooked up to X on CNC shield) stepper speed and accel
  stepperX.setMaxSpeed(2000);
  stepperX.setAcceleration(2000);

  //Set Theta2 (hooked up to Y on CNC shield) stepper speed and accel
  stepperY.setMaxSpeed(2000);
  stepperY.setAcceleration(2000);

  //Set Z stepper speed and accel (hooked up to Z on CNC shield)
  stepperZ.setMaxSpeed(4000);
  stepperZ.setAcceleration(2000);

  //Set Theta3 stepper speed and accel on CNC shield
  stepperA.setMaxSpeed(2000);
  stepperA.setAcceleration(2000);

  //Pass the new step location to the stepper objects
  stepperX.moveTo(tmpN1);
  stepperY.moveTo(tmpN2);
  stepperZ.moveTo(tmpNZ);
  stepperA.moveTo(tmpN3);

  //Run the stepper motors until the they reach their new step count/possition
  while (stepperX.currentPosition() != tmpN1 || stepperY.currentPosition() != tmpN2 || stepperZ.currentPosition() != tmpNZ || stepperA.currentPosition() != tmpN3) {
    stepperX.run();
    stepperY.run();
    stepperZ.run();
    stepperA.run();
  }

  //Assign new values to the official step count/possition
    N1 = tmpN1;
    N2 = tmpN2;
    N3 = tmpN3;
    NZ = tmpNZ;

  //Print Currnt Postion
  Serial.print("X-Coordinate: ");
  Serial.print(xCord.num);
  Serial.print("\n");
  Serial.print("Y-Coordinate: ");
  Serial.print(yCord.num);
  Serial.print("\n");
  Serial.print("Z-Coordinate: ");
  Serial.print(zCord.num);
  Serial.print("\n");
  Serial.print("End-Angle: ");
  Serial.print(theta3.num);
  Serial.print("\n");
  Serial.print("Servo Angle: ");
  Serial.print(endEffector);
  Serial.print("\n");
}

//Use function to move arm to any xyz-coordinates. See Move state in function Tick()
void moveArm() {
  Serial.print("Moving arm");
  //Turn count motors need to move to
    long tmpN1 = (184*theta1.num)/(11*0.225);
    long tmpN2 = (23 * (theta2.num + 145) )/(5*0.1125); //-145 degress should make N2=0 turns
    long tmpN3 = (23*theta3.num)/(5*0.225); 
    long tmpNZ = (zCord.num*360)/(2*0.45);

    long diffN1 = abs(tmpN1-N1);
    long diffN2 = abs(tmpN2-N2);
    long diffN3 = abs(tmpN3-N3); 
    long diffNZ = abs(tmpNZ-NZ);

    //Correction factor for elbow moving end-effector
    long N3_CorrectionFactor = (5*diffN2)/46;
    
    if(tmpN1 > N1)
      digitalWrite(dirPinX,HIGH); 
    else 
      digitalWrite(dirPinX,LOW); //Clock-wise rotation
    if(tmpN2 > N2)
      digitalWrite(dirPinY,HIGH); 
    else 
      digitalWrite(dirPinY,LOW); //Clock-wise rotation
    if(tmpN3 > N3) {
      digitalWrite(dirPinA,HIGH); 
      if(tmpN2 > N2) {
        diffN3 = diffN3 - N3_CorrectionFactor; } //minus if both moving same direction
      else
        diffN3 = diffN3 + N3_CorrectionFactor;
    }
    else {
      digitalWrite(dirPinA,LOW); //Clock-wise rotation
      if(tmpN2 > N2) {
        diffN3 = diffN3 + N3_CorrectionFactor; }
      else
        diffN3 = diffN3 - N3_CorrectionFactor;
    }
    if(tmpNZ > NZ)
      digitalWrite(dirPinZ,HIGH); 
    else 
      digitalWrite(dirPinZ,LOW);
    
    while(diffN1 || diffN2 || diffN3 || diffNZ) { 
      if(diffN1) { 
        if(tmpN1 > N1) { 
          digitalWrite(stepPinX,HIGH); 
          } 
        else if(!digitalRead(limitSwitchX)) { 
          digitalWrite(stepPinX,HIGH); 
          } 
      }  
      if(diffN2) { 
        if(tmpN2 > N2) { digitalWrite(stepPinY,HIGH); 
        } 
        else if(!digitalRead(limitSwitchY)) {
          digitalWrite(stepPinY,HIGH); 
          } 
      }
      if(diffNZ) { 
        if(tmpNZ > NZ) { 
          digitalWrite(stepPinZ,HIGH); 
        } 
        else if(!digitalRead(limitSwitchZ)) { 
          digitalWrite(stepPinZ,HIGH); 
          } 
      }
      if(diffN3) { 
        if(tmpN3 > N3) { 
          digitalWrite(stepPinA,HIGH); 
          } 
        else if(!digitalRead(limitSwitchA)) { 
          digitalWrite(stepPinA,HIGH); 
          } 
      }

      delayMicroseconds(300);

      if(diffN1) { 
        if(tmpN1 > N1) { 
          digitalWrite(stepPinX,LOW); diffN1 = diffN1 - 1; 
          } 
        else if(!digitalRead(limitSwitchX)) { 
          digitalWrite(stepPinX,LOW); diffN1 = diffN1 - 1; 
          } 
      }
      
      if(diffN2) { 
        if(tmpN2 > N2) { 
          digitalWrite(stepPinY,LOW); diffN2 = diffN2 - 1; 
          } 
        else if(!digitalRead(limitSwitchY)) { 
          digitalWrite(stepPinY,LOW); diffN2 = diffN2 - 1; 
          } 
      }
      
      if(diffNZ) { 
        if(tmpNZ > NZ) { 
          digitalWrite(stepPinZ,LOW); diffNZ = diffNZ - 1; 
          }  
        else if(!digitalRead(limitSwitchZ)) { 
          digitalWrite(stepPinZ,LOW); diffNZ = diffNZ - 1; 
          } 
      } 
      
      if(diffN3) { 
        if(tmpN3 > N3) { 
          digitalWrite(stepPinA,LOW); diffN3 = diffN3 - 1; 
          } 
        else if(!digitalRead(limitSwitchA)) { 
          digitalWrite(stepPinA,LOW); diffN3 = diffN3 - 1; 
          } 
      }

      delayMicroseconds(300);
    }
    //New values for turn count of each motor
    N1 = tmpN1;
    N2 = tmpN2;
    N3 = tmpN3;
    NZ = tmpNZ;

}
