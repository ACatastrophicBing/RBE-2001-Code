#include <Romi32U4.h>
#include <BlueMotor.h>
#include <Chassis.h>
#include <QTRSensors.h>
#include <Rangefinder.h>
#include "IRdecoder.h"
#include "servo32u4.h"

servo32U4 servo;

IRDecoder decoder;
Romi32U4ButtonA pb;
QTRSensors qtr;
const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];
Romi32U4Motors motors;
BlueMotor blu;
Chassis turner;
Rangefinder rangerDan;
bool paused = false;
int lineFollowSpeed = 65;

const uint16_t remoteAddressByte0 = 0x00; 
const uint16_t remoteAddressByte1 = 0xBF; 
const uint8_t remoteVolMinus = 0x00; 
const uint8_t remotePlayPause = 0x01; 
const uint8_t remoteVolPlus = 0x02; 
const uint8_t remoteSetup = 0x04; 
const uint8_t remoteUp = 0x05; 
const uint8_t remoteStopMode = 0x06; 
const uint8_t remoteLeft = 0x08; 
const uint8_t remoteEnterSave = 0x09; 
const uint8_t remoteRight = 0x0A; 
const uint8_t remote0 = 0x0C; 
const uint8_t remoteDown = 0x0D; 
const uint8_t remoteBack = 0x0E;
const uint8_t remote1 = 0x10; 
const uint8_t remote2 = 0x11; 
const uint8_t remote3 = 0x12; 
const uint8_t remote4 = 0x14; 
const uint8_t remote5 = 0x15; 
const uint8_t remote6 = 0x16; 
const uint8_t remote7 = 0x18; 
const uint8_t remote8 = 0x19; 
const uint8_t remote9 = 0x1A;
  
long counterStartTime = 0;

void printLineFollowNums(){
  qtr.read(sensorValues);
  int rightSensor = sensorValues[0];
  //int black = 2500;
  //int white = 100;
  Serial.print("Right ");
  Serial.println(rightSensor);
  qtr.read(sensorValues);
  int leftSensor = sensorValues[1];
  Serial.print("Left ");
  Serial.println(leftSensor);
}
bool lineFollow(){
  qtr.read(sensorValues);
  int rightSensor = sensorValues[0];
  //int black = 2500;
  //int white = 100;
  //Serial.print("Right ");
  //Serial.println(rightSensor);
  qtr.read(sensorValues);
  int leftSensor = sensorValues[1];
  //Serial.print("Left ");
  //Serial.println(leftSensor);
  float errorRight = rightSensor-500;//100 is the target(ground), black is 2500, so max average diff of lets say 1800*Kp = 75, Kp = .03125
  float errorLeft = leftSensor-500;
  float constantSpeed = 65; 
  float KpLineFollow = 0.04;
  float speedRight = constantSpeed+errorLeft*KpLineFollow;//when error is high(sensing black), motor turns faster
  float speedLeft = constantSpeed+errorRight*KpLineFollow;
  motors.setEfforts(speedLeft,speedRight);
  return (leftSensor>1700)&&(rightSensor>1700);//returns true if both are sensing black, so that means that in a loop I need a ! before it
}

bool lineFollowDistanceSense1(float dist){
  qtr.read(sensorValues);
  int rightSensor = sensorValues[0];
  qtr.read(sensorValues);
  int leftSensor = sensorValues[1];
  float errorRight = rightSensor-100;//100 is the target(white), black is 2500, so max average diff of lets say 1800*Kp = 75, Kp = .03125
  float errorLeft = leftSensor-100;
  float distanceError = dist-8.3;
  float KpDistanceSense = .125;
  float EffortDistance = KpDistanceSense*distanceError;
  //Serial.println(EffortDistance);
  float KpLineFollow = 0.015;
  float speedRight = 40+EffortDistance*(50+errorLeft*KpLineFollow);//when error is high(sensing black), motor turns faster
  float speedLeft = 40+EffortDistance*(50+errorRight*KpLineFollow);
  Serial.print("LEFT: ");
  Serial.println(speedLeft);
  Serial.print("RIGHT: ");
  Serial.println(speedRight);
  motors.setEfforts(speedLeft,speedRight);
  if(dist>8.2&&dist<8.4){
    return true;
  }
  return false;
}
bool goin = false;
void stopBlu(){
  delay(2);
  if(decoder.getKeyCode() == remoteEnterSave){
    blu.setEffort(0);
    Serial.println("Stopping blu motor");
  }
}
void checkRemote(){
  delay(2);
  if(decoder.getKeyCode() == remotePlayPause){
    goin = true;
    Serial.println("Paused Button Sensed, so that means it can start");
  }
}
void checkMainStop(){
  delay(2);
  if(decoder.getKeyCode() == remote2){
    goin = false;
    Serial.println("PLEASE STOP AND COMMIT UWU");
  }
}
bool checkForPlay(){
  delay(2);
  if(decoder.getKeyCode() == remoteUp){
    return true;
  }return false;
}

void moveDown(){
  delay(2);
  if(decoder.getKeyCode() == remoteVolMinus){
    blu.setEffort(-400);
    Serial.println("Moving Up");
  }
}

void moveUp(){
  delay(2);
  if(decoder.getKeyCode() == remoteVolPlus){
    blu.setEffort(400);
    Serial.println("Moving Down");
  }
}

void setup() {  
    qtr.setTypeRC();//Setting up line sensor
    qtr.setSensorPins((const uint8_t[]){20,21}, SensorCount);
    qtr.setEmitterPin(19);
    decoder.init();

    servo.SetMinMaxUS(900, 2100);
    pinMode(18, INPUT);
    servo.Init();
    servo.Attach();


    rangerDan.setup();
    blu.setup();
    Serial.begin(9600);
    Serial.println("Starting the code woo");
  }


bool CLAWSTATE = false;//false is open, true is closed
int preval = 0;
int newval = 0;
bool button = false;
unsigned long deadbandTimer = millis()+1000;
unsigned long printTimer = millis()+100;
long deadbandConstant = -40;
bool backnForth = true;
unsigned long armTimer = 0;

int lineFollowState = 0;
bool timer = true;
enum statemachine {LIFT_ARM_45,TURN_180,DRIVE_WAIT_FOR_BLACK,
  LINE_FOLLOW_WAIT_FOR_BLACK,TURN_90,LINE_FOLLOW_DISTANCE_SENSE,ARM_PICKUP,LIFT_ARM_25,OPEN_CLAW,CLOSE_CLAW,WAIT_FOR_PLAY};
int stateInNow = WAIT_FOR_PLAY;//WAIT_FOR_PLAY so that means that it will first
//assuming arm always starts at the exact angle necessary to pickup from pickup zone
//This is the code to do the 45 degree side

bool turnToLine(){
  turner.turnAngleNonBlock(180);
  qtr.read(sensorValues);
  int leftSensor = sensorValues[1];
  if(leftSensor<2000){
        turner.turnAngleNonBlock(180);
      }else{
        motors.setEfforts(0,0);
        return false;
      }
      return true;
}
bool openClaw(){
  servo.Write(900);//need a good close POS
  delay(500);
  return true;
}
bool closeClaw(){
  servo.Write(1433);
  delay(500);
  return true;
}

void loop(){
/*for(int i = 1100; i<1900;i+=100){
  servo.Attach();
  servo.Write(i);
  delay(100);
  servo.Detach();
}*/
/*openClaw();
delay(500);
closeClaw();
delay(500);
*/
  //rangerDan.getDistanceCM();
  checkRemote();
  //Serial.println("Here");
  moveDown();
  moveUp();
  stopBlu();
  //openClaw();
  //closeClaw();
  //printLineFollowNums();
  //lineFollowDistanceSense1(rangerDan.getDistanceCM());
  //turnToLine();
  //if(paused){
    //Serial.println("Paused Clicked");
  //}
  if(goin){//so I can test tidbits of code and use IR receiver, paused is true when going
  switch(stateInNow){
    case LIFT_ARM_45://lift arm
      blu.armPID(4860);//distance that the arm has to move
      if((blu.getPosition()>4855)&&(blu.getPosition()<4865)&&(timer)){//to start timer to make sure arm is in correct position
        armTimer = millis();
        timer = false;
        Serial.println("LIFTING ARM 45");
      }
      if((blu.getPosition()>4855)&&(blu.getPosition()<4865)&&(armTimer+250<millis())){
        stateInNow = TURN_180;//then turn around
        blu.setEffort(0);
        timer = true;
      }
      break;
    case TURN_180://arm is already lifted, solar panel in arm, now needs to turn around (180 degree turn)
      if(turnToLine()){
        Serial.print("Turning 180");
      }
        else{
        motors.setEfforts(0,0);
        Serial.println("180 has been turnt");
        stateInNow = LINE_FOLLOW_WAIT_FOR_BLACK;
      }
      break;
    case DRIVE_WAIT_FOR_BLACK://line follow
      if(!lineFollow()){
        motors.setEfforts(100,100);
      }else{
        motors.setEfforts(0,0);
        Serial.println("Line Sensed");
        stateInNow = DRIVE_WAIT_FOR_BLACK;
      }
      break;
    case LINE_FOLLOW_WAIT_FOR_BLACK://drive forwards and wait for other black line
      if(lineFollow()){//lineLeft == black && lineRight == black, if black then it stops and changes states
        motors.setEfforts(0,0);
        Serial.println("Line Crossing Sensed");
        stateInNow = TURN_90;
      }else{
        lineFollow();
      }
      break;
    case TURN_90://90 degree turn either left or right, if right just change signs of degrees
      if(turner.turnAngleNonBlock(90)){
        turner.turnAngleNonBlock(90);
      }else{
        motors.setEfforts(0,0);
        Serial.println("Turned 90");
        stateInNow = LINE_FOLLOW_DISTANCE_SENSE;
      }
      break;
    case LINE_FOLLOW_DISTANCE_SENSE://line follow and distance sense
      if(!lineFollowDistanceSense1(rangerDan.getDistanceCM())){
        Serial.println("Distance Sense Line Following");
      }else{
        Serial.println("Set up for drop off");
        stateInNow = WAIT_FOR_PLAY;
      }
      break;
    case ARM_PICKUP://bring arm back to position 0
      blu.armPID(0);//distance that the arm has to move
      if((blu.getPosition()>-5)&&(blu.getPosition()<5)&&timer){//to start timer to make sure arm is in correct position
        armTimer = millis();
        timer = false;
      }
      if((blu.getPosition()>-5)&&(blu.getPosition()<5)&&(armTimer+250<millis())){
        stateInNow = WAIT_FOR_PLAY;//need to figure out what is next
        Serial.println("Arm at Pickup");
        timer = true;
      }
      break;
    case LIFT_ARM_25://bring arm to 25 degree angle
      blu.armPID(7200);//distance that the arm has to move
      if((blu.getPosition()>7195)&&(blu.getPosition()<7205)&&timer){//to start timer to make sure arm is in correct position
        armTimer = millis();
        timer = false;
      }
      if((blu.getPosition()>7195)&&(blu.getPosition()<7205)&&(armTimer+20<millis())){
        stateInNow = 2;//then turn around
        Serial.println("Arm At 25");
        timer = true;
      }
      break;
    case CLOSE_CLAW://close claw, needs to keep claw closed so always in a state of closing since not in a good point
      closeClaw();
      Serial.println("Claw Closed");
      stateInNow = LIFT_ARM_45;
      break;
    case OPEN_CLAW://open claws
      openClaw();
      Serial.print("Claw Opened");

      break;
    case WAIT_FOR_PLAY:
      if(checkForPlay()){//will only change state once the up button is pressed
      Serial.println("Play has been Pushed");
        if(CLAWSTATE){//if true open claw, if false close claw since CLAWSTATE is false when open, true when closed
          stateInNow = OPEN_CLAW;
          CLAWSTATE = !CLAWSTATE;
          Serial.println("Opening Claw");
        }else{
          stateInNow = CLOSE_CLAW;
          CLAWSTATE = !CLAWSTATE;
          Serial.println("Closing Claw");
        }
      }
      break;
  }
  }
}