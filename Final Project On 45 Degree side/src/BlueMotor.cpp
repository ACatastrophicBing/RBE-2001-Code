#include <BlueMotor.h>
long count = 0;
unsigned long time = 0;
const int H1output = 3;
const int H2output = 2;
const int AIN2 = 4;
const int AIN1 = 13;
const int PWMA = 11;
long startTime = 0;
double velocity = 0;
const int X = 5; 
int newValue = 0;
long errorCount = 0;
int oldValue = 0;
long totalCount = 0;
unsigned long printStartTime;
long startingPOS = 0;
long velocityTime = 0;
int encoderArray[4][4] = {     
  {0, -1, 1, X},     
  {1, 0, X, -1},     
  {-1, X, 0, 1},     
  {X, 1, -1, 0}};
float ticksToRotationConstant = 1/540;
float radius = 6; // can change to gear shift constant thingy as well for the future currently 6 inches since thats what I think it is
float circumference = 2*(M_PI)*radius;
long timeTakenForChange = 0;
long prevCount = count;
float prevError = 0;
float Kdeffort = 0;//needed since I'm using a timer thingy in the code
double Kieffort = 0;
unsigned long pidTimer = 0;
float effort = 0;
//this section is filled with variables that will be used in final project
long fourFiveDegreePOS = 0; // in ticks to make it easy
long above45DegreePOSToDrop = fourFiveDegreePOS + 0;//in order to allow the plate to get over bolts
long sixZeroDegreePOS = 0; // in ticks to make it easy
long above60DegreePOSToDrop = sixZeroDegreePOS + 0;//in order to allow plate to get over bolts

void BlueMotor::setup(){
   pinMode(PWMA, OUTPUT);   
   pinMode(AIN2, OUTPUT);   
   pinMode(AIN1, OUTPUT); 
   pinMode(H1output, INPUT);
   pinMode(H2output, INPUT);
   TCCR1A = 0xA8; 
   TCCR1B = 0x11; 
   ICR1 = 400; 
   OCR1C = 0; 
   attachInterrupt(digitalPinToInterrupt(H1output),isr,CHANGE);
   attachInterrupt(digitalPinToInterrupt(H2output),isr,CHANGE);
}
void BlueMotor::isr(){
  newValue = (digitalRead(3) << 1) | digitalRead(2);
  int value = encoderArray[oldValue][newValue]; 
  if (value == X)   {     
    errorCount++;   
    } else {     
      count += value;   
      totalCount+=value;
      }   
      oldValue = newValue; 
}

void BlueMotor::setEffort(float effort){
  if(effort > 400){
    effort = effort/abs(effort)*400;
  }
  if(effort>0){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  OCR1C = effort;
  }else{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  OCR1C = -effort;
  }
}
float BlueMotor::setEffortWithoutDB(float effort){
  float zoom = effort;
  /*if((velocity == 0) && (abs(effort)<290)){//I NEED this since the motor sometimes needs a little kick to just get going
    //290 is the min speed needed to give it a jolt
    if(effort<0){
      zoom = -320;//but using 320 is just to be safe
      setEffort(zoom);
    }else{
      zoom = 320;
      setEffort(zoom);
      }
    */
  //}else{
    if(effort>400){
      effort = 400;
    }
    if(effort<-400){
      effort = -400;
    }
      if(effort<0){
        zoom = (float)effort*(400-150)/400-150;
      }else{
        zoom = (float)effort*(400-150)/400+150;
      }
    setEffort(zoom);
  //}
  return zoom;
}



unsigned long printTimerr = 0;
void BlueMotor::armPID(float target){//target is in degrees
//count = actual or totalCount = actual maybe
//if I reset count and don't change it then I'll need to use a previous count to get velocity, or I use totalCount to see how far off it is
//could also use count and have a start position, then an end position when I enter this maybe
float Kp = (float)400/2600;//
float currPosInches = (float)(totalCount-startingPOS)*1/540*360;
float error = (target - currPosInches);
float Ki = .05;
float Kd = .03;

float Kpeffort = Kp*error;//Doing 120 instead of actual drop off just to be safe
effort = (Kpeffort+Kieffort+Kdeffort);//effort is from -1 to 1 so I can multiply by 290 in the end to make it so it can easily reach from 120 to 400
  setEffortWithoutDB(effort);
  if(millis()>printTimerr){
    Kieffort = Ki*error*10;//since this is being called every 10ms, dT = 5
    Kdeffort = Kd*(error-prevError)/10;
    Serial.print("Current Position: ");
    Serial.print(totalCount*360/540);
    Serial.print(" Set Position: ");
    Serial.print(target);
    Serial.print(" Error: ");
    Serial.print(error);
    Serial.print(" Effort: ");
    Serial.print(effort);
    Serial.print(" Effort with DB: ");
    Serial.print(setEffortWithoutDB(effort));
    Serial.print(" Kp Value: ");
    Serial.print(Kp);
    Serial.print(" Ki Value: ");
    Serial.print(Ki);
    Serial.print(" Kd Value: ");
    Serial.println(Kd);
    printTimerr = millis()+10;
  }
}




float BlueMotor::getVelocity(){//returns rotations per minute
    long timeTaken = millis()-velocityTime;
    velocity = (float)(count-prevCount)/timeTaken/540*60*1000;//1 rotation per 540 ticks, converting from ticks per ms to rotations per minute
    velocityTime = millis();
    return velocity;
}


long BlueMotor::getPosition(){//returns pos in degrees
return (float)totalCount/540*360;
}
