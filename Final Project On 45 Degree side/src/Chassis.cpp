#include "Chassis.h"
int startPOS = 0;
float CprevError = 0;
unsigned long printTimerrr = 0;
float CKieffort = 0;
float CKdeffort = 0;


void Chassis::driveDistance(float inches)
{
    motors.setEfforts(100, 100);
    delay(inches * msPerInch);
    motors.setEfforts(0, 0);
}

void Chassis::turnAngle(float degrees)
{
    motors.setEfforts(100, -100);
    delay(degrees * msPerDegree);
    motors.setEfforts(0, 0);
}

bool Chassis::turnAngleNonBlock(float target)//sends in degrees
{
float CKp = (float)3/wheelTrack;
float CKi = 1.5/wheelTrack;
float CKd = .75/wheelTrack;
float errorLeft = target/300*wheelTrack*M_PI - (float)wheelDiameter*M_PI*encoders.getCountsLeft()/CPR;//running off of left motor but can change the code to include right motor but I feel like that is unecessary
float Kpeffort = CKp*errorLeft;//Doing 120 instead of actual drop off just to be safe
float leftEffort = 35+(Kpeffort+CKieffort+CKdeffort);
Serial.println(leftEffort);
motors.setEfforts(leftEffort,-leftEffort);

if(millis()>printTimerrr){
    CKieffort = CKi*errorLeft*10;//since this is being called every 10ms, dT = 5
    CKdeffort = CKd*(errorLeft-CprevError)/10;
    printTimerrr = millis()+10;
}

if(errorLeft>-0.02&&errorLeft<0.02){
 motors.setEfforts(0, 0);
 encoders.getCountsAndResetLeft();
 encoders.getCountsAndResetRight();
 return false;//driving is done
}
return true;//runs using a bool and returns true when not done
}
