#include <Arduino.h>
#include <Romi32U4.h>

class Chassis
{
public:
 void driveDistance(float inches);
 void turnAngle(float degrees);
 bool turnAngleNonBlock(float degrees);
 const float wheelDiameter = 2.8;
 const int CPR = 1440;
 const float wheelTrack = 5.75;
private:
 Romi32U4Motors motors;
 Romi32U4Encoders encoders;
 const float msPerDegree = 6.655987;
 const float msPerInch = 136.2;
}; 