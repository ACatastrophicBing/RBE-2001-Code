#include <Arduino.h>

class BlueMotor
{
public:
void setEffort(float effort);
long getPosition();
void setup();
float getVelocity();
void armPID(float target);
float setEffortWithoutDB(float effort);//I want it to return the effort it is putting in
private:
static void isr();

};