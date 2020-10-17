#pragma once

#include <Romi32U4.h>

class Rangefinder 
{ 
    public:
        //Rangefinder(int triggerPin, int echoPin);   
        void setup();   
        float getDistanceCM();
        //int getEchoPin();
    
    private:
        static const int triggerPin = 12;
        static const int echoPin = 0;

        //int triggerPin;
        //int echoPin;
        static long startTime;
        static long duration;

        static void ultrasonicISR();
};

//extern Rangefinder us1 = Rangefinder(12, 0);