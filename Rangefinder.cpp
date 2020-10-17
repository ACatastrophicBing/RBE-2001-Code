#include "Rangefinder.h"

long Rangefinder::startTime = 0;
long Rangefinder::duration = 0;

/**
Rangefinder::Rangefinder(int triggerPin, int echoPin){
    triggerPin = triggerPin;
    echoPin = echoPin;
}

void Rangefinder::ultrasonicISR() {
    if(digitalRead(us1.getEchoPin()) == HIGH){//checks if echoPin is HIGH
        startTime = micros();
    }
    else if(digitalRead(us1.getEchoPin()) == LOW){
        duration = micros() - startTime;
    }
}


int Rangefinder::getEchoPin(void){
    return echoPin;
}**/

void Rangefinder::ultrasonicISR() {
    if(digitalRead(Rangefinder::echoPin) == HIGH){//checks if echoPin is HIGH
        Rangefinder::startTime = micros();
    }
    else if(digitalRead(Rangefinder::echoPin) == LOW){
        Rangefinder::duration = micros() - Rangefinder::startTime;
    }
}


void Rangefinder::setup(void)
{
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(echoPin), Rangefinder::ultrasonicISR, CHANGE);
}


float Rangefinder::getDistanceCM(void){
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH); //measures in micro seconds
    float distance = duration * 0.034 / 2; //speed of sound is .034 cm/microsec
    Serial.println(distance);
    return distance;
}