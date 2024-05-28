#include "threeStateLED.h"

TriStateLED::TriStateLED(uint8_t pin){
    this->pin = pin;
    setLED(OFF);
}

void TriStateLED::setLED(uint8_t newState){
    this->state = newState;
    switch(state){
        Serial.println("setLED Called: "+state);
        case GREEN:
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
            break;
        case OFF:
            pinMode(pin, INPUT);
            break;
        case RED:
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
            break;
        default:
            Serial.println("WTF");
            break;
    }

}

uint8_t TriStateLED::getLEDState(){
    return state;
}
