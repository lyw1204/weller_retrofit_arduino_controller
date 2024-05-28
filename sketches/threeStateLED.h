#ifndef THREE_STATE_LEDS
#define THREE_STATE_LEDS

#include <Arduino.h>

#define GREEN 0
#define OFF 1
#define RED 2

class TriStateLED{
    private:
        uint8_t pin;
        uint8_t state;

    public:
        TriStateLED(uint8_t pin);
        void setLED(uint8_t newState);
        uint8_t getLEDState();
};


#endif