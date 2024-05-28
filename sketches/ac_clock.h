#ifndef AC_CLOCK_H
#define AC_CLOCK_H
#include "Arduino.h"


#define ADC_MAX 1024
#define ADC_OFFSET 512


class AC_Clock
{
private:
    /* data */
    uint8_t pin;

    uint32_t last_fall = 0;
    uint32_t last_rise = 0;
    int last_v = 1024;

    uint32_t period = 0;

    double duty = 0;
    
    void calc_trigger(double duty);


public:
    uint32_t trigger_start = 0;
    uint32_t trigger_end = 0;

    AC_Clock(uint8_t pin);

    void update();
    void set_duty(double duty);
    bool in_time();
};

#endif