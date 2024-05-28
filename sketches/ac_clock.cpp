#include "ac_clock.h"

AC_Clock::AC_Clock(uint8_t pin){
    this->pin = pin;
    last_v = analogRead(pin);
    last_rise = millis();
    last_fall = millis();
    period = 0;
    duty = 0;
    //Default, no triggering possible; 
    trigger_start = last_fall;
    trigger_end = last_fall;

}

void AC_Clock::update(){
    uint32_t v2 = analogRead(pin);
        if (v2 < ADC_OFFSET xor last_v < ADC_OFFSET){ //zero cross time logging
            if (v2 > last_v) { // Rising Edge
                last_rise = millis();
                Serial.print("Rise");
                
                } 
            else { // Falling Edge
                last_fall = millis();
                Serial.println("Fall");
            } 
        } 
        
        period = 2 * abs(last_fall - last_rise);
        //Serial.println(v2);

        calc_trigger(duty);
        last_v = v2;
    }


void AC_Clock::set_duty(double duty) {this->duty = duty; }

//routines
void AC_Clock::calc_trigger(double duty){
    const double duty_max = 255;
    double percent_adv = (duty_max - duty)/duty_max;
    trigger_start = last_fall + percent_adv * (period/2); //Advance trigger period by 0-180 deg
    trigger_end = trigger_start + period;
    //Serial.println(percent_adv);

}

bool AC_Clock::in_time(){
    uint32_t now = millis();
    return (now > trigger_start and now < trigger_end);

}