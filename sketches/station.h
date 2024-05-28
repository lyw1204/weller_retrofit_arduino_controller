#ifndef STATION_H
#define STATION_H
#include <Arduino.h>
#include <PID_v1.h>
#include "ac_clock.h"
#include "threeStateLED.h"
//Math Constants
#define CONST_DIAL 850
#define CONST_TEMP_SENS 1000


bool debounced_read(int btn);


class Station{
    //Pin Definitions
    uint8_t pot, clk, tmp, lck, out, led;



    bool STATE_LCK;

    double cal_offset; //Calibration Offset Value

    double Kp, Ki, Kd;

    PID* myPID;
    TriStateLED* myLed;
    AC_Clock* myClk;

private:
    void pin_attach();
    void read_tmp();
    void read_setpoint();
    void read_btn();
    void read_clk();
    void heat();
    void led_update();

public:
    double STATE_SET; //Setpoint
    double STATE_TMP; //Iron Temp
    double STATE_PWR_SW;


    Station(uint8_t pot, uint8_t clk, uint8_t tmp, uint8_t lck, uint8_t out, uint8_t led);
    //Setters
    void set_cal_offset(double new_temp);
    void tune_pid(double kp, double ki, double kd);


    //Loop
    void update();
    void debug_write();
    void cycle_led();
};

#endif