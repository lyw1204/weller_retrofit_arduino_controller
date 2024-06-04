#include "station.h"

//Button Reading routine
bool debounced_read(int btn) {
  static uint16_t state = 0;
  state = (state<<1) | digitalRead(btn) | 0xfe00;
  return (state == 0xff00);
}


Station::Station(uint8_t pot, uint8_t clk, uint8_t tmp, uint8_t lck, uint8_t out, uint8_t led)
{
    this->pot = pot;
    this->tmp = tmp;
    this->lck = lck; 
    this->out = out;

    pin_attach();

    myLed = new TriStateLED(led);
    myLed->setLED(RED);

    STATE_SET = 0; //Setpoint
    STATE_TMP = 0; //Iron Temp
    STATE_PWR_SW = 0; //Power Output
    STATE_LCK = false;

    cal_offset = 0; //Calibration Offset Value

    Kp=2, Ki=0, Kd=0;

    myPID = new PID(&STATE_TMP, &STATE_PWR_SW, &STATE_SET, Kp, Ki, Kd, DIRECT);
    myPID->SetMode(AUTOMATIC);


    myClk = new AC_Clock(clk);

    uint32_t last_write = 0;
    
}
//Subroutines
void Station::pin_attach(){
  pinMode(pot, INPUT);
  pinMode(clk, INPUT);
  pinMode(tmp, INPUT);
  pinMode(lck, INPUT_PULLUP);
  pinMode(out, OUTPUT);
}

void Station::read_tmp(){
    STATE_TMP = analogRead(tmp)/1024.0*CONST_TEMP_SENS + cal_offset;
}

void Station::read_setpoint(){ //Change setpoint by potentiometer
    if (!STATE_LCK)
        STATE_SET = analogRead(pot)/1024.0*CONST_DIAL;
}


void Station::set_cal_offset(double new_temp){
    cal_offset = new_temp - STATE_TMP;
}

void Station::tune_pid(double kp, double ki, double kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void Station::read_btn(){ //If button pressed, toggle lock state
    if (debounced_read(lck)) STATE_LCK = !STATE_LCK ;
}

void Station::heat(){
    if (myClk->in_time())
        digitalWrite(out, HIGH); //Pulls Thyristor Gate low
    else 
        digitalWrite(out, LOW); //Pulls Thyristor Gate high
}

void Station::led_update(){
    if (STATE_LCK) {
        myLed->setLED(RED);
        }
    else {
        if (STATE_TMP < STATE_SET) myLed->setLED(GREEN);
        else myLed->setLED(OFF);
    }
}

void Station::cycle_led(){
    myLed->setLED(GREEN);
    if (myLed->getLEDState() == GREEN){
        myLed->setLED(RED);
    
    }
    else if (myLed->getLEDState() == RED){
        myLed->setLED(GREEN);
    }

}

void Station::debug_write(){
    uint32_t now = millis();
    if (now-last_write > 1000){
        Serial.println(String(STATE_TMP)+","+String(STATE_SET)+","+String(STATE_PWR_SW)); 
        last_write = now;
    }
}


void Station::update(){

    led_update();
    //cycle_led();
    //Serial.println(myLed->getLEDState());
    read_btn();
    read_tmp();
    read_setpoint();
    myPID->Compute();
    myClk->set_duty(STATE_PWR_SW);
    myClk->update();

    heat();
    debug_write();

}
