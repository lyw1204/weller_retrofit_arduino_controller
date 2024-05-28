#include <PID_v1.h>
#include "pins.h" //pin definition here
#include "station.h"

/*
void debug_write(){
  delay(100);
}
*/


Station myStation = Station(IN_POT, IN_CLK, IN_TMP, IN_LCK, OUT_PWR, OUT_LED1);



void setup() {
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  myStation.update();
}
