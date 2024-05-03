#include <PID_v1.h>

#define IN_POT A2
#define IN_CLK A3
#define IN_TMP A4
#define IN_LCK 4

#define OUT_LED1 2 //red
#define OUT_LED2 3 //green

#define OUT_PWR 5 //heater
#define OUT_SPK 12 //beeper, maybe?

//Math Constants
#define CONST_DIAL 850
#define CONST_TEMP_SENS 1000

bool STATE_LED1 = 0;
bool STATE_LED2 = 0;
bool STATE_BUTTON = 0;
bool STATE_LOCK = 0;


double STATE_SET = 0; //Setpoint
double STATE_TMP = 0; //Iron Temp
double STATE_PWR_SW = 0;

double Kp=5, Ki=1, Kd=0;

PID mainPID(&STATE_TMP, &STATE_PWR_SW, &STATE_SET, Kp, Ki, Kd, DIRECT);



bool debounced_read(int btn) {
  static uint16_t state = 0;
  state = (state<<1) | digitalRead(btn) | 0xfe00;
  return (state == 0xff00);
}


void pin_attach(){
  pinMode(IN_POT, INPUT);
  pinMode(IN_CLK, INPUT);
  pinMode(IN_TMP, INPUT);
  pinMode(IN_LCK, INPUT_PULLUP);

  pinMode(OUT_LED1, OUTPUT);
  pinMode(OUT_LED2, OUTPUT);
  pinMode(OUT_PWR, OUTPUT);
  pinMode(OUT_SPK, OUTPUT);
}

void read_btn(){
  if (debounced_read(IN_LCK)){
    STATE_LOCK = !STATE_LOCK;
  }
}

double translate_sensor(double vSensor){
  //Calibration of sensor goes here :D
  double tSensor = vSensor;
  return tSensor;
}



void reads(){
  read_btn();
  //Read iron temp
  STATE_TMP = translate_sensor(analogRead(IN_TMP))/1024.0*CONST_TEMP_SENS;
  if (!STATE_LOCK){//Read setting pot, ignore if locked
    STATE_SET = analogRead(IN_POT)/1024.0*CONST_DIAL;
  }
}

void update_out(){
  //STATE_LED2 = !STATE_LED2;

  //output heater pwm signal
  analogWrite(OUT_PWR, STATE_PWR_SW);

  //indicators logic
  if (STATE_SET>=STATE_TMP){//Heating is on
    STATE_LED2 = true;
  }
  else{
    STATE_LED2 = false;
  }

  if (STATE_LOCK){//Locked state, constant red
    digitalWrite(OUT_LED1, 1);
    digitalWrite(OUT_LED2,0);
  }
  else{//Unlocked, green LED indicates heating
    digitalWrite(OUT_LED1, 0);
    digitalWrite(OUT_LED2, STATE_LED2);
  }
}

void debug_write(){
  Serial.println("Temp="+String(STATE_TMP)+"Set="+String(STATE_SET)+"Pwr="+String(STATE_PWR_SW));
  delay(100);
}



void setup() {
  Serial.begin(115200);
  pin_attach();
  mainPID.SetMode(AUTOMATIC);

}

void loop() {
  // put your main code here, to run repeatedly:
  reads();
  mainPID.Compute();
  update_out();

  debug_write();
}
