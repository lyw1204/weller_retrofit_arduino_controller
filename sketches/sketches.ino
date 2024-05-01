#define IN_POT A2
#define IN_CLK A3
#define IN_TMP A4
#define IN_LCK 4

#define OUT_LED1 2 //red
#define OUT_LED2 3 //green

#define OUT_PWR 5 //heater
#define OUT_SPK 12 //beeper, maybe?

//Math Constants
#define DIAL_SCALE 850


bool STATE_LED1 = 0;
bool STATE_LED2 = 0;
bool STATE_BUTTON = 0;
bool STATE_LOCK = 0;




float STATE_SET = 0;

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

void update_led(){
  STATE_LED2 = !STATE_LED2;


  analogWrite(OUT_PWR, STATE_SET*255.0/DIAL_SCALE);

  if (STATE_LOCK){
    digitalWrite(OUT_LED1, 1);
    digitalWrite(OUT_LED2,0);
  }
  else{
    digitalWrite(OUT_LED1, 0);
    digitalWrite(OUT_LED2, STATE_LED2);
  }
}

void reads(){
  if (!STATE_LOCK){//ignore pot update if locked
    STATE_SET = analogRead(IN_TMP)/1024.0*DIAL_SCALE;
  }
}
void read_btn(){
  if (debounced_read(IN_LCK)){
    STATE_LOCK = !STATE_LOCK;
  }
}


void debug_write(){
  Serial.println("Temp="+String(STATE_SET)+"Lock="+String(STATE_LOCK));

}



void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pin_attach();
}

void loop() {
  // put your main code here, to run repeatedly:
  reads();
  read_btn();
  update_led();

  debug_write();
}
