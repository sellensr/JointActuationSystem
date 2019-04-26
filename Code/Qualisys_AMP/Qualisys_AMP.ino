#define VER               "0.07"  // started from Aurora All In One V0.07
#define ANG_SPEED         57600   // the angle data source
#define AP_SPEED          57600   // the actuator platform
#define CON_SPEED        115200   // the console
#define BUTTON_PIN           13   // a control button
#define DEBUG                 1   // set to 0 for console silent

char cr = 13;         // carriage return and line feed values for writing to ports
char lf = 10;

HardwareSerial* angPort = &Serial2;   // port A, possible fake on Serial1
HardwareSerial* apPort = &Serial1;    // port D

void setup() { 
  pinMode(BUTTON_PIN,INPUT);
  Serial.begin(CON_SPEED);          // the port for monitoring the process if a computer is watching
  apPort->begin(AP_SPEED);                                   // assume just powered on
  angPort->begin(ANG_SPEED);                                   // assume just powered on
}

void loop() {
  static unsigned long cnt = 0;
  if(angPort->available()){
    char c = angPort->read();       // read inbound from angle source
    if(c == '$'){                   // insert a \n before each $
      if(DEBUG > 0){
        Serial.print("\n");
        Serial.print(millis());
        Serial.print("  ");
        Serial.print(cnt++);
        Serial.print("  ");
      }
      apPort->write('\n');         // repeat out to actuator platform
    }
    if(DEBUG > 0) Serial.print(c);
    apPort->write(c);
  }
}
