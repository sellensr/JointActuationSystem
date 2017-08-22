// doubleQ declares a quaternion of the form {q0, qx, qy, qz}, initialized to no rotation
#define doubleQ(a) double a[4] = {1.0d, 0.0, 0.0, 0.0}
#define doubleT(a) double a[4][4] = {1.d, 0.d, 0.d, 0.d, 0.d, 1.d, 0.d, 0.d, 0.d, 0.d, 1.d, 0.d, 0.d, 0.d, 0.d, 1.d}
/*
 *  Aurora All In One -- doesn't use XBee communication
 *  
 *  
 *  
 */
 // Because the transmitted packets are long, bump up the serial buffer sizes in the serial1.c, serial2.c, etc. files
 // See Controller for details! *************************IMPORTANT**************

#define VER               "0.07"
#define AURORA_SPEED      57600
#define XBEE_SPEED        57600
#define DUE_SPEED         57600
#define CON_SPEED        115200
#define BUTTON_PIN           13

char cr = 13;         // carriage return and line feed values for writing to ports
char lf = 10;

char scrAB[500], scrBA[500];    // scratch strings for collecting characters going from 2 to 3 and 3 to 2
char scratchy[500];             // scratch string with no durability -- anybody can use it
unsigned scrABIndex = 0;        // Next open position in each of the strings
unsigned scrBAIndex = 0;

HardwareSerial* aurPort = &Serial2;   // port A, possible fake on Serial1
HardwareSerial* duePort = &Serial1;   // port D

void setup() { 
  pinMode(BUTTON_PIN,INPUT);
  Serial.begin(CON_SPEED);          // the port for monitoring the process if a computer is watching
  duePort->begin(DUE_SPEED);                                   // assume just powered on
  delay(5000);

  Serial.print("\n\nAuroraAllInOne: ");
  Serial.print("  Version: "); Serial.println(VER);

  // start the communications ports
  // look for a real Aurora on the RS232 on Serial2, if not found settle for a fake on Serial1
  aurPort = &Serial2;
  aurPort->begin(9600);                                   // assume just powered on
  while(aurPort->available()){
    char c = aurPort->read();   // flush the incoming buffer
    Serial.print(c);
  }
  aurPort->print("TX "); aurPort->print(cr);              // send a command -- should yield an error, or something
  Serial.print("\n TX sent to Serial2 -- waiting...\n");
  delay(1000);
  if(aurPort->available()){                               // flush the incoming buffer response and leave on Serial2
    while(aurPort->available()){ 
      char c = aurPort->read(); 
      Serial.print(c);   
    }
  } else{
    Serial.print("closing Serial2  ");
    aurPort->end();
    aurPort = &Serial3;
    Serial.println("opening Serial1 at 57600  ");
    aurPort->begin(57600);
  }

  // initialize the Aurora to run at 57600
  setupAuroraSpeed(aurPort);
  setupAurora(aurPort);
  
}

void loop() {
  static unsigned long lastOpenButton = 0;
  static unsigned long lastTX = 0;
  static char buff[500];
  static unsigned buffIndex = 0;

  if(!digitalRead(BUTTON_PIN)) lastOpenButton = millis();
  // Send a TX requesting more data if it has been a while this will either start the 
  // exchange, or get it restarted if it gets out of synch
  if(millis()-lastTX > 5000 || (millis()-lastOpenButton > 20 && millis()-lastTX > 500)){ 
    if(Serial.availableForWrite()){ Serial.print(millis()); Serial.print(": Sending a TX to resynch\n");}
    sendTX(aurPort);
    lastTX = millis();
  }
  if(millis() - lastOpenButton > 500){
    if(Serial.availableForWrite()){ Serial.print(millis()); Serial.print(": Calling setup() to restart\n");}
    setup();
  }
  
  //now act like a controller listening to the Aurora data coming from the XBee
  //get the XBee Data coming up from the link
  if(aurPort->available()){
    char c = aurPort->read();
    //if(Serial.availableForWrite()) Serial.print(c);
    buff[buffIndex] = c;              // add the character to the string
    if(c == cr || buffIndex > 450){   // terminate the string if it is a <CR> or too long
      //Serial.print("\nfound <CR> terminated string:\n");
      buff[buffIndex] = 0;
      buffIndex = 0;                  // start again for the next string
      parseTX(buff);
      // since this is the end of a response packet, then send another TX and act on the packet
      sendTX(aurPort);
      lastTX = millis();
    }else buffIndex++;
  }


  // check for requests from the DUE and respond  
}

void setupAuroraSpeed(HardwareSerial* port){
  // change the speed to 115200 and report on console
    askAnswer(port,(char*) "COMM 50000",cr,5000);  // Set the baud rate if different from the 9600 baud default
                                           // COMM 40000 is 57600 baud, 00000 is 9600 baud, 50000 is 115200 baud
    port->end(); delay(5000);              // reset the port on the microcontroller to match and wait for settling
    port->begin(115200); delay(5000);
//  // change the speed to 57600 and report on console
//    askAnswer(port,(char*) "COMM 40000",cr,5000);  // Set the baud rate if different from the 9600 baud default
//                                           // COMM 40000 is 57600 baud, 00000 is 9600 baud, 50000 is 115200 baud
//    port->end(); delay(5000);              // reset the port on the microcontroller to match and wait for settling
//    port->begin(57600); delay(5000);
}

void setupAurora(HardwareSerial* port){
  // run Aurora setup sequence and report on console
  askAnswer(port,(char*) "VER:4A6EF",cr,1000);   // get the firmware version information
  askAnswer(port,(char*) "INIT ",cr,5000);       // initialize the system
  askAnswer(port,(char*) "PHSR 00",cr,1000);     // report all allocated port handles
  askAnswer(port,(char*) "PHSR 01",cr,1000);     //        need to be freed
  askAnswer(port,(char*) "PHSR 02",cr,1000);     //        occupied, but not initialized
  askAnswer(port,(char*) "PHSR 03",cr,1000);     //        occupied, initialized, but not enabled
  askAnswer(port,(char*) "PHSR 04",cr,1000);     //        enabled

  askAnswer(port,(char*) "PHINF 0A0025",cr,1000);// get information on port 0A for location0020, part0004, and information 0001
  askAnswer(port,(char*) "PINIT 0A",cr,5000);    // initialize port handle 0A
  askAnswer(port,(char*) "PHSR 02",cr,1000);
  askAnswer(port,(char*) "PHSR 03",cr,1000);
  askAnswer(port,(char*) "PENA 0AD",cr,1000);    // enable port 0A as a dynamic tool
  askAnswer(port,(char*) "PHINF 0A0025",cr,1000);

  askAnswer(port,(char*) "PHINF 0B0025",cr,1000);// port 0B
  askAnswer(port,(char*) "PINIT 0B",cr,5000);    
  askAnswer(port,(char*) "PHSR 02",cr,1000);
  askAnswer(port,(char*) "PHSR 03",cr,1000);
  askAnswer(port,(char*) "PENA 0BD",cr,1000);   
  askAnswer(port,(char*) "PHINF 0B0025",cr,1000);

  askAnswer(port,(char*) "PHINF 0C0025",cr,1000);// port 0C
  askAnswer(port,(char*) "PINIT 0C",cr,5000);    
  askAnswer(port,(char*) "PHSR 02",cr,1000);
  askAnswer(port,(char*) "PHSR 03",cr,1000);
  askAnswer(port,(char*) "PENA 0CD",cr,1000);   
  askAnswer(port,(char*) "PHINF 0C0025",cr,1000);

  askAnswer(port,(char*) "TSTART ",cr,5000);
}

void askAnswer(HardwareSerial* port, char* ask, char eol, unsigned long wait){
  // send the ask string to the port with eol if non-zero, then wait for an answer and echo it to the console
  unsigned long lastChar = millis();
  Serial.print(ask); Serial.print(": ");
  port->print(ask);
  if(eol != 0) port->print(eol);
  // wait up to wait milliseconds for the first character
  while(millis()-lastChar < wait && !port->available()){
    if(millis()%100 == 0){
      Serial.print("."); 
      delay(1);
    }
  }
  //wait a while between characters
  lastChar = millis();
  while(millis()-lastChar < 500){
    if(port->available()){
      char c = port->read(); 
      Serial.print(c);
      lastChar = millis();
    }
  }
  Serial.print("\n");
}

// The globally shared data resulting from parsing the Aurora data
  long frameNo[10];
  long portStatus[10];
  float indVal[10];
  float tx[10];
  float ty[10];
  float tz[10];
  float q0[10];
  float qx[10];
  float qy[10];
  float qz[10];
  unsigned handle[10];

int parseTX(char *buff){  // return number of sensors
  // parses buff destructively in place to extract the data 
  static unsigned long lastPrinted = 0;
  static unsigned sinceLast = 0;
  byte printThisTime = 0;
  char *l[10];
  if(millis()-lastPrinted > 200){ 
    printThisTime = 1;
    lastPrinted = millis();
  } else printThisTime = 0;

  int n = buff[1] - '0';    // the number of active sensors
  int len = 0;
  while(buff[len]) len++;
  if(len / n < 71){
    if(printThisTime){ 
      Serial.print("\n A string that's too short to parse might crash the process -- rejecting\nProbably just missing one or more of the targets\n");
      Serial.println(buff);
    }
    return 0;
  }
  if(printThisTime == 1){ 
    Serial.print(n); Serial.print(" sensors, ");
    Serial.print(millis()/1000); Serial.print(" total seconds, in parseTX after: "); Serial.print(sinceLast); 
    Serial.println(" packets skipped. received:");sinceLast = 0; 
    Serial.println(buff);
    Serial.println("End of received packet");   
  } else sinceLast++;
  l[0] = buff + 2;      // point to first line
  *(l[0]+69) = 0;       // terminate first line
    
  for(int i = 1;i < n;i++){ 
    l[i] = l[i-1] + 70; // point to each additional line
    *(l[i]+69) = 0;     // terminate each line 
  }
  for(int i = 0; i < n;i++){  //parse each line
    // Frame number is 8 hex characters at 61-68
    frameNo[i] = strtol(l[i]+61,NULL,16);
    *(l[i]+61)=0;
    // Port status is 8 hex characters at 53-60
    portStatus[i] = strtol(l[i]+53,NULL,16);
    *(l[i]+53)=0;
    // Error indicator is 6 characters at 47-52
    indVal[i] = strtol(l[i]+47,NULL,10) /10000.;
    *(l[i]+47)=0;
    // TZ is seven characters at 40-46
    tz[i] = strtol(l[i]+40,NULL,10) /100.;
    *(l[i]+40)=0;
    // TY is seven characters at 33-39
    ty[i] = strtol(l[i]+33,NULL,10) /100.;
    *(l[i]+33)=0;
    // TX is seven characters at 26-32
    tx[i] = strtol(l[i]+26,NULL,10) /100.;
    *(l[i]+26)=0;
    // QZ is six characters at 20-25
    qz[i] = strtol(l[i]+20,NULL,10) /10000.;
    *(l[i]+20)=0;
    // QY is six characters at 14-19
    qy[i] = strtol(l[i]+14,NULL,10) /10000.;
    *(l[i]+14)=0;
    // QX is six characters at 8-13
    qx[i] = strtol(l[i]+8,NULL,10) /10000.;
    *(l[i]+8)=0;
    // Q0 is six characters at 2-7 
    q0[i] = strtol(l[i]+2,NULL,10) /10000.;
    *(l[i]+2)=0;
    // Handle is two hex characters at 0-1 
    handle[i] = strtol(l[i],NULL,16);
    if(printThisTime == 1){ 
      Serial.print(handle[i]); Serial.print(" q0xyz: ");
      Serial.print(q0[i],4); Serial.print(" ");         Serial.print(qx[i],4); Serial.print(" ");     Serial.print(qy[i],4); Serial.print(" "); 
      Serial.print(qz[i],4); Serial.print(" txyz: ");         Serial.print(tx[i]); Serial.print(" ");     Serial.print(ty[i]); Serial.print(" "); 
      Serial.print(tz[i]); Serial.print(" status: ");         Serial.print(indVal[i]); Serial.print(" "); 
      Serial.print(portStatus[i]); Serial.print(" "); Serial.print(frameNo[i]); Serial.print("\n");
    }
  }
  doubleQ(q01);
  doubleQ(q02);
  doubleQ(qc01);
  doubleQ(q12);       // the quaternion from the first sensor to the second sensor
  doubleT(T12);
  doubleT(T21);
  double eu1[3],eu2[3];
  q01[0] = q0[0]; q01[1] = qx[0]; q01[2] = qy[0]; q01[3] = qz[0];
  q02[0] = q0[1]; q01[1] = qx[1]; q01[2] = qy[1]; q01[3] = qz[1];
  conjQ(q01,qc01);
  multQ(q02,qc01,q12);
  qRot(q12,T21);
  transposeT(T21,T12);
  rEuler(T12,eu1,eu2);
  //calculate Aurora angles
  double th[3],thy[3],thx[3];
  for(int i = 0;i<3;i++){
    th[i] = 2 * acos(q0[i]);
    double sth2 = sin(th[i] / 2.0);
    thx[i] = qx[i] / sth2 * th[i];
    thy[i] = qy[i] /sth2 * th[i];    
  }
  // calculate RU and FE angles as differences and convert to degrees
  // Sense is positive for sensors facing away from generator, negative for facing towards.
  double ruAngle = -1.0 * (thx[1]-thx[0]) * 57.3;
  while(ruAngle < -180.) ruAngle += 180.; //put in +/- 180 degree zone
  while(ruAngle > 180.) ruAngle -= 180.;
  double feAngle = -1.0 * (thy[1]-thy[0]) * 57.3;
  while(feAngle < -180.) feAngle += 180.; //put in +/- 180 degree zone
  while(feAngle > 180.) feAngle -= 180.;
  int ruda = ruAngle * 100;     // put in hundredths as integer
  int fea = feAngle * 100;
  int relFea = 0;
  int relRuda = 0;
  char matty[100];
  //sprintf (matty, "$%d,%d,%d,%d\n",ruda, fea, relRuda, relFea); 
  sprintf (matty, "$%d,%d\n",ruda, fea); // new version takes only two values 
  duePort->print(matty);
  if(printThisTime == 1){ 
    Serial.print(matty);
    Serial.print("FE / RU [deg] = "); Serial.print(feAngle);Serial.print(" / ");Serial.println(ruAngle); 
    }   
  if(printThisTime == 1) Serial.println("End of parsed packet values\n");   
  return n;
}

void sendTX(HardwareSerial* port){
  port->print("TX "); port->print(cr);
}


