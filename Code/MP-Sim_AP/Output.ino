void waitForSerial(){
    int tStart = micros();
    while(Serial.availableForWrite() < 30){ 
      delayMicroseconds(147);
      if(micros()-tStart > 1000) break;   // give it a limited time     
    }
}

void showSettings(){
  Serial.print("\nHard Coded Settings\n");
  Serial.print("Tension Base/Incremental/Limit [N]: "); 
  Serial.print(START_TENSION);   Serial.print(" / ");
  Serial.print(INC_TENSION);     Serial.print(" / ");
  Serial.print(MAX_TENSION);     Serial.print("\n");
  waitForSerial();
  Serial.print("        Speeds Min/Default/Max [-]: "); 
  Serial.print(MIN_SPEED);       Serial.print(" / "); 
  Serial.print(DEF_SPEED);       Serial.print(" / ");
  Serial.print(MAX_SPEED);       Serial.print("\n");  

}

void showAngleStatus(unsigned mask) { //diagnostic info to monitor, no action taken!
  if(mask == 0) mask = 0xFF;          // default to printing everything
  bool verb = mask & verboseMask; 
  //verbose = false;
  updateMotorPos();
  waitForSerial();
  Serial.print(micros()/1000000.,3);
  waitForSerial();
  if(mask & tarMask){
    if(verb) Serial.print(" s\n      tarAng: ");
    for(int i = 0;i<3;i++){ 
      waitForSerial();
      Serial.print(", ");
      Serial.print(tarAng[i]);
    }
  }
  waitForSerial();
  if(mask & actMask){
    if(verb) Serial.print("\n      actAng: ");
    for(int i = 0;i<3;i++){ 
      waitForSerial();
      Serial.print(", ");
      Serial.print(actAng[i]);
    }
  }
  waitForSerial();
  if(mask & posMask){
    if(verb) Serial.print("\n   latestPos: ");
    for(int i = 0;i < NCH;i++){
      waitForSerial();
      Serial.print(", ");
      Serial.print(latestPos[i]);
    }
  }
  waitForSerial();
  if(mask & deltaMask){
    if(verb) Serial.print("\n    deltaPos: ");
    for(int i = 0;i < NCH;i++){
      waitForSerial();
      Serial.print(", ");
      Serial.print(deltaPos[i]);
    }
  }
  waitForSerial();
  if(mask & frcMask){
    if(verb) Serial.print("\n smoothForce: ");
    for(int i = 0;i < NCH;i++){
      waitForSerial();
      Serial.print(", ");
      Serial.print(smoothForce(i));
    }
  }
  Serial.print("\n");
  waitForSerial();
}

void showAnalogForce(){
  Serial.print("Press the button to return to command line\n\n");
  delay(2);
  float w = .01;
  float as[NCHIND] = {0};
  static unsigned long timeLastPrint = 0;
  unsigned long timeNow = micros();
  while(!buttonStatus(4)){
    timeNow = micros();
    for(int i = 0; i < NCH;i++){
      as[i] = (1.0-w) * as[i] + w * analogRead(_FRC[i]);
      if(timeNow-timeLastPrint > 1000000){
        waitForSerial();
        Serial.print(as[i],0);
        Serial.print(", ");
      }
    }
    if(timeNow-timeLastPrint > 1000000){
      Serial.print("Force values, smoothed");
      for(int i = 0;i<NCH;i++){
        Serial.print(", "); Serial.print(smoothForce(i));
      }
      Serial.print("\n");
      timeLastPrint = timeNow;
    }
    //Serial.print(timeLastPrint); Serial.print(", "); Serial.println(timeNow);
  }
}
