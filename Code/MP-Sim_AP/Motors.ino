/////goToPos() moves a specific motor towards a set goal position at set speed within a user set RANGE
// Needs to be called often to make sure speed reduces when you get within range of the position.
// Set speed to zero after a run of these calls.
void goToPos(int motor, int goalPos, int spd = DEF_SPEED) {
  int RANGE = 1;
  updateForce();
  goalPos = max(goalPos, inPos[motor]);                     // force inside position limits
  goalPos = min(goalPos, outPos[motor]);
  spd = max(MIN_SPEED, abs(spd));                           // make positive and force inside speed limits
  spd = min(spd, MAX_SPEED); 
  if (latestForce[motor] > MAX_TENSION) spd = MIN_SPEED;    // ease tension slowly, +spd is extending
  else {                                                    // go in the right direction
    if (smoothMotorPos(motor) > (goalPos + RANGE))          // If current Pos is more than goal
      spd = -spd;                                            //   go backwards
    else if (smoothMotorPos(motor) < (goalPos - RANGE))     // If current Pos is less than goal
      spd = spd;                                            //   go forwards
    else spd = 0;                                           // Inside +/- RANGE so sit still
  }
  setMtrSpeed(motor, spd);
}


void tensionMotors(float t, float tOut = 15.0){
  Serial.print("Tensioning Motors -- press button when complete...\n   will time out after ");
  Serial.print(tOut); Serial.print(" seconds\n");
  byte counts = 0;
  unsigned long started = micros();
  while(buttonStatus(4) == 0 && micros()-started < tOut * 1000000){
    getAngles();
    if(counts++ == 0) showAngleStatus(verboseMask + posMask + frcMask + actMask);
    for(int i = 0;i<NCH;i++){
      if(i < NCH/2) tension(i, t);      // lower ones at START_TENSION
      else tension(i, t + INC_TENSION); // upper ones at a little higher for gravity
    }
    delay(1);
  }
  setAllSpeeds(0);
}
//This tells an actuator to keep a specified constant tension on it's rope through velocity control----------------------------
// Needs to be called often to keep the actuators moving in the right directions.
void tension(int mtr, int desForce) { //Arguments (which motor, what force)

  //float tensionSpd[NCHIND] = {0, 0, 0, 0, 0, 0, 0, 0};
  int range = 1; //this number is the +/- for the desired force range
  float frc = smoothForce(mtr);
  float spd = 0;    // sit still by default
  if (frc > (desForce + range) && smoothMotorPos(mtr) < outPos[mtr]){ //outPos[] is max actuator piston position
    //increase or decrease the speed by a varying value dependent upon the difference in force
    //default setting below, ((MAX_SPEED-100)/4) is a chosen value to change how quickly the speed changes
    spd = max((frc - desForce) * ((MAX_SPEED - 100) / 4), MIN_SPEED);
    spd = min(spd, MAX_SPEED);
  }
  //slow down/retract----------------------------------------------------------
  else if (frc < (desForce - range) && smoothMotorPos(mtr) > inPos[mtr]) { //inPos[] is minimum actuator position
    spd = max(abs(frc - desForce) * ((MAX_SPEED - 100) / 4), MIN_SPEED);
    spd = - min(spd, MAX_SPEED);
  }
  //Serial.print(mtr); Serial.print(", "); Serial.print(spd); Serial.print("\n");
  setMtrSpeed(mtr, spd);
}


//tensionRange takes a force range from lowValue to highValue and controls the actuator
//(using the Tension function) to maintain the given range
void tensionRange(float lowValue, float highValue, int motor) {
  if (smoothForce(motor) < highValue && smoothForce(motor) > lowValue) {
    setMtrSpeed(motor, 0);
  }
  else if (smoothForce(motor) > highValue) {
    tension(motor, highValue);
  }
  else if (smoothForce(motor) < lowValue) {
    tension(motor, lowValue);
  }
}

/////Set speed for motor[]---------------------------------------------------------
/////Takes the motor number from the function input and adjusts the speed of that motor
/////Values to be used for speed are from -MAX_SPEED to MAX_SPEED which translate to -100% to 100% PWM
void setMtrSpeed(int motor , int speed) {
  unsigned char reverse = 0;
  // test for overload
  float f = smoothForce(motor);
  if(speed < 0 && f > MAX_TENSION) speed = 0;
  if(speed > 0 && f < -MAX_TENSION) speed = 0;
  
  if (speed < 0) {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  speed = min(speed,MAX_SPEED);
  // default to using analogWrite, mapping MAX_SPEED to 255
  analogWrite(_PWM[motor], speed * 255 / MAX_SPEED);
  if (reverse) {
    digitalWrite(_INA[motor], HIGH);
    digitalWrite(_INB[motor], LOW);
  }
  else {
    digitalWrite(_INA[motor], LOW);
    digitalWrite(_INB[motor], HIGH);
  }
}

/////Function to set speed for ALL MOTORS---------------------------------------------------------------------------------------
void setAllSpeeds(int mtrSpeed) {
  int i;
  for (i = 0; i < NCH; i++) setMtrSpeed(i, mtrSpeed);
}

////// Function to exercise motors, can be unidirectional
void exerciseMotors(int dir) {
  int del = 500;
  for (int i = 0; i < NCH; i++) {
    if (dir <= 0) {
      Serial.print("Reverse Motor (speed negative)"); Serial.print(i);
      setMtrSpeed(i , -MAX_SPEED);
      delay(del);
    }
    Serial.print(" OFF");
    setMtrSpeed(i, 0);
    delay(del);
    if (dir >= 0) {
      Serial.print(" Forward");
      setMtrSpeed(i, MAX_SPEED);
      delay(del);
    }
    Serial.print(" OFF\n");
    setMtrSpeed(i, 0);
    getAngles();
    delay(del);
    showAngleStatus(verboseMask + posMask + frcMask + actMask);
  }
}

void extendMotors(float tOut = 10.0) { // extend all motors to limit
  for (int i = 0; i < NCH; i++) {
    unsigned long started = micros();
    byte counts = 0;
    Serial.print("Extending "); Serial.println(i);
    updateMotorPos();
    while (latestPos[i] < outPos[i] && micros()-started < tOut * 1000000) {
      if(counts++ == 0) showAngleStatus(verboseMask + posMask + frcMask + actMask);
      setMtrSpeed(i , MAX_SPEED);
      updateMotorPos();
      //if(smoothForce(i) < -MAX_TENSION) break;
      delay(3);
    }
    setMtrSpeed(i, 0);
    getAngles();
    showAngleStatus(verboseMask + posMask + frcMask + actMask);
    Serial.print("Done Extending "); Serial.println(i);
  }
  setAllSpeeds(0);
}

void retractMotors(float tOut = 10.0) { // retract all motors to limit
  for (int i = 0; i < NCH; i++) {
    unsigned long started = micros();
    byte counts = 0;
    Serial.print("Retracting "); Serial.println(i);
    updateMotorPos();
    while (latestPos[i] > inPos[i] && micros()-started < tOut * 1000000) {
      if(counts++ == 0) showAngleStatus(verboseMask + posMask + frcMask + actMask);
      setMtrSpeed(i , -MAX_SPEED);
      updateMotorPos();
      //if(smoothForce(i) > MAX_TENSION) break;
      delay(3);
    }
    setMtrSpeed(i, 0);
    getAngles();
    showAngleStatus(verboseMask + posMask + frcMask + actMask);
    Serial.print("Done Retracting "); Serial.println(i);
  }
  setAllSpeeds(0);
}

void ditherCalForce(){      // Average analog force reading while motors in slow osc motion 
  double f[NCHIND] = {0};
  for(int i = 0; i < NCH; i++) f[i] = analogRead(_FRC[i]);
  Serial.print("\nCalibrating Motions to get force contants -- push button to quit.\n");
  while(!buttonStatus(4)){
    for(int i = 0; i < NCH; i++) goToPos(i, midPos[i] + 5 * sin(micros()/1000000.), MAX_SPEED);
    for(int i = 0; i < NCH; i++){ 
      f[i] = f[i] * 0.999 + analogRead(_FRC[i]) * 0.001;
      Serial.print(f[i],0); Serial.print(", ");
    }
    Serial.print("\n");
  }
  setAllSpeeds(0);
}
