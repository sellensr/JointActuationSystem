//This is the main function that actually controls the position of the Joint Platform

// Adapted from goToBothAngles() in V1.0
void goToAllAngles(float *angles) { //always start the loop with this first  
  getAngles(); //get the newest angle information from the AMP and store to actAng[]
  //caclulate the angular errors, positive if target angles exceed actual angles
  for(int i = 0;i<3;i++) deltaAng[i] =  angles[i] - actAng[i];
  //Calculate delta for each actuator  
  for (int i = 0; i < NCH; i++){
    deltaPos[i] = 0;  // positions are positive in y direction, reducing tension
    // so if the target exceeds the actual, the position should be decreased (more tension)
    // which happens because the respective gain is negative.
    // Note that the sense of deltaPos[] has changed from the Thesis code V1.0.
    for(int j = 0;j<3;j++) deltaPos[i] += gains[j][i] * deltaAng[j];
  }
  
  //chooseWhatToDo function that sends commands to the actuators depending on the Delta calculation
  chooseWhatToDo(deltaPos);

}

// This function sends the commands to each actuator in order to control the angular position of the Joint Platform
// The inputs are the Delta calculation for each actuator which is done in goToAllAngles
// Details of what each of these statements do is outlined in the Thesis.
// Note that the sense of deltaPos[] has changed from the Thesis code V1.0.
void chooseWhatToDo(float *inputs){
  for(int x = 0; x < NCH; x++){
    if (inputs[x] >= (0 + range)){    // if the sum of the gain*errors > 0, it is the off motor and hold low tension
      tension(x, 10);
    }
    else if (abs(inputs[x]) < range){ // if the position is in range, stand your ground with tension in a moderate range 
      tensionRange(15, 50, x);
    }
    else {                            // still need to keep moving at default speed
      motorGoalPos[x] = motorGoalPos[x] + inputs[x]; 
      goToPos(x, motorGoalPos[x], DEF_SPEED);
    }
  }
}

//Function that reads angular data from the serial1 port
//There is a specific way the information must be sent over the serial link in order for this function to read the data
//Fails when angle data comes in too fast, not sure why, could involve buffer overruns.
unsigned long timeLastAngles = 0;     // last time valid angles were collected
void getAngles(){  
  while(Serial1.available() > 17){  //if there are enough characters, the info is there, so read it as many times as necessary   
    char c = 0;
    while(c != '$') c = Serial1.read();  //throw away till '$'           
    // Using parseInt to save the chars serialRead to vars as floats floats with two decimals
    for(int i = 0;i<3;i++) actAng[i] = Serial1.parseInt() / 100.0;
    timeLastAngles = micros(); 
  }
}

// Spit out some characters from the angles input to verify they look OK.
// Just for debugging.
void sampleAnglesStream(int n){  
    for(int i = 0;i<n;i++) Serial.write(Serial1.read()); 
    Serial.print(": ");          
}
