//Second Arduino Program Tab
float deltaPos[] = {0, 0, 0, 0}; //Delta variable declaration

float gFe = 0.04*1; //scaled gain value
float gRu = 0.04*1;

//radial ulnar deviation gain signs must be flipped in order to account for left hand
float gainFe[] = {-gFe, -gFe, gFe, gFe}; //extension is positive, motors 2/3 must move -ve to create this
float gainRu[] = {-gRu, gRu, -gRu, gRu}; //radial dev is positive, motors 0/2 must move -ve to create this

long lastPrint = 0;  
float range = 0.5*1*gFe; //choosewhat to do "range"

float deltaFe = 0;
float deltaRu = 0;

float wristAng[] = {0, 0}; //latest position of the wrist [RUD, FE]

//This is the main function that actually control the position of the Joint Platform

void goToBothAngles(float angleRu, float angleFe) { //always start the loop with this first
  //long checkPoint1 = micros();
  
  getAngles(); //get the newest angle information from the AMP
  //calulate the two angular errors
  deltaFe = angleFe - wristAng[1];
  deltaRu = angleRu - wristAng[0];
  
  //long checkPoint2 = micros();

  //Calculate delta for each actuator  
  for (int i = 0; i < 4; i++){
    deltaPos[i] = (gainFe[i] * deltaFe) + (gainRu[i] * deltaRu);
  }
  
  //long checkPoint3 = micros()

  //chooseWhatToDo function that sends commands to the actuators depending on the Delta calculation
  chooseWhatToDo(deltaPos[0], deltaPos[1], deltaPos[2], deltaPos[3]);

  //long checkPoint4 = micros();

//debugging timing printout
//  Serial.print((checkPoint2 - checkPoint1)); //for seeing the timing of different parts of the function
//  Serial.print(", ");
//  Serial.print((checkPoint3-checkPoint2));
//  Serial.print(", ");
//  Serial.print((checkPoint4-checkPoint3));
//  Serial.print(", ");
//  Serial.println((checkPoint5-checkPoint4));
  
  //print around ever 100ms
  if( (micros()/1000 - lastPrint) > 100){ //for full print out of all info
    Serial.print(micros()/1000);
    Serial.print(", ");
    Serial.print(wristAng[0]);
    Serial.print(", ");
    Serial.print(wristAng[1]);
    Serial.print(" : ");
    Serial.print(deltaPos[0]);
    Serial.print(", ");
    Serial.print(deltaPos[1]);
    Serial.print(", ");
    Serial.print(deltaPos[2]);
    Serial.print(", ");
    Serial.print(deltaPos[3]);
    Serial.print(" : ");
    Serial.print(smoothForce(0));
    Serial.print(", ");
    Serial.print(smoothForce(1));
    Serial.print(", ");
    Serial.print(smoothForce(2));
    Serial.print(", ");
    Serial.print(smoothForce(3));
    Serial.print(", ");
    Serial.print(iN);
    Serial.println(" ");
    lastPrint = micros()/1000;
  }
}

//Function that reads angular data from the serial1 port
//There is a specific way the information must be sent over the serial link in order for this function to read the data
void getAngles(){  
  if(Serial1.available() > 9){  //if there are more than 10 characters, the info is there, so read it   
    char c = 0;
    while(c != '$') c = Serial1.read();  //throw away till '$'           
    
    ruda = Serial1.parseInt(); //Using parseInt to save the chars serialRead to vars
    fea = Serial1.parseInt();
          
    wristAng[0] = rudaF = ruda/100.00; //Change Ints into floats and move decimal twice
    wristAng[1] = feaF = -fea/100.00;
  }
}

//This function sends the commands to each actuator in order to control the angular position of the Joint Platform
//The inputs are the Delta calculation for each actuator which is done in goToBothAngles
//Details of what each of these statements do is outlined in the Thesis
void chooseWhatToDo(float thing0, float thing1, float thing2, float thing3){
  float inputs[] = {thing0, thing1, thing2, thing3};
  for(int x = 0; x < 4; x++){
    if (inputs[x] <= (0 - range)){ //if the sum of the gain*errors < 0, it is the off motor and hold tension
      Tension(x, 10);
    }
    else if (inputs[x] > (0 - range) && inputs[x] < (0 + range)){ 
      tensionRange(15, 50, x);
    }
    else {
      motorGoalPos[x] = motorGoalPos[x] - inputs[x]; 
      goToPos2(x, motorGoalPos[x], 85); //default 85, can change depending on performance
    }
  }
  
}
