//Third Arduino Program Tab
/////setup code here, to run once
float xAngle = 0;
float yAngle = 0;

float timeToTake = 25; //number of seconds per cycle
unsigned long lastIteration = 0;
float interval = timeToTake*1000/((sizeof(array)/sizeof(float))/2);

long lastPrint2 = 0;

//-------------------Flexion/Extension && Radial/Ulnar Deviation parameters--------------------
#define NUMCYCLES 25
int angleFlex = 15;
int angleExtend = 15;
int angleRadial = 15;
int angleUlnar = 15;
//------------------------------------------------------------------------------------------------


void setup() {
  Serial.begin(250000);
  Serial1.begin(57600);
  analogReadResolution(16);
  Serial.println("  ");
  Serial.println("Initializing pinModes...");
  // Initialization code here - Initialize all pinModes
  initialize();
  
  initializePosition();
  initializeForce();
  delay(100);
  Serial.println("Initialization Complete");  
  delay(200);  
  Serial.println("Set Up Complete");
  Serial.println("Program Ready");  
  delay(200);   
  Serial.println("Tensioning Motors");
  while(btnStatus(4) == 0){
    Tension(0, STARTTENSION);
    Tension(1, STARTTENSION);
    Tension(2, STARTTENSION+5);
    Tension(3, STARTTENSION+5);
  }
  
  delay(500);
  Serial.println("Motors Tensioned");
    getAngles();
  Serial.println("Got Angles");
  xAngle = wristAng[1];
  yAngle = wristAng[0];
  Serial.println("Program Begin"); 
}

void loop() {  
  
  Serial.println("Moving to Start Pos");
  iN = 0;
  while(btnStatus(4) == 0){
    goToBothAngles(15, 0);
  }
  delay(200);
  
  Serial.println("Begin Circumduction");

  while(btnStatus(4) == 0){
  goToBothAngles(xAngle, yAngle);
  timedCircArray();
  }
  delay(200);
  Serial.println("Restarting...");
}


//This is a function that changes the target anguluar position once the current angular position is within the set eRange
void positionCircArray(){ //this gets placed behind goToBothAngles() in the main loop to trace circumduction point by point once the error is small it moves on
  float eRange = .75; //Can be adjusted, will change/effect performance
  //forwards through the array
  xAngle = array[iN*2];
  yAngle = array[iN*2 + 1];
  if((micros()/1000 - lastPrint2) > 50 && deltaFe < eRange && deltaFe > -eRange && deltaRu < eRange && deltaRu > -eRange){
    iN = iN + 1;
    lastPrint2 = micros()/1000;
    if(iN >= ((sizeof(array)/sizeof(float))/2) ){
       iN=0;
    }
  }
}


//This changes the target angular position after a set amount of time based on the user set cycle time
void timedCircArray(){ //this gets placed behind goToBothAngles() in the main loop to trace the circumduction array
  if(iN < ((sizeof(array)/sizeof(float))/2) ){
    xAngle = array[iN*2];
    yAngle = array[iN*2 + 1];
    
    unsigned long currentMillis = micros()/1000;
    if((currentMillis - lastIteration) >= interval){ //(timeNow - lastIteration) < time between each point
      iN++;
      lastIteration = currentMillis;
      if(iN >= ((sizeof(array)/sizeof(float))/2) ){
        iN=0;
      }
    }
  }
}





