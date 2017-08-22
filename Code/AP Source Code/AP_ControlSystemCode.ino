//First Arduino Program Tab
//Array created using Excel and copied into the code
//5rad 10flex shifted left 5 this is fully ulnar clockwise circumduction
float array[] = {-5,10,-4.686047402,9.980267284,-4.373333832,9.921147013,-4.063093427,9.822872507,-3.756550564,9.685831611,
-3.454915028,9.510565163,-3.159377237,9.297764859,-2.871103542,9.048270525,-2.591231629,8.7630668,-2.320866025,8.443279255,
-2.061073739,8.090169944,-1.812880051,7.705132428,-1.57726447,7.289686274,-1.355156863,6.845471059,-1.147433786,6.374239897,
-0.954915028,5.877852523,-0.778360372,5.35826795,-0.6184666,4.817536741,-0.475864738,4.257792916,-0.351117571,3.681245527,
-0.244717419,3.090169944,-0.157084194,2.486898872,-0.088563746,1.873813146,-0.039426493,1.253332336,-0.009866358,0.627905195,
0,-6.04876E-15,-0.009866358,-0.627905195,-0.039426493,-1.253332336,-0.088563746,-1.873813146,-0.157084194,-2.486898872,
-0.244717419,-3.090169944,-0.351117571,-3.681245527,-0.475864738,-4.257792916,-0.6184666,-4.817536741,-0.778360372,-5.35826795,
-0.954915028,-5.877852523,-1.147433786,-6.374239897,-1.355156863,-6.845471059,-1.57726447,-7.289686274,-1.812880051,-7.705132428,
-2.061073739,-8.090169944,-2.320866025,-8.443279255,-2.591231629,-8.7630668,-2.871103542,-9.048270525,-3.159377237,-9.297764859,
-3.454915028,-9.510565163,-3.756550564,-9.685831611,-4.063093427,-9.822872507,-4.373333832,-9.921147013,-4.686047402,-9.980267284,
-5,-10,-5.313952598,-9.980267284,-5.626666168,-9.921147013,-5.936906573,-9.822872507,-6.243449436,-9.685831611,-6.545084972,
-9.510565163,-6.840622763,-9.297764859,-7.128896458,-9.048270525,-7.408768371,-8.7630668,-7.679133975,-8.443279255,-7.938926261,
-8.090169944,-8.187119949,-7.705132428,-8.42273553,-7.289686274,-8.644843137,-6.845471059,-8.852566214,-6.374239897,-9.045084972,
-5.877852523,-9.221639628,-5.35826795,-9.3815334,-4.817536741,-9.524135262,-4.257792916,-9.648882429,-3.681245527,-9.755282581,
-3.090169944,-9.842915806,-2.486898872,-9.911436254,-1.873813146,-9.960573507,-1.253332336,-9.990133642,-0.627905195,-10,
-1.83772E-15,-9.990133642,0.627905195,-9.960573507,1.253332336,-9.911436254,1.873813146,-9.842915806,2.486898872,-9.755282581,
3.090169944,-9.648882429,3.681245527,-9.524135262,4.257792916,-9.3815334,4.817536741,-9.221639628,5.35826795,-9.045084972,
5.877852523,-8.852566214,6.374239897,-8.644843137,6.845471059,-8.42273553,7.289686274,-8.187119949,7.705132428,-7.938926261,
8.090169944,-7.679133975,8.443279255,-7.408768371,8.7630668,-7.128896458,9.048270525,-6.840622763,9.297764859,-6.545084972,
9.510565163,-6.243449436,9.685831611,-5.936906573,9.822872507,-5.626666168,9.921147013,-5.313952598,9.980267284,-5,10};
int iN = 0; //counter for arrays

//the following are declarations for each variable
//pins for each variable in array form, call motors 0 to 3 with [0],[1],[2],[3]
unsigned char _INA[] = {2, 7, 22, 25};
unsigned char _INB[] = {4, 8, 23, 3};
static const unsigned char _PWM[] = {9, 10, 5, 13};
unsigned char _ENGDIAG[] = {6, 12, 24, 11};
unsigned char _CS[] = {A0, A1, A2, A3};
unsigned char _POS[] = {A8, A10, A9, A11};
unsigned char _FRC[] = {A4, A5, A6, A7};
const int buttonPin[] = {28, 29, 30, 31, 32}; //pins for the added push buttons
//end pin declaration--------------------------------------------------------------------------------------

//Calibration variables constants 
unsigned int _FRCOFFSET[] = {4607, 3345, 2665, 5805};
double _FRCMULTI[] = {0.3323, 0.3308, 0.3318, 0.3198};
double _POSOFFSET[] = {3.00959, 0, 0, 0};
float posSmooth = 0.995; ////closer to 0 for a fast response and closer to 1 for a smoother response
float forceSmooth = 0.99;

//global variables for functions
float motorGoalPos[] = {0, 0, 0, 0};
//variables for smoothing Pos and force
double oldPos[] = {0, 0, 0, 0};
float oldForce[] = {0, 0, 0, 0};
float oldAnalogForce[] = {0, 0, 0, 0};
float mtrForce = 0;
float mtrAnalogForce = 0;
double mtrPos = 0;


#define STARTTENSION 15

long ruda, fea;      //vars for anglular positions for getAngles
float rudaF, feaF;

/////Initialize all pinModes and TIMER 1, set PWN to 20kHZ--------------------------------------------------------------
void initialize(){
  for(int x = 0; x < 4; x++){
    pinMode(_INA[x],OUTPUT);
    pinMode(_INB[x],OUTPUT);
    pinMode(_PWM[x],OUTPUT);
    pinMode(_ENGDIAG[x],INPUT);
    pinMode(_CS[x],INPUT);
  }  
  
  for(int x = 0; x < 5; x++){
    pinMode(buttonPin[x], INPUT);
  }
  
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)//this is not for the DUE
  // Timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;  //for 9/10
  TCCR1B = 0b00010001;  //for 9/10
  TCCR4A = 0b10100000;  //for 6/13
  TCCR4B = 0b00010001;  //for 6/13
  TCCR3A = 0b10100000;  //for 5
  TCCR3B = 0b00010001;  //for 5
  ICR1 = 400;
  #endif
}

//Short function to check if a button is pressed or not for 0 to 4
unsigned int btnStatus(int button){
  return digitalRead(buttonPin[button]);
}

/////Set speed for motor[]---------------------------------------------------------
/////Takes the motor number from the function input and adjusts the speed of that motor
/////Values to be used for speed are from -400 to 400 which translate to -100% to 100% PWM
void setMtrSpeed(int motor ,int speed){
  unsigned char reverse = 0;
  
  if (speed < 0){
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400){  // Max PWM dutycycle
    speed = 400;
  }
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  OCR1A = speed;
  #else
  analogWrite(_PWM[motor],speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif
  if (reverse){
    digitalWrite(_INA[motor],HIGH);
    digitalWrite(_INB[motor],LOW);
  }
  else{
    digitalWrite(_INA[motor],LOW);
    digitalWrite(_INB[motor],HIGH);
  }
}

/////Function to set speed for ALL MOTORS---------------------------------------------------------------------------------------
void setAllSpeeds(int mtrSpeed){
  setMtrSpeed(0, mtrSpeed);
  setMtrSpeed(1, mtrSpeed);
  setMtrSpeed(2, mtrSpeed);
  setMtrSpeed(3, mtrSpeed);
}

//Function to initialize the smoothed actuator position for 4 actuators---------------------------------------------------------
//To correct and for the delayed smoothed readings, position readings must be taken many times
//This function is meant for the setup{} of the main code and prints out each motors current position
void initializePosition(){
  Serial.println("Initializing Motor Position Readings");
  for(int i = 0; i < 4; i++){
    Serial.print("Reading Position of Motor ");
    Serial.println(i); 
    int timeNow = millis();   
    while( (millis() - timeNow) < 2000 ){
      smoothMotorPos(i);
    }
    Serial.print("Position of Motor ");
    Serial.print(i);
    Serial.print(" - ");
    Serial.println(smoothMotorPos(i));
  }
}

///// Return Smoothed Analog Read Position of motor Calibrated into MILLIMETERS--------------------------------------------
double smoothMotorPos(int motor){
  mtrPos = (posSmooth * oldPos[motor] + (1 - posSmooth) * analogRead(_POS[motor]));
  oldPos[motor] = mtrPos;
  return (mtrPos* 3.009598 / 1000 );
}

/////Function to Read all four smoothed motor positions using one line of code in order to keep track-------------------------
/////of where the motors are at all times without returning any values
//If the positions of each actuator are not updated everytime around a main loop, this can be used
//Positions must be tracked regularly in order to produce accurate smoothed position measurements
void smoothPosAllRead(){
 for(int i=0; i<4; i++){
  smoothMotorPos(i);
 }
}

///// Return Analog Read Position of Motor-------------------------------------------------------------------------------------
unsigned int motorPos(int mtr){
  return analogRead(_POS[mtr]);
}

/////Opperation to read a throw away analogRead on position pin to decharge it--------------------------------------------------
unsigned int throwAwayPos(int mtr){
  analogRead(_POS[mtr]);
}

//Initilizing the smoothed force readings, this is the same as for position measurements--------------------------------
void initializeForce(){
  Serial.println("Initializing Motor Force Readings");
  int i;
  for(i = 0; i < 4; i++){
    Serial.print("Reading Force of Motor ");
    Serial.println(i);
    long timeNow = millis();
    while( (millis() - timeNow) < 2000){
      smoothForce(i);
      smoothAnalogForce(i);
    }
    Serial.print("Force of Motor ");
    Serial.print(i);
    Serial.print(" - ");
    Serial.println(smoothForce(i));
  }
}

/////Return Smoothed Calibrated analog read of force of motor IN NEWTONS-------------------------------------------------------
float smoothForce(int motorF){
  mtrForce = ((smoothAnalogForce(motorF) - _FRCOFFSET[motorF] ) * _FRCMULTI[motorF]) * 0.00980665002864;
  return mtrForce; 
}
////Return smoothed analog read of force, not calibrated------------------------------------------------------------------------
float smoothAnalogForce(int motorF){
  mtrForce = (forceSmooth * oldAnalogForce[motorF] + (1 - forceSmooth) * analogRead(_FRC[motorF]));
  oldAnalogForce[motorF] = mtrForce;
  return mtrForce; 
}

 //This tells an actuator to keep a specified constant tension on it's rope through velocity control----------------------------
#define MAXSPEED 400
void Tension(int mtr, int desForce){ //Arguments (which motor, what force)
  
  float tensionSpd[] = {0, 0, 0, 0};
  int range = 1; //this number is the +/- for the desired force range
  float frc = smoothForce(mtr);
  
    if(frc > (desForce + range) && smoothMotorPos(mtr) < 145){ //145 position is max actuator piston position
      //increase or decrease the speed by a varying value dependent upon the difference in force

      //default setting blow, ((MAXSPEED-100)/4) is a chosen value to change how quickly the speed changes
      tensionSpd[mtr] = abs(frc - desForce) * ((MAXSPEED-100)/4); 
      if(tensionSpd[mtr] > MAXSPEED){tensionSpd[mtr] = MAXSPEED;}
      if(tensionSpd[mtr] < 50){tensionSpd[mtr] = 50;}
      setMtrSpeed(mtr, tensionSpd[mtr]);
    }
    
    //slow down/retract----------------------------------------------------------
    else if(frc < (desForce - range) && smoothMotorPos(mtr) > 20){ //20 is minimum actuator position
      tensionSpd[mtr] = abs(frc - desForce) * ((MAXSPEED-100)/4); //default setting
      if(tensionSpd[mtr] < 50){tensionSpd[mtr] = 50;}
      if(tensionSpd[mtr] > MAXSPEED){tensionSpd[mtr] = MAXSPEED;}
      setMtrSpeed(mtr, -tensionSpd[mtr]);
    }
    //If force is within range, set motor speed 0------------------------------
    else if(frc <= (desForce + range) && frc >= (desForce - range) || smoothMotorPos(mtr) >= 145 || smoothMotorPos(mtr) <= 22){
      setMtrSpeed(mtr, 0);
    }
}

//Short function to tension all actuators to the same force
void tensionAll(int desForce){
  for(int i = 0; i < 4; i++){
    Tension(i, desForce);
  }
}

/////goToPos2 moves a specific motor to a set goal position at set speed within a user set range
void goToPos2(int motor, int goalPos, int spd){
  int RANGE = 1; 
  long delta; //the relative position of the motor relative to goal
  
  ///if current Pos is more than goal
  if(smoothMotorPos(motor) > (goalPos + RANGE)){

   if(spd > 400){ ///failsafe to make sure 400 is max speed
    spd = 400;
   }     
   if(spd < 50){
    spd = 50;
   }
    setMtrSpeed(motor, -spd);
  }
    
  ///If current Pos is less than Goal
  else if(smoothMotorPos(motor) < (goalPos - RANGE)){

   if(spd > 400){ ///failsafe to make sure 400 is max speed
    spd = 400;
   }     
   if(spd < 50){
    spd = 50;
   }
    setMtrSpeed(motor, spd);
  }
  else if((goalPos + RANGE) > smoothMotorPos(motor) && smoothMotorPos(motor) > (goalPos - RANGE)){ 
    setMtrSpeed(motor, 0);
   }
}

//tensionRage takes a force range from lowValue to highValue and controls the actuator 
//(using the Tension function) to maintain the given range
void tensionRange(float lowValue, float highValue, int motor){
  if(smoothForce(motor) < highValue && smoothForce(motor) > lowValue){
    setMtrSpeed(motor, 0);
    }
  else if(smoothForce(motor) > highValue){
    Tension(motor, highValue);
    }
  else if(smoothForce(motor) < lowValue){
    Tension(motor, lowValue);
    }
}

