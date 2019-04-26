/* To Do:
 *  
 */



// Get tasks to fail after timing out -- motor tensioning for example
//    tensioning, extending, retracting done

// AP_ControlSystemCode V 2
#define VER            2.0
#define NCHIND         8      // make the arrays big enough we could go to 8 actuators
#define DEBUG          2      // determine how much monitoring goes to the console
#define START_TENSION  7      // starting tendon tension [N], Matt had 15
#define INC_TENSION    2      // small increase in tension [N], Matt had 5
#define MAX_TENSION   20      // maximum tension [N] to allow in any tendon
#define STOP_LOAD     30      // stop the motors pushing/pulling against this high a load [N]
#define MAX_SPEED    400      // maximum speed units with Pololu motor control library
#define DEF_SPEED     85      // go slower than maximum speed by default
#define MIN_SPEED     50      // minimum speed that is still actually reliably moving

#define tarMask       0b00000001
#define actMask       0b00000010
#define posMask       0b00000100
#define deltaMask     0b00001000
#define frcMask       0b00010000
#define verboseMask   0b10000000


// pick one and only one to establish AP hardware module
//#include "ch4due.h"   // obsolete now that DUE serial connector failed 2019-01-24
#include "ch6teensy.h"

/* Code for control of Actuator Platform (AP) of simulator system.
 * Receives serial input from the Angular Measurement Platform (AMP) via the Angle Interface Device (AID)
 * 
 * AP Coordinate Systems, etc. Viewed from behind the actuators
 * 
 * 
 *    z      4     5         NC    NC  
 *    ^      2     3          2     3
 *    |      0     1          0     1
 *    |   Actuator Motor    Actuator Motor 
 *    |    Numbers V2      Numbers Original
 *    |     
 *     ----------> x
 *     y is positive away from the viewer in normal RHS format
 *     Angles in degrees are arrays of 3 elements [a,b,c] corresponding to x,y,z axes.
 *      a is the x angle, 0 aligned with x-y plane, positive upwards
 *      b is the y angle, 0 aligned with x-y plane, positive clockwise
 *      c is the z angle, 0 aligned with y-z plane, positive leftwards
 *     
 *     For a right arm, palm down neutral:
 *      a is flexion-extension with extension positive
 *      b is pronation-suppination with zero pronated, increasing angle clockwise to suppination
 *        (ignored at present, but included in the angle definition for completeness)
 *      c is radial-ulnar deviation, radial deviation positive
 *      
 *      Motor speeds and positions are positive in the positive y direction, 
 *      away from the viewer, releasing tension on the tendons. 
 *      
 *      Forces should probably be negative in tension, but aren't yet
 *      
 *      Positions are in mm, positive in the positive y directionand always greater than zero.
 *      Position of 60 mm is almost fully retracted on original 4 motor rig.
 *     
 */
float posSmooth = 0.995; //closer to 0 for a fast response and closer to 1 for a smoother response
float forceSmooth = 0.99;
float posTau = 0.01;      // time constants in seconds 2019-02-22
float forceTau = 0.01;

//global variables for functions
int motorEnabled[NCHIND]   = {1, 1, 1, 1, 0, 0, 0, 0};   // set to zero for motors not to be used
float motorGoalPos[NCHIND] = {0, 0, 0, 0, 0, 0, 0, 0};   // mm
//variables for smoothing Pos and force
double oldPos[NCHIND] = {0, 0, 0, 0, 0, 0};              // most recent smoothed position data [mm]
double latestPos[NCHIND] = {0, 0, 0, 0, 0, 0};           // most recently read position data [mm]   
double inPos[NCHIND] = {70, 70, 70, 70, 70, 70};         // fully retracted motor positions [mm]
double midPos[NCHIND] = {100, 100, 100, 100, 100, 100};  // mid range motor positions [mm]
double outPos[NCHIND] = {140, 140, 140, 140, 140, 140};  // fully extended motor positions [mm]
float oldForce[NCHIND] = {0, 0, 0, 0, 0, 0};             // most recent smoothed force data [N]
float latestForce[NCHIND] = {0, 0, 0, 0, 0, 0};          // most recently read force data [N]
float deltaPos[NCHIND] = {0, 0, 0, 0, 0, 0};             // desired change in position [mm]

//Second Arduino Program Tab

float gBase = 0.04;         // scaled gain values all positive [mm / degree / iteration]
float range = 0.5 * gBase;  // choose what to do "range" is angle tolerance * gain

// Treating pulling on the tendons as a negative position actuation, gain is negative 
// if pulling on that tendon would increase the angle. The gain is in units of 
// position change desired to increase a given angle [mm / degree / iteration]. 
// angle[0] is increased by pulling on actuators 2 and 3 of the 4CH system
                             // and probably 4 5 as well in the 6CH system
// angle[0] is decreased by pulling on actuators 0 and 1 of the 4CH system
float gains[3][NCHIND] = {  { gBase,  gBase, -gBase, -gBase, -gBase, -gBase},      
                            // angle[1] is about the y axis and we don't care yet
                            {     0,      0,      0,      0,      0,      0},  
                            // angle[3] is increased by pulling on the left (0,2,4)    
                            {-gBase,  gBase, -gBase,  gBase, -gBase,  gBase} };    
  

// Note that v1.0 angles are different and wrist oriented, with pronation/supination ignored.
// Qualisys angle measurement version will deliver x angle, y angle, z angle in hundredths of a degree
// Floating point angles are in degrees.
//     float wristAng[3] = {0, 0, 0}; //latest position of the wrist {RUD, FE, PS}
float actAng[3] = {0, 0, 0};   //latest position in actuator coordinates {x axis angle,y axis angle,z axis angle}
float tarAng[3] = {0, 0, 0};   //current target angle in actuator coordinates {x axis angle,y axis angle,z axis angle}
float neutAng[3] = {0, 0, 0};   //neutral target angle in actuator coordinates {x axis angle,y axis angle,z axis angle}
float deltaAng[3] = {0, 0, 0}; //target angle - current angle
// conversion factors from analog read units to mm of position
float mmPerUnit[NCHIND] = {0.003009598, 0.003009598, 0.003009598, 0.003009598, 0.003009598, 0.003009598};


/////Initialize all pinModes and TIMER 1, set PWN to 20kHZ--------------------------------------------------------------
void initializePins(){
  for(int x = 0; x < NCH; x++){
    pinMode(_INA[x],OUTPUT);
    pinMode(_INB[x],OUTPUT);
    pinMode(_PWM[x],OUTPUT);
    pinMode(_ENGDIAG[x],INPUT);
    pinMode(_CS[x],INPUT);
  }  

  // 5 buttons on the 4CHDUE version and button #4 is the big red one
  for(int x = 0; x < 5; x++) pinMode(buttonPin[x], INPUT);  
}

//  Short function to check if a button is pressed or not for 0 to 4
//  Not used for anything in the original, except buttonStatus(4)
// Pulled low normally, connected to high when pushed
unsigned int buttonStatus(int button){
  return digitalRead(buttonPin[button]);
}


//Function to initialize the smoothed actuator position for all actuators---------------------------------------------------------
//To correct and for the delayed smoothed readings, position readings must be taken many times
//This function is meant for the setup{} of the main code and prints out each motors current position
void initializePosition(){
  Serial.print("Initial Motor Position Readings:");
  for(int j = 0;j<1000;j++) for(int i = 0; i < NCH; i++) smoothMotorPos(i);
  for(int i = 0; i < NCH; i++){
    Serial.print("  "); Serial.print(smoothMotorPos(i));
  }
  Serial.print("\n");
}

void updateMotorPos(){ for(int i = 0;i<NCH;i++) smoothMotorPos(i);}

///// Return Smoothed Analog Read Position of motor Calibrated into MILLIMETERS--------------------------------------------
double smoothMotorPos(int motor){
  static unsigned long timeLastP[NCHIND] = {0};
  unsigned long timeNowP = micros();
  float dt = (timeNowP-timeLastP[motor])/1000000.;
  if(dt > posTau) posSmooth = 0;
  else posSmooth = 1-dt/posTau;
  latestPos[motor] = analogRead(_POS[motor]) * mmPerUnit[motor];
  oldPos[motor] = posSmooth * oldPos[motor] + (1 - posSmooth) * latestPos[motor];
  return oldPos[motor];
}

//Initilizing the smoothed force readings, this is the same as for position measurements--------------------------------
void initializeForce(){
  // set grams / analogRead unit based on input values 1 lb = 454 g
  for(int i = 0;i < NCH;i++) _FRCMULTI[i] = 4540. / (_LB10OFFSET[i] - _GRAVOFFSET[i]);
  Serial.print("   Initial Motor Force Readings:");
  for(int j=0;j<1000;j++) for(int i = 0; i < NCH; i++) smoothForce(i);
  for(int i = 0; i < NCH; i++){
    Serial.print("  ");
    Serial.print(smoothForce(i));
  }
  Serial.print("\n");
}

void updateForce(){ for(int i = 0;i<NCH;i++) smoothForce(i);}

/////Return Smoothed Calibrated analog read of force of motor IN NEWTONS-------------------------------------------------------
float smoothForce(int motorF){
  static float NperGram =  .00980665002864; // N per g under 1g field = 9.8 N per kg
  static unsigned long timeLastF[NCHIND] = {0};
  unsigned long timeNowF = micros();
  float A = analogRead(_FRC[motorF]);
  float dt = (timeNowF-timeLastF[motorF])/1000000.;
  if(dt > forceTau) forceSmooth = 0;
  else forceSmooth = 1-dt/forceTau;
  latestForce[motorF] = NperGram * ((A - _FRCOFFSET[motorF] ) * _FRCMULTI[motorF]); // Newtons
  oldForce[motorF] = (forceSmooth * oldForce[motorF] + (1 - forceSmooth) * latestForce[motorF]);
  timeLastF[motorF] = timeNowF;
  return oldForce[motorF]; 
}
