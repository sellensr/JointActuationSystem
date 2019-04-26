#define NCH       4     // the number of actuator channels to use
#define CH4DUE          // the 4 actuator version based on the Arduino Due
// connect to DUE using programming port, USB closest to the power jack

//the following are declarations for each variable
//pins for each variable in array form, call motors 0 to 3 with [0],[1],[2],[3]
unsigned char _INA[] = {2, 7, 22, 25};                // motor direction control pins A and B
unsigned char _INB[] = {4, 8, 23, 3};                 //      used to set CW, CCW, or Brake
static const unsigned char _PWM[] = {9, 10, 5, 13};   // motor speed control pins
unsigned char _ENGDIAG[] = {6, 12, 24, 11};           // motor driver diagnostic pins
unsigned char _CS[] = {A0, A1, A2, A3};               // motor current sense analog inputs
unsigned char _POS[] = {A8, A10, A9, A11};            // actuator position signals from pots 
unsigned char _FRC[] = {A4, A5, A6, A7};              // actuator force signals from load cells and amps
const int buttonPin[] = {28, 29, 30, 31, 32};         // pins for the added push buttons
//end pin declaration--------------------------------------------------------------------------------------

//Calibration variables constants 
// Original design used INA125 in common ground mode for single sided output.
// Gain resistor was 100 ohm for gain approximately = 4 + 60000/Rg = 604
unsigned int _FRCOFFSET[] = {4607, 3345, 2665, 5805}; // in analogRead() units with zero load applied
double _FRCMULTI[] = {0.3323, 0.3308, 0.3318, 0.3198};// grams per analogRead() unit with load applied under 1 g field
