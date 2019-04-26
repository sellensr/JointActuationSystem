#define NCH       4       // the number of actuator channels to use
#define CH6TEENSY         // the 6 actuator version based on the Teensy 3.5

// include the SD library and set up to use:
#include <SPI.h>
#include <SD.h>
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = BUILTIN_SDCARD;  // for the Teensy 3.5

//the following are declarations for each variable
//pins for each variable in array form, call motors 0 to 3 etc. with [0],[1],[2],[3] etc.

// all down the left side leaving out pins 0,1 for Serial1 and 31,32 for Serial4
unsigned char               _INA[] = { 3,  6,  2, 12, 25, 27}; // motor direction control pins A and B
unsigned char               _INB[] = { 4,  7, 11, 24, 26, 28}; //      used to set CW, CCW, or Brake
static const unsigned char  _PWM[] = { 5,  8,  9, 10, 29, 30}; // motor speed control pins

// available on bottom. may not be needed. not used by Matt or up to 2019-01-18
unsigned char           _ENGDIAG[] = {40, 41, 42, 43, 44, 45}; // motor driver diagnostic pins

// near SD card end on right side
const int              buttonPin[] = {39, 39, 39, 39, 39};     // pins for the added push buttons

// all down the right side, leaving 39/A20 open for the big Red button
// _CS not used by Matt or up to 2019-01-18
unsigned char          _CS[] = { A4,  A5,  A6,  A7,  A8,  A9}; // motor current sense analog inputs
unsigned char         _POS[] = {A21, A22,  A0,  A1,  A2,  A3}; // actuator position signals from pots 
unsigned char         _FRC[] = {A14, A15, A16, A17, A18, A19}; // actuator force signals from load cells and amps
/*  Hardware    Motor      Proto #  T3.5 Pins from Above
 *   Other      Drivers                     USB Port
 *   Sources    GND              1       GND        Vin
 *     AID                       2 RX1   0          Analog GND
 *                               3 TX1   1          3V3
 *              M2INA            4       2~         A9   Current Sense M5
 *              M0INA            5       3~         A8                 M4
 *              M0INB            6       4~         A7                 M3
 *              M0PWM            7       5~         A6                 M2
 *              M1INA            8       6~         A5                 M1
 *              M1INB            9       7~         A4                 M0
 *              M1PWM           10       8~         A3        Position M5
 *              M2PWM           11       9~         A2                 M4 
 *              M3PWM           12      10~         A1                 M3
 *              M2INB           13      11          A0                 M2
 *              M3INA           14      12          13 LED
 *              VDD             15      V3V         GND
 *              M3INB           16      24          A22       Position M1
 *              M4INA           17      25          A21                M0
 *              M4INB           18      26       39/A20    Big Red Button
 *              M5INA           19      27          A19       Force    M5
 *              M5INB           20      28          A18                M4
 *              M4PWM           21      29~         A17                M3
 *              M5PWM           22      30~         A16                M2
 *  (External                      RX4 A12          A15                M1
 *   Control)                      TX4 A13          A14                M0
 *                                         SD Card
 *                                         
 *              M5EN           45
 *              M4EN           44
 *              M3EN           43
 *              M2EN           42
 *              M1EN           41
 *              M0EN           40
*                                         
 *                                         
 *                                         
 *                                         
 *                                         
 *                                         
 *                                         
 *                                         
 *                                         
 *                                         
 *                                         
 *                                         
 */
//end pin declaration--------------------------------------------------------------------------------------

//Calibration variables constants 
// V2 Design incorporates INA 126 Gain = 5 + 80000/Rg in pseudo ground mode 
// with 1.24V reference provided from INA125. 82 ohm resistors provide gain of 980.
// To calibrate measure average analogRead at horizontal for _FRCOFFSET. This is the no load case.
// Orient vertically down and measure average analogRead for _GRAVOFFSET. This allows for the weight 
//  of the actuator and string, etc. hanging from the load cell.
// Add a 5kg weight to the end of the string and measure average analogRead for _K5OFFSET. 
//  _FRCMULTI will then be 5000 / (_K5OFFSET - _GRAVOFFSET)

// These values need to be replaced with a current calibration! 2019-01-24
unsigned int _FRCOFFSET[NCHIND] = {31250,31620,26082,28880, 0, 0}; // in analogRead() units with zero load applied
//unsigned int _FRCOFFSET[NCHIND] = {0, 0, 0, 0, 0, 0, 0, 0}; // use to obtain calibration values
unsigned int _GRAVOFFSET[NCHIND] = {34195,35145,29255,32189, 0, 0}; // in analogRead() units with gravity load applied
unsigned int _LB10OFFSET[NCHIND] = {47580, 49617, 42465, 47430, 0, 0}; // in analogRead() units with gravity + 5 kg load applied
double _FRCMULTI[NCHIND] = {1,1,1,1,0,0};// grams per analogRead() unit with load applied under 1 g field
//double _FRCMULTI[NCHIND] = {101.97, 101.97, 101.97, 101.97, 101.97, 101.97};// use to obtain calibration values
