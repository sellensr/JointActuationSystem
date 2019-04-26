//-------------------Flexion/Extension && Radial/Ulnar Deviation parameters--------------------
#define NUMCYCLES 25
int angleFlex = 15;
int angleExtend = 15;
int angleRadial = 15;
int angleUlnar = 15;

//-------------------General Gesture parameters ----------------------------------------------
#define GPARMS           10
#define GSAVED           10
#define GNONE             0
#define CIRCUMDUCTION     1
#define LINE              2
#define POINT             3
#define GTYPES            4
const char *gestureTypeNames[] = {"Null","Circumduction","Line","Point"};

// Gestures are generally a closed path, there and back again. 
// gesture {type,duration,reps,dir,start/centrex, y, z,finish/ampx, y,  z}
//            0      1      2   3       4         5  6     7        8   9
// a null gesture that does nothing for 5 seconds
float gNone[] = {GNONE, 5, 1};
// a circle of 30 seconds per rotation, repeated 10 times, CW (+y rot), centre 3,0,5, radius 15 degrees
float circ15[] = {CIRCUMDUCTION, 30, 10, 1, 3, 0, 5, 15};
// a line traverse of 30 seconds each way, repeated 10 times, no dir, from 15 up and left to 15 down and right
float dart15[] = {LINE,          60, 10, 1, 15, 0, 15, -15, 0, -15};
// a direction to go to a point, hold for 60 seconds, repeat 10 times, no dir, to 5 degrees up and left 
float point5[] = {POINT,         60, 10,  1, 5, 0,  5};

float *savedGestures[GSAVED] = {0};
String savedGestureNames[GSAVED];
float thisGesture[GPARMS] = {GNONE, 5, 1};  // current, will change
unsigned long timeGestureStart = 0;         // the time the current gesture started [us]
float printTime = 0.5;                      // seconds between console outputs
unsigned printVerbose = verboseMask;        // make the console display human friendly
//----------------------------------------------------------------------------------------------



void setup() {
  // Serial communication ports are hard coded throughout 2019-01-19
  Serial.begin(115200);           // the console for reporting status hard coded throughout
  Serial1.begin(57600);           // the angle inputs from the Angular Measurement Platform (AMP)
                                  // via the Angular Interface Device (AID)
  analogReadResolution(16);       // always 16 bit, even if actual resolution is lower
  waitForSerial();
  Serial.print("\n\nMP-Sim v "); Serial.print(VER); Serial.print(" for tendon actuation of joints\n"); 
  Serial.print("   - derived from https://github.com/MattCPearson/WristActuationSystem\n");
  Serial.print("   - major rewrite and expansion starting with v 2.0\n");
  Serial.print("   - modifications by Rick Sellens licensed under Creative Commons CC-BY-SA\n");
  waitForSerial();
  Serial.print("\n\nInitializing...\n");
  // Initialization code here
  initializePins();               // Initialize all pinModes
  initializePosition();           // Take enough Pos data for smoothing
  initializeForce();              // Take enough Frc data for smoothing
  for(int i = 0;i < GSAVED;i++){  // Initialize the table to be empty
    savedGestureNames[i] = "";
    savedGestures[i] = (float *) malloc(GPARMS * sizeof(float));
    for(int j = 0;j < GPARMS;j++) savedGestures[i][j] = 0;
  }
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) Serial.println("Card failed, or not present");
  else Serial.println("Card successfully initialized.");
  waitForSerial();
  File gestureFile;
  if(DEBUG > 2){      // display the contents of the gesture file
    gestureFile = SD.open("gestures.txt", FILE_READ);
    if (gestureFile) {
      while(gestureFile.available()){
        char c = gestureFile.read();
        if(c == '\n') Serial.print("[newline]");
        if(c == '\r') Serial.print("[CR]");
        Serial.write(c);
        waitForSerial();
      }
      gestureFile.close();
    }
  }
  gestureFile = SD.open("gestures.txt", FILE_READ);
  if (gestureFile) {
    gestureFile.readStringUntil('\n');    // skip the first line
    for(int i = 0;i<GSAVED;i++){
      if(gestureFile.available()){
        savedGestureNames[i] = gestureFile.readStringUntil(',');
        savedGestureNames[i] = savedGestureNames[i].trim();
        for(int j = 0;j < GPARMS;j++) savedGestures[i][j] = gestureFile.parseFloat();
        gestureFile.readStringUntil('\n'); // get rid of the end of line
      }
    }
    gestureFile.close();
  }

  Serial.print("Initialization Complete...\n\n");
  waitForSerial();
  delay(1200);
  timeGestureStart = micros();
}

void loop() {
  static unsigned long timeLast = 0;  
  static unsigned long timeLastPrint = 0;  
  unsigned long timeNow = micros();
  unsigned long dt = timeNow-timeLast;
  static int ret = 0;
  static bool prompted = false;
  
  static String consoleIn = "";
  if(Serial.available()){       // accumulate command string and act on it when completed.
    char c = Serial.read();
    if(c == '\n' || c == '\r'){
      //Serial.print("Console Input:["); Serial.print(consoleIn); Serial.println("]");
      if(consoleIn.length() != 0){ 
        if(doConsoleCommand(consoleIn)){ 
          ret = 0;      // there may be a new valid gesture
          timeGestureStart = micros();
          delay(200);
          timeNow = micros();
        }
      }
      consoleIn = "";
    } else consoleIn += c;
  }
  
  if(buttonStatus(4) == 0 && ret == 0){     // do the main control loop actions
    prompted = false;
    ret = updateTargetAngles((timeNow-timeGestureStart)/1000000.,thisGesture,tarAng);
    if(ret == 0) goToAllAngles(tarAng);     // act only on valid angles
    if( (timeNow - timeLastPrint)/1000000. > printTime){
      Serial.print(dt); Serial.print(", "); if(DEBUG > 2) Serial.print("us delta, ");
      showAngleStatus(printVerbose + tarMask + actMask + posMask + deltaMask + frcMask);
      timeLastPrint = timeNow;
    } else  waitForSerial();                // let serial output happen
  } else {                                  // its all over!
    setAllSpeeds(0);                        // stop everything
    waitForSerial();
    if(prompted == false){                  // prompt only once while idling through the loop
      Serial.print("\n********************\nGesture Completed or Killed... Motors Stopped... \n");
      Serial.print("Waiting for a new Command from Serial Monitor\n********************\n");
      delay(2000);
      doConsoleCommand("h");
      prompted = true;
    }
  }
  timeLast = timeNow;
  waitForSerial();               // let serial output happen
}

int doConsoleCommand(String cmd){
  static float pos[NCHIND] = {0};
  static float pendingGesture[GPARMS] = {0};
  static String pendingGestureName = "";

  // Parse the command into an array of floats that follow the one letter code
  int comma[GPARMS] = {0};        // find the commas, and there better not be more than 7!
  float val[GPARMS] = {0};
  comma[0] = cmd.indexOf(',');
  //Serial.print(comma[0]); Serial.print(", ");
  for(int i = 1; i < GPARMS; i++){ 
    comma[i] = 1 + comma[i-1] + cmd.substring(comma[i-1]+1).indexOf(',');
    //Serial.print(comma[i]); Serial.print(", ");
  }
  val[0] = (cmd.substring(1,comma[0])).toFloat();   // extract the values between the commas
  for(int i = 1;i < GPARMS;i++){
    if(comma[i] == comma[i-1]){                     // this is the last one so break out
      val[i] = (cmd.substring(comma[i-1]+1)).toFloat();
      break;
    }
    else val[i] = (cmd.substring(comma[i-1]+1,comma[i])).toFloat();
  }

  // Declarations are not allowed inside the switch statement
  //Serial.print("\nDoing the Command: ["); Serial.print(cmd); Serial.print("]\n");
  char c = cmd.charAt(0);
  String s = cmd.substring(1);  // the whole string after the command letter
  int ival = val[0];            // an integer version of the first float arg
  int inList = -1;              // index of the current name in the list
  unsigned long start = micros();
  File gestureFile;
   switch(c){
    case 'c':
      for(int i = 0;i < 3;i++) pendingGesture[i+4] = val[i];
      //Serial.print("Setting start/centre to: "); 
      //showTriplet(pendingGesture+4);
      //Serial.print("\n");
      break;
    case 'd':
      if(pendingGesture[3] < 0) pendingGesture[3] = 1;
      else pendingGesture[3] = -1;
      //Serial.print("Changing direction\n");
      break;
    case 'D':
      printTime = abs(val[0]);
      if(printTime < 0.025) printTime = 0.025;
      if(printTime > 10.0) printTime = 10.0;
      Serial.print(printTime,3); Serial.print(" s between console reports.\n");
      break;
    case 'E': 
      doConsoleCommand("K"); // Kill the current gesture!!!
      Serial.print("Extending Motors...  "); 
      extendMotors();
      Serial.println("Motor Extension Complete");  
      break;
    case 'f':
      for(int i = 0;i < 3;i++) pendingGesture[i+7] = val[i];
      //Serial.print("Setting finish/amplitudes to: "); 
      //showTriplet(pendingGesture+7);
      //Serial.print("\n");
      break;
    case 'g':
      if(ival <= GTYPES && ival > 0){
        pendingGesture[0] = ival;
        //Serial.print("Setting next gesture type to: "); Serial.println(pendingGesture[0]); 
      } else{
        Serial.print("Invalid gesture type: "); Serial.println(ival);
      }
      break;
    case 'G':
      if(ival > 0 && ival <= GSAVED){
        for(int j = 0;j < GPARMS;j++) pendingGesture[j] = savedGestures[ival-1][j];
        pendingGestureName = savedGestureNames[ival-1];
        //Serial.print("Setting next gesture to:\n");
        //showGestureItem(ival,savedGestureNames[ival-1],pendingGesture);
      } 
      break;
    case 'K':           // kill the current gesture by making it null
      setAllSpeeds(0);  // first stop everything immediately
      for(int i = 0;i<GPARMS;i++) thisGesture[i] = 0;
      break;
    case 'l':
    case 'L':
      Serial.print("\nSaved Gestures:\n");
      for(int i = 0;i < GSAVED;i++){
        if(savedGestures[i][0] != 0 && savedGestures[i][0] <= GTYPES)
          showGestureItem(i+1,savedGestureNames[i],savedGestures[i]);
      }
      break;
    case 'n':
      pendingGesture[2] = max(1,abs(ival));
      //Serial.print("Setting repetitions to: "); Serial.println(pendingGesture[2]);
      break;
    case 'N':
      pendingGestureName = s.trim();
      //Serial.print("Setting gesture name to: "); Serial.println(pendingGestureName);
      //Serial.print("New gestures with non-blank names will be saved when executed.\n");
      break;
    case 'P':
      doConsoleCommand("K"); // Kill the current gesture!!!
      for(int i = 0;i < NCH;i++) pos[i] = val[i];
      Serial.print("Setting actuator positions to: "); 
      for(int i = 0;i<NCH-1;i++){ Serial.print(pos[i]); Serial.print(", ");} Serial.println(pos[NCH-1]); 
      start = micros();
      while(micros()-start < 10.0 * 1000000) for(int i = 0;i<NCH;i++) goToPos(i, pos[i]);
      setAllSpeeds(0);
      // needs action
      
      break;
    case 'R':
      doConsoleCommand("K"); // Kill the current gesture!!!
      Serial.print("Retracting Motors...  "); 
      retractMotors();
      Serial.print("Motor Retraction Complete\n");  
      break;
    case 't':
      pendingGesture[1] = max(1,abs(val[0]));
      //Serial.print("Setting time per repetition to: "); Serial.println(pendingGesture[1]);
      break;
    case 'T': 
      doConsoleCommand("K"); // Kill the current gesture!!!
      //Serial.print("Tensioning Motors...  "); 
      tensionMotors(START_TENSION);
      //Serial.println("Motor Tensioning Complete");  
      break;
    case 'V': // toggle verbose mode
      if(printVerbose == verboseMask){ printVerbose = 0; Serial.print("CSV output set\n");} 
      else{ printVerbose = verboseMask; Serial.print("Verbose output set\n");}
      break;
    case 'W': // calibrate with weights
      doConsoleCommand("K"); // Kill the current gesture!!!
      showAnalogForce();
      break;
    case 'X': // execute the next gesture
      doConsoleCommand("K"); // Kill the current gesture!!!
      // validate pending gesture
      if(pendingGesture[0] == 0 || pendingGesture[0] > GTYPES){
        Serial.print("Not a valid gesture type\n");
        break;
      }
      if(pendingGestureName != ""){     // only save if named
        // is it a new name?
        for(int i = GSAVED-1;i >= 0;i--) if(pendingGestureName == savedGestureNames[i]) inList = i;
        if(inList >= 0){          // existing name, so replace settings in list and move to top
          for(int i = inList;i > 0;i--){
            savedGestureNames[i] = savedGestureNames[i-1];
            for(int j = 0;j<GPARMS;j++) savedGestures[i][j] = savedGestures[i-1][j];
          }
          savedGestureNames[0] = pendingGestureName;
          for(int j = 0;j<GPARMS;j++) savedGestures[0][j] = pendingGesture[j];
        } else {                 // new name so add to the top of the list and drop oldest
          for(int i = GSAVED-1;i > 0;i--){
            savedGestureNames[i] = savedGestureNames[i-1];
            for(int j = 0;j < GPARMS;j++) savedGestures[i][j] = savedGestures[i-1][j]; 
          }
          savedGestureNames[0] = pendingGestureName;
          for(int j = 0;j < GPARMS;j++) savedGestures[0][j] = pendingGesture[j]; 
        }
      }
      // Delete the old version and save the gesture list to the SD Card
      SD.remove("gestures.txt");
      gestureFile = SD.open("gestures.txt", FILE_WRITE);
      if (gestureFile) {
        gestureFile.print("MP-Sim Gesture List\n");
        for(int i = 0;i < GSAVED;i++){
          gestureFile.print(savedGestureNames[i]);
          for(int j = 0;j < GPARMS;j++){
             gestureFile.print(", "); gestureFile.print(savedGestures[i][j]);
          }
          gestureFile.print("\n");
        }
        gestureFile.close();
      }
      
      // copy the pending gesture to the active gesture -- needs some safeguards!
      for(int i = 0;i<GPARMS;i++) thisGesture[i] = pendingGesture[i];
      return -1;     // there is a new gesture live, so wake up and run it
      break;
    default:
      Serial.print("\n*****************************\n");
      Serial.print(c); 
      Serial.print(" is not the start of a valid command.\n*****************************\n");
    case 'h':
      waitForSerial();
      Serial.print("\nCommands (in typical usage order for Gesture Definition) include:\n");
      Serial.print("  g - set the type for the next (g)esture, e.g. g2 for a line\n");
      Serial.print("      1 for circumduction, 2 for line, 3 for point, ...\n");
      Serial.print("  t - set the (t)ime/rep for the next gesture, e.g. t53.7\n");
      waitForSerial();
      Serial.print("  n - set the (n)umber of repetitions for the next gesture, e.g. n2 for twice\n");
      Serial.print("  c - set the (x,y,z) angular (c)entre/start for the next gesture, e.g. c5,0,9\n");
      Serial.print("  f - set the (x,y,z) angular (f)inish/amplitudes for the next gesture, e.g. f5,0,9\n");
      Serial.print("  d - toggle the (d)irection between clockwise and counter-clockwise, e.g. d\nGesture List Commands\n");
      waitForSerial();
      
      Serial.print("  N - name the next gesture, just a mnemonic tag, e.g.N RWS12\n");
      Serial.print("  X - execute the next gesture! (will be saved at top of list if named)\n"); 
      Serial.print("  L - print the (L)ist of saved gestures\n");
      Serial.print("  G - set the next (G)esture from the saved list, e.g. G3 for list item 3\nOther Commands\n");
      waitForSerial();
      
      Serial.print("  D - set the time between (D)isplay console reports [s], e.g. D1.5\n");
      Serial.print("  E - fully (E)xtend actuators to slack lines.\n");
      Serial.print("  K - (K)ill the current gesture!\n");
      Serial.print("  P - move actuators to (P)ositions [mm], e.g. p90,95,96,102,105,107\n"); 
      Serial.print("  R - fully (R)etract actuators, unhook lines to avoid damage!!!\n");
      Serial.print("  T - (T)ension actuators to starting levels of about ");
      Serial.print(START_TENSION); Serial.print(" N\n");
      Serial.print("  V - toggle between (V)erbose mode and CSV console reports.\n");
      Serial.print("  W - calibrate with (W)eights, show raw analog outputs.\n\n");
      waitForSerial();
      
      Serial.print("  h - print this help message.\n\n");
      Serial.print("  All units in mks and degrees, except as noted.\n\n");
      break;
  }
  Serial.print("\nNow pending: (type 'h' for help)\n");
  showGestureItem(-1,pendingGestureName,pendingGesture);
  return 0;     // no need to change what you are doing
}

void showTriplet(float * g){
  Serial.print("("); Serial.print(g[0]);Serial.print(", "); 
                     Serial.print(g[1]);Serial.print(", "); 
                     Serial.print(g[2]);Serial.print(")");
}

void showGestureItem(int i,String n,float *g){
        if(i >= 0) Serial.print(i); 
        Serial.print(" ["); Serial.print(n); Serial.print("]: ");
        showGesture(g);
}

void showGesture(float *g){
  unsigned t = g[0];
  if(t > GTYPES) t = 0;
  Serial.print(gestureTypeNames[t]);
  if(t != 0){   // only print details if non-null 
    Serial.print(" t = "); Serial.print(g[1],0);
    Serial.print("s, n = "); Serial.print(g[2],0);
    Serial.print(", dir = "); Serial.print(g[3],0);
    Serial.print(", c = "); showTriplet(g+4);
    Serial.print(", f = "); showTriplet(g+7);
    for(int j = 10;j < GPARMS;j++){
      Serial.print("  ");
      Serial.print(g[j]);
    }
  }
  Serial.print("\n");  
}

// update the target angles ang, for gesture g, based on time t, since start of motion in seconds
int updateTargetAngles(float t, float *g, float *ang){
  int gestureType = g[0];
  float gestureDuration = g[1];                 // the duration in seconds for one gesture cycle
  int repeats = g[2];                           // the number of gesture cycles to complete
  float dir = 0;
  if(g[3] != 0) dir = g[3] / abs(g[3]);         // +/- 1 for rotation direction about y
  float progress = t / gestureDuration;         // normalized progress variable
  progress = progress - (int) progress;         // stay between 0 and 1 on repeats
  if(t > gestureDuration * repeats) return -1;  // the gesture is over
  switch(gestureType){
    case POINT:{            // go to a point
      for(int i = 0;i < 3;i++) ang[i] = g[i+4]; // set the point
      break;
    }
    case CIRCUMDUCTION:{    // an ellipsoidal circumduction at a variable angle
      // start with a simple circle, but need to figure out the rest of the parameters
      // there should be a variable centre
      float radius = g[7];
      ang[0] = radius * sin(2*PI*progress) + g[4];
      ang[1] = 0.;
      ang[2] = - radius * cos(2*PI*progress) * dir + g[6];
      break;
    }
    case LINE:{             // a straight line between two angular extremes and return 
      if(progress < 0.5){   // go from the first point to the second
        ang[0] = g[4] + (g[7]-g[4]) * 2 * progress;
        ang[1] = g[5] + (g[8]-g[5]) * 2 * progress;
        ang[2] = g[6] + (g[9]-g[6]) * 2 * progress;
      } else {              // go from the second point back to the first
        progress = progress - 0.5;  
        ang[0] = g[7] + (g[4]-g[7]) * 2 * progress;
        ang[1] = g[8] + (g[5]-g[8]) * 2 * progress;
        ang[2] = g[9] + (g[6]-g[9]) * 2 * progress;        
      }
      break;
    }
    default:
      return -2;        // unrecognized gesture type
      break;
  }
  return 0;             // a new angle value has been set
}
