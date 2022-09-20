#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// Let Multistepper handled up to 10 steppers
MultiStepper steppers;

//**** ARDUINO PINS *****

//***** X-Axis *****
#define X_STEP_PIN         23 // Step
#define X_DIR_PIN          24 // Direction
#define X_LIMIT            46 // Limit Switch

//***** Y-Axis *****
#define Y_STEP_PIN         27 // Step
#define Y_DIR_PIN          28 // Direction
#define Y_LIMIT            42 // Limit Switch

//***** Z-Axis *****
#define Z_STEP_PIN         32 // Step
#define Z_DIR_PIN          33 // Direction
#define Z_LIMIT            45 // Limit Switch

//***** P-Axis (Servo Motor) *****
Servo myServo;
int pos = 0;
byte servoPin = 8;
byte servoMin = 70;
byte servoMax = 170;
byte servoPos = 70;
byte newServoPos = servoMax;
//byte newServoPos = servoMin;

//***** Create new instance of AccelStepper class *****
AccelStepper stepperX(AccelStepper::DRIVER,X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// NOT YET SURE WHAT THIS DOES
int changeDirection = 1;

// Determine if limit switch has been pressed
volatile bool shouldMoveMotor = true;

// ***** Variables for Serial Communication with Computer *****

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
char mode[6];
int integerFromPC = 0;
float floatFromPC = 0.0;
boolean newData = false;

float servoFraction = 0.0; // fraction of servo range to move

//***** Limit Switches *****
//variables to hold the limit switch states
boolean homeX;
boolean homeY;
boolean homeZ;

// Belt drive
// 1mm = (no. of steps per rev)/(linear distance moved per rev)
// 1/4th resolution so 1mm = (200*4)/40 = 20 steps/mm

// Lead screw drive
// 1mm = (no. of steps per rev)/(linear distance moved per rev)
// 1/4th resolution so 4mm = (200*4)/1 = 800 steps/mm
// 1mm = 800/4 = 200 steps/mm
// Each revolution corresponds to 4mm of travel (see https://engineering.stackexchange.com/questions/40995/how-do-i-calculate-the-ideal-travel-distance-of-the-ball-screw-per-revolution-of)
// Account for pitch of lead screw = 2 mm (divide by this)
// 200/2 = 100 steps/mm


//***** STEP PER MM *****
//Its important that these values are correct
//so that the digital coordinate system = matches real world 
//Total step per revolution = full steps/rev * microsteps
//200 full steps * 16 microsteps = 3200 digital steps
int x_steps_mm = 20;
int y_steps_mm = 20;
int z_steps_mm = 100;


// variables to hold location data
double locX_in_mm;
double locY_in_mm;
double locZ_in_mm;

int locX_in_steps;
int locY_in_steps;
int locZ_in_steps;


void setup() {
    
    Serial.begin(9600);
    while(!Serial);

    //***** Setting up Switches, Sensors, and Buttons *****
//    pinMode(X_LIMIT, INPUT); 
//    pinMode(Y_LIMIT, INPUT); 
//    pinMode(Z_LIMIT, INPUT); 
    
    stepperX.setMaxSpeed(1500);
    stepperX.setAcceleration(1000);
    stepperX.setPinsInverted(true, false, true); // (directionInvert, stepInvert, enableInvert)
    stepperX.enableOutputs();

    stepperY.setMaxSpeed(1500);
    stepperY.setAcceleration(1000);
    stepperY.setPinsInverted(false, false, true); // (directionInvert, stepInvert, enableInvert)
    stepperY.enableOutputs();

    stepperZ.setMaxSpeed(1200);
    stepperZ.setAcceleration(1000); 
    stepperZ.setPinsInverted(false, false, true); // (directionInvert, stepInvert, enableInvert)
    stepperZ.enableOutputs();

    // Multistepper
    steppers.addStepper(stepperX);
    steppers.addStepper(stepperY);

    // Servo Motor
    myServo.attach(servoPin);

    // Limit
    pinMode(X_LIMIT, INPUT);
    attachInterrupt(digitalPinToInterrupt(X_LIMIT), triggerStopMotor, RISING);
    pinMode(Y_LIMIT, INPUT);
    attachInterrupt(digitalPinToInterrupt(Y_LIMIT), triggerStopMotor, RISING);
    pinMode(Z_LIMIT, INPUT);
    attachInterrupt(digitalPinToInterrupt(Z_LIMIT), triggerStopMotor, RISING);

}

void loop() {
    shouldMoveMotor = true;
    getDataFromPC();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        coordinateMove();
        newData = false;
    }
}

void triggerStopMotor() {
  Serial.println("Interrupt Triggered");
  shouldMoveMotor = false;
}

//============

void getDataFromPC() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
          if (rc != endMarker) {
              receivedChars[ndx] = rc; // Keep populating the array until the end marker is found
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          }
          else {
              receivedChars[ndx] = '\0'; // if the end marker is found then terminate the string
              recvInProgress = false;
              ndx = 0; // reset position for the next message
              newData = true;
          }
      }

      else if (rc == startMarker) {
          recvInProgress = true;
      }
  }
}

//============

void parseData() {      // split the data into its parts

    //***** Poll the current location of the stepper motors *****
    locX_in_mm = stepperX.currentPosition() / x_steps_mm;
    locY_in_mm = stepperY.currentPosition() / y_steps_mm;
    locZ_in_mm = stepperZ.currentPosition() / z_steps_mm;
    
    char * strtokIndx; // this is used by strtok() as an index
    
    strtokIndx = strtok(tempChars," ");
    strcpy(mode, strtokIndx);
    strtokIndx = strtok(NULL," ");

    // Servo 
    byte servoRange = servoMax - servoMin;
    if (servoFraction >= 0 && servoFraction <= 1) {
//      newServoPos = servoMin + ((float) servoRange * servoFraction);
      newServoPos = servoMax - ((float) servoRange * servoFraction);
    }
    
    while (strtokIndx != NULL){
      if (strtokIndx[0] == 'X'){
        memmove(strtokIndx, strtokIndx+1, strlen(strtokIndx));
        locX_in_mm = atof(strtokIndx); 
      }
      if (strtokIndx[0] == 'Y'){
        memmove(strtokIndx, strtokIndx+1, strlen(strtokIndx));
        locY_in_mm = atof(strtokIndx); 
      }
      if (strtokIndx[0] == 'Z'){
        memmove(strtokIndx, strtokIndx+1, strlen(strtokIndx));
        locZ_in_mm = atof(strtokIndx); 
      }
      if (strtokIndx[0] == 'P') {
        memmove(strtokIndx, strtokIndx+1, strlen(strtokIndx));
        servoFraction = atof(strtokIndx);
      }
    strtokIndx = strtok (NULL, " ");
    }
    
    //***** mm to steps conversions *****
    locX_in_steps = locX_in_mm * x_steps_mm;
    locY_in_steps = locY_in_mm * y_steps_mm;
    locZ_in_steps = locZ_in_mm * z_steps_mm;

    Serial.print("X: ");
    Serial.print(locX_in_mm);
    Serial.print("    Y: ");
    Serial.print(locY_in_mm);
    Serial.print("    Z: ");
    Serial.print(locZ_in_mm);
    Serial.print("    Servo Fraction: ");
    Serial.print(servoFraction);
    Serial.print(", Servo Position: ");
    Serial.print(newServoPos);
    Serial.print("\n");
}

//============

void coordinateMove(){
  stepperX.setMaxSpeed(500);
  stepperX.setAcceleration(1000);
  stepperY.setMaxSpeed(500);
  stepperY.setAcceleration(1000);
  stepperZ.setMaxSpeed(500);
  stepperZ.setAcceleration(1000);
  if (strcmp(mode, "G1") == 0){
    
    stepperX.moveTo(locX_in_steps);
    stepperY.moveTo(locY_in_steps);
    stepperZ.moveTo(locZ_in_steps);

    stepperX.setMaxSpeed(1000);
    stepperX.setAcceleration(1000);
    stepperY.setMaxSpeed(1000);
    stepperY.setAcceleration(1000);
    stepperZ.setMaxSpeed(1000);
    stepperZ.setAcceleration(1000);

    while(stepperX.targetPosition() != stepperX.currentPosition() or stepperY.targetPosition() != stepperY.currentPosition() or stepperZ.targetPosition() != stepperZ.currentPosition()){
      if (shouldMoveMotor) {
        stepperX.run();
        stepperY.run();
        stepperZ.run();
        delay(0.001);
        }
        Serial.println("<ok>");
    }
  }

  //***** Pipette Servo *****
  else if (strcmp(mode, "M1") == 0){
    Serial.println(servoPos);
    Serial.println(newServoPos);
//    if (servoPos != newServoPos) {
    for (pos = servoPos; pos <= newServoPos; pos +=1) {
      myServo.write(pos);
      delay(20); // Wait 20ms to get to next pos
    }
    for (pos = servoPos; pos >= newServoPos; pos -= 1) { // goes from 180 degrees to 0 degrees
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(20);                       // waits 15 ms for the servo to reach the position
  }
  servoPos = newServoPos;
//    }
  }


  //***** G28 - Homing Cycle *****
  // For this code to work as written the limit switches must be located at
  // X - Maximum
  // Y - Maximum
  // Z - Maximum 
  else if (strcmp(mode, "G28") == 0){
    stepperX.setMaxSpeed(3000);
    stepperX.setAcceleration(1000);
    stepperY.setMaxSpeed(5000);
    stepperY.setAcceleration(5000);
    stepperZ.setMaxSpeed(2000);
    stepperZ.setAcceleration(1000);
     // Z-Axis Homing
    int initial_homing = stepperZ.currentPosition();
    homeZ = false;
    while (homeZ == false){
      initial_homing++;  
      stepperZ.moveTo(initial_homing);
      stepperZ.run(); 
      if (digitalRead(Z_LIMIT) != 0) {
        delay(1);
        if (digitalRead(Z_LIMIT) !=0){
          homeZ = true;
        }
      }
    }
//    stepperZ.setMaxSpeed(100.0);      
//    initial_homing=0;
//    while (digitalRead(Z_LIMIT) == p1) { 
//      initial_homing--;
//      stepperZ.move(initial_homing);  
//      stepperZ.runSpeed();
//    }
//    stepperZ.setCurrentPosition(0);
  
   // X-Axis Homing
   initial_homing = stepperX.currentPosition();
   homeX = false;
   while (homeX == false){
      initial_homing++;  
      stepperX.moveTo(initial_homing);
      stepperX.run(); 
      if (digitalRead(X_LIMIT) != 0) {
        delay(1);
        if (digitalRead(X_LIMIT) !=0){
          homeX = true;
        }
      
      }
    }
    stepperX.setMaxSpeed(100.0); 
    initial_homing=0;
    while (digitalRead(X_LIMIT) == 1) { 
      initial_homing--;
      stepperX.moveTo(initial_homing);  
      stepperX.run();
    }
    stepperX.setCurrentPosition(0);
  
  // Y-Axis Homing 
    initial_homing=stepperY.currentPosition();
    homeY = false;
    while (homeY == false){
      initial_homing++;  
      stepperY.moveTo(initial_homing);
      stepperY.run(); 
      if (digitalRead(Y_LIMIT) != 0) {
        delay(1);
        if (digitalRead(Y_LIMIT) !=0){
          homeY = true;
        }
      }
    }
    stepperY.setMaxSpeed(100.0);      
    initial_homing=0;
    while (digitalRead(Y_LIMIT) == 1) { 
      initial_homing--;
      stepperY.moveTo(initial_homing);  
      stepperY.run();
    }
    
    stepperY.setCurrentPosition(0);
    stepperY.setMaxSpeed(1000.0);      
  
  
    stepperX.setCurrentPosition(26400);
    stepperY.setCurrentPosition(144000);
    stepperZ.setCurrentPosition(0);
    stepperX.setMaxSpeed(10000);
    stepperX.setAcceleration(5000);
    stepperY.setMaxSpeed(10000);
    stepperY.setAcceleration(1500);
    stepperZ.setMaxSpeed(10000);
    stepperZ.setAcceleration(5000);
    Serial.println("<OK Homed>");
  }
  
}


