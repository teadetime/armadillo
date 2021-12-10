/////////////////
//Stepper Setup//
/////////////////
#include <AccelStepper.h>
#include <Servo.h>
AccelStepper stepper1(1, 3, 2); //Step dir?
AccelStepper stepper2(1, 5, 4);
AccelStepper stepper3(1, 9, 8);
Servo servoEOF;

///////////////////////
//Potentiometer Setup//
///////////////////////
#define pot0 A0
#define pot1 A1
#define pot2 A2
#define potButton A3

const bool usePotentiometerAdjust = true;

int pot0val = 0;
int pot1val = 0;
int pot2val = 0;
bool potButtonVal = false;


float j1PC_adjust = 0.0;
float j2PC_adjust = 0.0;
float j3PC_adjust = 0.0;

float potSensitivity = 60.0 / 1024;

void readPots() {
  if(usePotentiometerAdjust) {
    pot0val = analogRead(pot0);
    pot1val = analogRead(pot1);
    pot2val = analogRead(pot2);

    j1PC_adjust = (pot0val - 1024 / 2) * potSensitivity;
    j2PC_adjust = (pot1val - 1024 / 2) * potSensitivity;
    j3PC_adjust = (pot2val - 1024 / 2) * potSensitivity;
    potButtonVal = 1 - digitalRead(potButton);
  }
}

///////////////////////////
//Vars for Limit Switches//
///////////////////////////

const int j1_limitPin = 10;
const int j2_limitPin = 6;
const int j3_limitPin = 7;
const int servoPin = 11;
const int pumpPin = 12;
const int vacPin = 13;

const int servoZero = 74;         // Lets sero the servo at the center of its range
int servoPos = 0;

bool j1_limitVal = 0;                  // In current config, switch will go low when pressed
bool j2_limitVal = 1;
bool j3_limitVal = 1;
bool j1Homed = false;
bool j2Homed = false;
bool j3Homed = false;

////////////////////////////
//Vars for Incoming Serial//
////////////////////////////
char objectiveType = 'Z';           // Objective type, must be one of messChar***
const char messCharMove = 'M';      // First Byte in a message sent that has robot state data
const char messCharInfo = 'I';      // First Byte in data the indicates something about an objective
const char messCharHome = 'H';          // First Byte for messages regarding homing
const char messCharTest = 'T';
const char messCharOther = 'O';
const char messCharSuccess = 'Y';
const char messCharFail = 'N';
const char messCharCalibrate = 'B';

const char startMarker = '<';
const char endMarker = '>';
const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];   // temporary array for use when parsing
// variables to hold the parsed data
float j1PC = 0.0;
float j2PC = 0.0;
float j3PC = 0.0;
float j4PC = 0.0;
float vacPC = 0.0;
float speedPC = 0.0;
boolean newData = false;

bool moving = false;        //Set to true to if executing a move command
boolean objectiveInProgress = false;  // Switches to true when waiting to send a specific response
boolean debug = true;       // Flag for debuggin data NOT IN USE

byte microStep = 16;
int stepsRev = 600;

/////////////////////////////////
//Used for reporting RobotState//
/////////////////////////////////
uint32_t messTime;
uint32_t objectiveStartTime;
uint32_t messHz = 100;
uint16_t messInt = 1000 / messHz;
uint32_t numMessages = 0;
bool it_is_time(uint32_t t, uint32_t t0, uint16_t dt);

void setup() {
  // put your setup code here, to run once:

  // need to use INPUT_PULLUP here to have hardware debouncing
  pinMode(j1_limitPin, INPUT_PULLUP);
  pinMode(j2_limitPin, INPUT_PULLUP);
  pinMode(j3_limitPin, INPUT_PULLUP);
  servoEOF.attach(servoPin);
  servoEOF.write(servoZero);
  readLimitSwitches();

  pinMode(pumpPin, OUTPUT);
  pinMode(vacPin, OUTPUT);

  messTime = millis();
  stepper1.setMaxSpeed(800);
  stepper1.setSpeed(700);
  stepper1.setCurrentPosition(0);
  stepper1.setAcceleration(300);

  stepper2.setMaxSpeed(1000);
  stepper2.setSpeed(300);
  stepper2.setCurrentPosition(0);
  stepper2.setAcceleration(100);

  stepper3.setMaxSpeed(1000);
  stepper3.setSpeed(500);
  stepper3.setCurrentPosition(0);
  stepper3.setAcceleration(200);

  objectiveStartTime = messTime;    // Set this to the current time
  Serial.begin(115200);     // Fast Baud to send data more quickly!
  establishContact();       // send a byte to establish contact until receiver responds
  //digitalWrite(pumpPin, HIGH);
  Serial.println();
  //delay(50);
}

void loop() {
  uint32_t currTime;
  currTime = millis();
  // ALwatys have the steppers stay put
  stepper1.run();
  stepper2.run();
  stepper3.run();

  // Only Send Status if we are moving
  if (moving && objectiveInProgress && it_is_time(currTime, messTime, messInt)) {
    numMessages += 1;
    messTime = currTime;
    //////////////////
    //Send Messages///
    //////////////////
    if (!debug) {
      sendRobotState(numMessages);
    }
  }
  ///////////////////////////
  //Check for new objective//
  ///////////////////////////
  if (!objectiveInProgress) {
    recvWithStartEndMarkers();
    setVac(vacPC);

    if (newData == true) {
      strcpy(tempChars, receivedChars);     // Temporary copy needed for parsing
      parseData();
      showParsedData();
      newData = false;
      objectiveInProgress = true;
      objectiveStartTime = currTime;

      if (objectiveType == messCharMove) {
        // TODO PUT THIS IN A FUNCTION THAT CALCULATES DEGREES TO STEPS
        moving = true;
//        Serial.println("movingSteppers");
        //THESE BREAK EVERYTHING
        //SHOULD REPLACE WITH VARIABLES
        stepper1.setSpeed(700);
        stepper2.setSpeed(300);
        stepper3.setSpeed(500);
        //          stepper1.setSpeed(speedPC);
        //          stepper2.setSpeed(speedPC);
        //          stepper1.setAcceleration(vacPC);
        //          stepper2.setAcceleration(vacPC);
        stepper1.moveTo(j1PC + j1PC_adjust);
        stepper2.moveTo(j2PC + j2PC_adjust);
        stepper3.moveTo(j3PC + j3PC_adjust);
        servoPos = int(j4PC)+servoZero; //This may need to be minus
        servoEOF.write(servoPos);

         // checking for limit switch hit during general movement - this should happen multiple times in the main loop???
        readLimitSwitches();
        if(j1_limitVal == 0) {
          stepper1.stop();
        }
        if(j1_limitVal == 1) {
          stepper2.stop();
        }
        if(j1_limitVal == 1) {
          stepper3.stop();
        }

      }
      if (objectiveType == messCharHome) {
        j1Homed = false;
        j2Homed = false;
        j3Homed = false;
        stepper1.setSpeed(100);
        stepper2.setSpeed(100);
        stepper3.setSpeed(100);
        Serial.println("Sending Moveto");
        stepper1.moveTo(stepsRev*microStep);
        stepper2.moveTo(stepsRev*microStep);
        stepper3.moveTo(stepsRev*microStep);
        servoPos = servoZero;
        servoEOF.write(servoPos);
      }
    }
  }
  ////////////////
  //Do Objective//
  ////////////////
  if (objectiveInProgress) {
    switch (objectiveType) {
      case messCharMove:
      
         
        if (!motorsMoving()) {
          // Send Objective completed message
          objectiveInProgress = false;
          moving = false;
          //TODO CHECK IF START POSITION IS LIKE FINAL POSITION?
          sendObjectiveCompleted(objectiveType, messCharSuccess);
        }
        // shouldn't need this line
        // setVac(vacPC);
        break;
      case messCharHome:
        homingLoop();

        if(j1Homed==1 && j2Homed==1 && j3Homed==1){
          resetSteppers(j1PC,j2PC,j3PC,j4PC);
          sendObjectiveCompleted(objectiveType, messCharSuccess);
          moving = false;
          objectiveInProgress = false;
        }
        break;
      case messCharInfo:
        sendRobotState(numMessages);
        objectiveInProgress = false;
        sendObjectiveCompleted(objectiveType, messCharSuccess);
        break;
      default:
        break;
      case messCharCalibrate:
        readPots();
        stepper1.moveTo(j1PC + j1PC_adjust);
        stepper2.moveTo(j2PC + j2PC_adjust);
        stepper3.moveTo(j3PC + j3PC_adjust);
        if (potButtonVal) {
          break;
        }
    }

  }

}

void establishContact() {

  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
  // Get rid of incoming data
  serialFlush();
}

void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}

/*
** Returns a boolean value that indicates whether the current time, t, is later than some prior
** time, t0, plus a given interval, dt.  The condition accounts for timer overflow / wraparound.
*/
bool it_is_time(uint32_t t, uint32_t t0, uint16_t dt) {
  return ((t >= t0) && (t - t0 >= dt)) ||         // The first disjunct handles the normal case
         ((t < t0) && (t + (~t0) + 1 >= dt));  //   while the second handles the overflow case
}

//////////////////
//Serial Parsing//
//////////////////
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char rc;
  //TODO ADD IF CHECK IF SERILA>AVAILABLE >10
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}


void parseData() {      // split the data into its parts
  //Example <M,0,0,0,0,1,50>
  char * strtokIndx; // this is used by strtok() as an index


  objectiveType = tempChars[0]; // char(strtokIndx);     // convert this part to a float
  strtokIndx = strtok(tempChars, ",");     // get the first part - the string we already put this into a char
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  j1PC = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  j2PC = atof(strtokIndx);     // convert this part to an float

  strtokIndx = strtok(NULL, ",");
  j3PC = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  j4PC = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  vacPC = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  speedPC = atof(strtokIndx);     // convert this part to a float

}

//============

void showParsedData() {
  Serial.print("messChar: ");
  Serial.println(objectiveType);
  Serial.print("J1: ");
  Serial.println(j1PC);
  Serial.print("J2: ");
  Serial.println(j2PC);
  Serial.print("J3: ");
  Serial.println(j3PC);
  Serial.print("J4: ");
  Serial.println(j4PC);
  Serial.print("Vac: ");
  Serial.println(vacPC);
  Serial.print("speed: ");
  Serial.println(speedPC);
}

void sendObjectiveCompleted(char objective, char success) {
  Serial.print(startMarker);
  Serial.print(objective);
  Serial.print(',');
  Serial.print(success);
  Serial.println(endMarker);
}

void sendRobotState(int message) {
  char sep = ',';
  Serial.print(startMarker);
  Serial.print(messCharInfo);
  Serial.print(sep);
  Serial.print(stepper1.currentPosition());
  Serial.print(sep);
  Serial.print(stepper2.currentPosition());
  Serial.print(sep);
  Serial.print(stepper3.currentPosition());
  Serial.print(sep);
  Serial.print(servoEOF.read());
  Serial.println(endMarker);
}

bool motorsMoving() {
  return (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning() || servoEOF.read() != servoPos );
  // return (stepper1.isRunning() || stepper2.isRunning()|| stepper3.isRunning() || servoSPINNING);
}

void setVac(float val) {
  // bool boolVal = val == 1.0;
  digitalWrite(vacPin, val);
}

/////////////////////////////
//Used When Homing the steppers//
/////////////////////////////
void resetSteppers(float theta1,float theta2,float theta3, float theta4) {
  stepper1.setCurrentPosition(theta1);
  stepper2.setCurrentPosition(theta2);
  stepper3.setCurrentPosition(theta3);
  //TODO 0 of the servo will be constant
}

void readLimitSwitches() {
    j1_limitVal = digitalRead(j1_limitPin);
    j2_limitVal = digitalRead(j2_limitPin);
    j3_limitVal = digitalRead(j3_limitPin);
}

void homingLoop(){
  readLimitSwitches();
  if (j1_limitVal == 1 && !j1Homed) {
      stepper1.stop();
      //stepper1.moveTo(stepper1.currentPosition());
      stepper1.setCurrentPosition(j1PC);
      j1Homed = true;
    }
  if (j2_limitVal == 0 && !j2Homed) {
    //  Serial.println(j2_limitVal);
      stepper2.stop();
      stepper2.setCurrentPosition(j2PC);
      //stepper2.moveTo(stepper2.currentPosition());
      j2Homed = true;
  }
  if (j3_limitVal == 0 && !j3Homed) {
    //  Serial.println(j3_limitVal);
      stepper3.stop();
      stepper3.setCurrentPosition(j3PC);
      //stepper3.moveTo(stepper3.currentPosition());
      j3Homed = true;
  }
}

void homingProcedureBlocking() {
  // so far this homes each joint individually

  //homes j1
  stepper1.moveTo(10000);
  readLimitSwitches();
  while (digitalRead(j1_limitPin) == 0) {
    stepper1.run();
    readLimitSwitches();
    //Serial.println(j1_limitVal);
    if (j1_limitVal == 1) {
    //  Serial.println(j1_limitVal);
      stepper1.stop();
      j1Homed = true;
    }
  }

  // homes j2
  stepper2.moveTo(10000);
  readLimitSwitches();
  while (digitalRead(j2_limitPin) == 1) {
    stepper2.run();
    readLimitSwitches();
    //Serial.println(j2_limitVal);
    if (j2_limitVal == 0) {
    //  Serial.println(j2_limitVal);
      stepper2.stop();
      j2Homed = true;
    }
  }

  // homes j3
  stepper3.moveTo(10000);
  readLimitSwitches();
  while (digitalRead(j3_limitPin) == 1) {
    stepper3.run();
    readLimitSwitches();
    //Serial.println(j3_limitVal);
    if (j3_limitVal == 0) {
      //Serial.println(j3_limitVal);
      stepper3.stop();

      j3Homed = true;
    }
  }

}
