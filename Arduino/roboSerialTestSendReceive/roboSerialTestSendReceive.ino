/////////////////
//Stepper Setup//
/////////////////
#include <AccelStepper.h>
AccelStepper stepper1(1, 3, 2);
AccelStepper stepper2(1, 5, 4);
AccelStepper stepper3(1, 9, 8);


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
  messTime = millis();
  stepper1.setMaxSpeed(1000);
  stepper1.setSpeed(1000);
  stepper1.setCurrentPosition(0);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(1000);
  stepper2.setSpeed(1000);
  stepper2.setCurrentPosition(0);
  stepper2.setAcceleration(500);
  
  stepper3.setMaxSpeed(1000);
  stepper3.setSpeed(1000);
  stepper3.setCurrentPosition(0);
  stepper3.setAcceleration(500);

  objectiveStartTime = messTime;    // Set this to the current time
  Serial.begin(115200);     // Fast Baud to send data more quickly!
  establishContact();       // send a byte to establish contact until receiver responds
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
    if (newData == true) {
      strcpy(tempChars, receivedChars);     // Temporary copy needed for parsing
      parseData();
      //showParsedData();
      newData = false;
      objectiveInProgress = true;
      objectiveStartTime = currTime;

      if (objectiveType == messCharMove) {
        // TODO PUT THIS IN A FUNCTION THAT CALCULATES DEGREES TO STEPS
        moving = true;
//        Serial.println("movingSteppers");
        //THESE BREAK EVERYTHING
        //          stepper1.setSpeed(speedPC);
        //          stepper2.setSpeed(speedPC);
        //          stepper1.setAcceleration(vacPC);
        //          stepper2.setAcceleration(vacPC);
        stepper1.moveTo(j1PC);
        stepper2.moveTo(j2PC);
        stepper3.moveTo(j3PC);

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
        break;
      case messCharHome:
        resetSteppers(j1PC,j2PC,j3PC,j4PC);
        sendObjectiveCompleted(objectiveType, messCharSuccess);
        objectiveInProgress = false;
        break;
      case messCharTest:
        resetSteppers(j1PC,j2PC,j3PC,j4PC);
        sendObjectiveCompleted(objectiveType, messCharSuccess);
        objectiveInProgress = false;
        break;
      case messCharInfo:
        sendRobotState(numMessages);
        objectiveInProgress = false;
        sendObjectiveCompleted(objectiveType, messCharSuccess);
        break;
      default:
        break;
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
  //  Serial.print(sep);
  //  Serial.print(stepper1.currentPosition());
  Serial.println(endMarker);
}

bool motorsMoving() {
  return (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning() );
  // return (stepper1.isRunning() || stepper2.isRunning()|| stepper3.isRunning() || servoSPINNING);
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
