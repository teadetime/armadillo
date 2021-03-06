////////////////////////////
//Vars for Incoming Serial//
////////////////////////////
char codeState = 'S';           // First Byte in a message sent that has robot state data
char codeObjectiveStatus = 'O'; // First Byte in data the indicates something about an objective
char codeHoming ='H';           // First Byte for messages regarding homing

char startMarker = '<';
char endMarker = '>';
const byte numChars = 36;
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

boolean objective = false;  // Switches to true when there is a position to drive to 
boolean debug = false;      // Flag for debuggin data NOT IN USE


/////////////////////////////////
//Used for reporting RobotState//
/////////////////////////////////
uint32_t messTime;
uint32_t objectiveStartTime;
uint32_t messHz = 100;
uint16_t messInt = 1000/messHz;
uint32_t numMessages = 0;
bool it_is_time(uint32_t t, uint32_t t0, uint16_t dt);

void setup() {
  // put your setup code here, to run once:
  messTime = millis();
  objectiveStartTime = messTime;    // Set this to the current time
  Serial.begin(115200);     // Fast Baud to send data more quickly!
  establishContact();       // send a byte to establish contact until receiver responds
  //delay(50);
}

void loop() {
  uint32_t currTime;
  currTime = millis();
  
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);     // Temporary copy needed for parsing
      parseData();
      showParsedData();
      newData = false;
      objective = true;
      objectiveStartTime = currTime;
  }
  if (it_is_time(currTime, messTime, messInt)) {
    numMessages += 1;
    messTime = currTime;
    //////////////////
    //Send Messages///
    //////////////////
    if(!debug){
      sendRobotState(numMessages);
    }
  }

  //////////////////
  //Movement code!//
  //////////////////
  if(objective){
    // WE WILL DO stepper thingies here

    
    if(it_is_time(currTime, objectiveStartTime, 1000)){ 
      // Hardcode a move to take 1000ms This will be end condition saying we are at the position and the vacuum is at the same state as the request
      objective = false;
      // Send Objective completed message
      sendObjectiveCompleted();
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

void serialFlush(){
  while(Serial.available() > 0) {
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
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
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

void sendObjectiveCompleted() {
  Serial.print(startMarker);
  Serial.println(endMarker);
}

void sendRobotState(int message) {
  char sep = ',';
  Serial.print(startMarker);
//  Serial.print(j1);
//  Serial.print(sep);
//  Serial.print(j2);
//  Serial.print(sep);
//  Serial.print(j3);
//  Serial.print(sep);
//  Serial.print(j4);
//  Serial.print(sep);
//  Serial.print(vac);
//  Serial.print(sep);
//  Serial.print(speed);
  Serial.print(message);
  Serial.print(sep);
  Serial.print(message);
  Serial.print(sep);
  Serial.println(endMarker);
}
