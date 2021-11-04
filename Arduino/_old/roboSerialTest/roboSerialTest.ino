
uint32_t messTime, yellow_time;
uint32_t messHz = 100;
uint16_t messInt = 1000/messHz, yellow_interval = 500;
uint32_t numMessages = 0;
bool it_is_time(uint32_t t, uint32_t t0, uint16_t dt);

void setup() {
  // put your setup code here, to run once:
  messTime = millis();
  Serial.begin(115200);
  establishContact();  // send a byte to establish contact until receiver responds
  delay(50);
}

void loop() {
  uint32_t currTime;
  currTime = millis();
  if (it_is_time(currTime, messTime, messInt)) {
    numMessages += 1;
    messTime = currTime;
    //////////////////
    //Send Messages///
    //////////////////
    Serial.print('<');
    Serial.print(numMessages);
    Serial.println('>');
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
