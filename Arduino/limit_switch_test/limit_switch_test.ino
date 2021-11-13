int limit_switch_1 = 5;


void setup() {
  Serial.begin(9600);
  pinMode(limit_switch_1, INPUT_PULLUP);

}

void loop() {
  if (digitalRead(limit_switch_1) == LOW)
  {
    Serial.println("Activated!");
  }
 
  else
  {
    Serial.println("Not activated.");
  }
   
  delay(100);
}
