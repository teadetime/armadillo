int limit_switch_1 = 6;


void setup() {
  Serial.begin(115200);
  pinMode(limit_switch_1, INPUT_PULLUP);

}

void loop() {
  while (digitalRead(limit_switch_1) == HIGH) {
         Serial.println(digitalRead(limit_switch_1));
        
        if (digitalRead(limit_switch_1) == LOW) {
          Serial.println(digitalRead(limit_switch_1));
        }
  }
//  Serial.println("limit switch hit!");
   
  
}
