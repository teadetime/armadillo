int limit_switch_1 = 6;
int limit_switch_2 = 7;



void setup() {
  Serial.begin(115200);
  pinMode(limit_switch_1, INPUT_PULLUP);
  pinMode(limit_switch_2, INPUT_PULLUP);


}

void loop() {

  if (digitalRead(limit_switch_1) == LOW) {
      Serial.println("limit 1 HIT");
//          Serial.println(digitalRead(limit_switch_1));
        }
  else if (digitalRead(limit_switch_2) == LOW) {
          Serial.println("limit 2 HIT");

//          Serial.println(digitalRead(limit_switch_2));
        }
  else {
  Serial.println("Nothing HIT");
  }
}
   
  
