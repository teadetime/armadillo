int solenoidPin = 2;                    //This is the output pin on the Arduino
int vacPin = 3;

void setup()
{
  Serial.begin(9600);
  pinMode(solenoidPin, OUTPUT);
  pinMode(vacPin, OUTPUT);          //Sets that pin as an output
  digitalWrite(vacPin, HIGH);
  digitalWrite(solenoidPin, HIGH);
}

void loop()
{
//  digitalWrite(vacPin, HIGH);
//  delay(2000);
//  digitalWrite(solenoidPin, LOW);      //Switch Solenoid ON
//  delay(2000);
//  digitalWrite(vacPin,LOW);
//  delay(2000);
//  digitalWrite(solenoidPin, HIGH);       //Switch Solenoid OFF
//  delay(2000);                          //Wait 5 Second

  // digitalWrite(solenoidPin, LOW);      //Switch Solenoid ON
  // delay(6000);
  // digitalWrite(solenoidPin, HIGH);       //Switch Solenoid OFF
  // delay(6000);                          //Wait 1 Second
}
