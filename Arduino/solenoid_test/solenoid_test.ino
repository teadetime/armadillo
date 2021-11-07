int solenoidPin = 5;                    //This is the output pin on the Arduino
int vacPin = 6;

void setup() 
{
  pinMode(solenoidPin, OUTPUT);
  pinMode(vacPin, OUTPUT);          //Sets that pin as an output
  digitalWrite(vacPin, HIGH);
}

void loop() 
{
  digitalWrite(solenoidPin, HIGH);      //Switch Solenoid ON
  delay(5000);                          //Wait 1 Second
  digitalWrite(solenoidPin, LOW);       //Switch Solenoid OFF
  delay(5000);                          //Wait 1 Second
}
