#define j1_limitPin 5
#define j2_limitPin 6
#define j3_limitPin 7

/////////////////
//Stepper Setup//
/////////////////
#include <AccelStepper.h>
AccelStepper stepper1(1, 3, 2);
AccelStepper stepper2(1, 5, 4);
AccelStepper stepper3(1, 9, 8);


bool j1_limit = 0;
bool j2_limit = 0;
bool j3_limit = 0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(j1_limitPin, INPUT);
  pinMode(j2_limitPin, INPUT);
  pinMode(j3_limitPin, INPUT);

  messTime = millis();
  stepper1.setMaxSpeed(2000);
  stepper1.setSpeed(4000);
  stepper1.setCurrentPosition(0);
  stepper1.setAcceleration(1500);

  stepper2.setMaxSpeed(2000);
  stepper2.setSpeed(4000);
  stepper2.setCurrentPosition(0);
  stepper2.setAcceleration(1500);

  stepper3.setMaxSpeed(1000);
  stepper3.setSpeed(1000);
  stepper3.setCurrentPosition(0);
  stepper3.setAcceleration(1000);

}





void readLimits() {
    j1_limit = digitalRead(j1_limitPin);
    j2_limit = digitalRead(j2_limitPin);
    j3_limit = digitalRead(j3_limitPin);
}

void goToHome() {

    while (!j1_limit || !j2_limit || !j3_limit) {
        readLimits();

        if (!j1_limit) {
            setSpeed(j1, "slow");
        }
        else {
            setspeed(j1, "stop");
        }

        if (!j2_limit) {
            setSpeed(j2, "slow");
        }
        else {
            setspeed(j2, "stop");
        }

        if (!j3_limit) {
            setSpeed(j3, "slow");
        }
        else {
            setspeed(j3, "stop");
        }
    }
}

void loop() {

}