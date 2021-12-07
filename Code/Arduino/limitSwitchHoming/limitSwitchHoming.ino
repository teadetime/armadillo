#define j1_limitPin 6
#define j2_limitPin 7
#define j3_limitPin 8

/////////////////
//Stepper Setup//
/////////////////
#include <AccelStepper.h>
AccelStepper stepper1(1, 3, 2);
AccelStepper stepper2(1, 5, 4);
AccelStepper stepper3(1, 9, 8);


//in current config, limit switch will go low when pressed
bool j3_limit = 1;
bool j1_limit = 1;
bool j2_limit = 1;

const int SLOW_SPEED = 200;
const int FAST_SPEED = 200;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(j1_limitPin, INPUT_PULLUP);
  pinMode(j2_limitPin, INPUT_PULLUP);
  pinMode(j3_limitPin, INPUT_PULLUP);

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

    while (j1_limit || j2_limit || j3_limit) {
        readLimits();

        if (j1_limit) {
            stepper1.runSpeed(SLOW_SPEED);
        }
        else {
            stepper1.stop();
            stepper1.moveTo(currentPosition());
        }

        if (j2_limit) {
            stepper2.runSpeed(SLOW_SPEED);
        }
        else {
            stepper2.stop();
            stepper2.moveTo(currentPosition());
        }

        if (j3_limit) {
            stepper3.runSpeed(SLOW_SPEED);
        }
        else {
            stepper3.stop();
            stepper3.moveTo(currentPosition());
        }
    }
}

void loop() {
    stepper1.run();
    stepper2.run();
    stepper3.run();
}
