#include <AccelStepper.h>

#define dirPin1 2
#define stepPin1 3
#define motorInterfaceType 1

#define dirPin2 4
#define stepPin2 5
#define motorInterfaceType 1

AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);



#define L1 180 // millimeters
#define L2 180 // millimeters
#define PI 3.1415926535

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution

AccelStepper motorThetaBase(stepsPerRevolution, 2, 3, 8, 9);
AccelStepper motorPhiBase(stepsPerRevolution, 4, 5, 8, 9);
AccelStepper motorPhiArm(stepsPerRevolution, 6, 7, 8, 9);

long phiBase;
long phiArm;
long thetaBase;

void setXYZ(long x, long y, long z) {
    long a = L1;
    long b = sqrt(x ^ 2 + z ^ 2 + y ^ 2);
    long c = L2;

    phiBase = -(acos((a ^ 2 + b ^ 2 - c ^ 2) / (2 * a * b)) + atan(z / sqrt(x ^ 2 + y ^ 2))) + PI / 2; // Base vertical
    phiArm = -acos((-cos(phiBase) * L1 + z) / L2); // Elbow
    thetaBase = asin(y / sqrt(x ^ 2 + y ^ 2)); // Base lateral

    setArm(thetaBase, phiBase, phiArm);
}

void setArm(long thetaBase, long phiBase, long phiArm) {
    motorThetaBase.moveTo(thetaBase);
    motorPhiBase.moveTo(phiBase);
    motorPhiArm.moveTo(phiArm);
}

void runMotors() {
    motorThetaBase.run();
    motorPhiBase.run();
    motorPhiArm.run();
}

void radToSteps(float rad) {
    return long(rad / (2 * PI) * stepsPerRevolution);
}

void printMotors() {
    Serial.print("motorThetaBase: "); Serial.print(motorThetaBase.currentPosition()); Serial.print("\t");
    Serial.print("motorPhiBase: "); Serial.print(motorPhiBase.currentPosition()); Serial.print("\t");
    Serial.print("motorPhiArm: "); Serial.print(motorPhiArm.currentPosition()); Serial.println("");
}

void printTarget() {
    Serial.print("thetaBase: "); Serial.print(thetaBase); Serial.print("\t");
    Serial.print("phiBase: "); Serial.print(phiBase); Serial.print("\t");
    Serial.print("phiArm: "); Serial.print(phiArm); Serial.println("");
}

int stepCount = 0;  // number of steps the motor has taken

void setup() {
  Serial.begin(115200);
  stepper1.setMaxSpeed(500);
  stepper1.setSpeed(50);
  stepper1.setCurrentPosition(0);

  stepper2.setMaxSpeed(500);
  stepper2.setCurrentPosition(0);

  long steppy = 50;

  stepper1.moveTo(steppy);



//  setXYZ(100, 100, 25); // initializes the target desination and prepares the motors to move
}

void loop() {
  stepper1.run();
//  stepper1.moveTo(steppy);
//  stepper1.run();
//  stepper2.moveTo(50);


//  stepper1.moveTo(-steppy);
//  stepper1.run();




  Serial.print("motorThetaBase: "); Serial.print(stepper1.currentPosition()); Serial.print("\t");
  Serial.print("motorPhiBase: "); Serial.print(stepper2.currentPosition()); Serial.print("\n");


//  while((stepper1.currentPosition() != 50) && (stepper2.currentPosition() != 50))
//  {
//    stepper1.setSpeed(50);
//    stepper2.setSpeed(50);
//
//    stepper1.runSpeed();
//    stepper2.runSpeed();
//  }
//
//  delay(1000);
//
//  stepper1.setCurrentPosition(0);
//  stepper2.setCurrentPosition(0);
//
//  while((stepper1.currentPosition() != -50) && (stepper2.currentPosition() != -50))
//  {
//    stepper1.setSpeed(-50);
//    stepper2.setSpeed(-50);
//
//    stepper1.runSpeed();
//    stepper2.runSpeed();
//  }
//  delay(1000);

//    printTarget(); // what is the target destination in terms of arm angles?
//    printMotors(); // what is the current location of all the angles?
//    runMotors(); // needs to be run every loop cycle
}
