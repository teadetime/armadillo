#include <AccelStepper.h>

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
  setXYZ(100, 100, 25); // initializes the target desination and prepares the motors to move
}

void loop() {
    printTarget(); // what is the target destination in terms of arm angles?
    printMotors(); // what is the current location of all the angles?
    runMotors(); // needs to be run every loop cycle
}