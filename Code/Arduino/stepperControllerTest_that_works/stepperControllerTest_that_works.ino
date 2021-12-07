// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

AccelStepper stepper1(1,5,4);
AccelStepper stepper2(1,3,2);



void setup()
{  
   stepper1.setMaxSpeed(1000);
   stepper1.setSpeed(500);	
   stepper1.setCurrentPosition(0);
   stepper1.setAcceleration(25);
   stepper1.moveTo(-17);

   stepper2.setMaxSpeed(1000);
   stepper2.setSpeed(500); 
   stepper2.setCurrentPosition(0);
   stepper2.setAcceleration(25);
   stepper2.moveTo(34);
}

void loop()
{  
   stepper1.run();
   stepper2.run();
}
