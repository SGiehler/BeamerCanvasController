#include "MultiHomeSpeedyStepper.h"


// ---------------------------------------------------------------------------------
//                                  Setup functions 
// ---------------------------------------------------------------------------------

//
// constructor for the stepper class
//
MultiHomeSpeedyStepper::MultiHomeSpeedyStepper()
{
  //
  // initialize constants
  //
  limitPin = 0;
}

//
// connect the stepper object to the IO pins
//  Enter:  stepPinNumber = IO pin number for the Step
//          directionPinNumber = IO pin number for the direction bit
//          enablePinNumber = IO pin number for the enable bit (LOW is enabled)
//            set to 0 if enable is not supported
//
void MultiHomeSpeedyStepper::connectToPins(byte stepPinNumber, byte directionPinNumber, byte limitPinNumber)
{
  SpeedyStepper::connectToPins(stepPinNumber, directionPinNumber);
  limitPin = limitPinNumber;
  
  pinMode(limitPin, INPUT_PULLUP);

}

//
// check iof limit switch is activated
//  Exit:  true returned if the stepper is at the target position
//
bool MultiHomeSpeedyStepper::limitSwitchActivated()
{
  return(digitalRead(limitPin) == LOW);
}



// -------------------------------------- End --------------------------------------
