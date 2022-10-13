#include <Arduino.h>
#include <LinearGripper.h>
#include <Romi32U4.h>
#include <servo32u4.h>
#include "Timer.h"


Servo32U4 jawServo;
int delay1 = 250; // 250 ms delay
int delay2 = 1000; // 1 second delay

Timer timer1(delay1); // timer with 250 ms delay
Timer timer2(delay2); // timer with 1 second delay

LinearGripper::LinearGripper()
{
  gripperState = STOP;
}

/**
 * Gets the current state of the gripper
 * @return The state of the gripper
 */
enum LinearGripper::gripperStateChoices LinearGripper::getState() {
  return gripperState;
}

/**
 * Set the gripper to a specific state
 * @param STATE the state the gripper is set to
 */
void LinearGripper::setGripperState(gripperStateChoices STATE) {

  bool checkStuck = false;
  linearPotVoltageADC = analogRead(linearPotPin); // Current value for the linear potentiometer
  switch(STATE) {

    case OPEN:

      gripperState = OPEN;
      nextGripperState = STOP;

      // Sets the gripper to move down if the linear potentiometer is greater than the open position
      if (linearPotVoltageADC > jawOpenPotVoltageADC) {
        jawServo.writeMicroseconds(servoJawDown);
      } else {
        // Sets the gripper to the stop state
        gripperState = STOP;
        nextGripperState = CLOSE;
      } 

    break;

    case CLOSE:

      // Checks to see if the gripper is stuck by seeing if the linear potentiometer has moved more than a tolerance and that a certain time has passed
      checkStuck = !timer1.isExpired() && timer2.isExpired() && (abs(linearPotVoltageADCOLD - linearPotVoltageADC) < linearPotVoltageADCTolerance);      
      checkStuck = false;
      
      gripperState = CLOSE;
      nextGripperState = STOP;

      // If stuck then set the gripper to be open
      if (checkStuck) {
        timer1.reset();
        timer2.reset();
        gripperState = OPEN;
        nextGripperState = OPEN;
      } 
      // Sets the gripper to move up if the linear potentiometer is less than the closed position
      else if (linearPotVoltageADC < jawClosedPotVoltageADC) {
        jawServo.writeMicroseconds(servoJawUp);
      }
      // Checks to see if gripper has finished closeing, if yes then stop
      else if (linearPotVoltageADC >= jawClosedPotVoltageADC) {
        gripperState = STOP;
        nextGripperState = OPEN;
      } 

      // Sets the old value of the linear potentiometer after a time has expired
      if (timer1.isExpired()){
        linearPotVoltageADCOLD = linearPotVoltageADC;
      }

    break;

    case STOP:

      // Reset both timers and stop the gripper
      timer1.reset();
      timer2.reset();
      gripperState = STOP;
      jawServo.writeMicroseconds(servoStop);

    break;
  }

}

/**
 * Toggle the gripper state to open if it's closed or closed if it's open
 */
void LinearGripper::toggleGripper() {
  if (nextGripperState == OPEN) {
    gripperState = OPEN;
  } else if (nextGripperState == CLOSE) {
    gripperState = CLOSE;
  }
}