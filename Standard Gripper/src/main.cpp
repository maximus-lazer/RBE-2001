#include <Romi32U4.h>
#include <servo32u4.h>
#include "Timer.h"

Servo32U4 servo;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;

int openPos = 230; // <
int closePos = 390; // >
int closeWrite = 2000;
int openWrite = 1000;
int currentPosition = 1000;
int positionTolerance = 20;
Timer timer(1000);

enum gripperStateChoices{OPEN, CLOSE} gripperState, nextGripperState;

void setup()
{
  servo.setMinMaxMicroseconds(400, 2500);
  gripperState = OPEN;
  buttonB.waitForButton();
}

/**
 * Set the gripper to a specific state
 * @param STATE the state the gripper is set to
 */
void setGripperState(gripperStateChoices STATE) {

  currentPosition = analogRead(A0); // Current value of the servo position
  switch(STATE) {

    case OPEN:

      // Sets the gripper state to open and writes it to the open position
      gripperState = OPEN;
      nextGripperState = CLOSE;
      servo.writeMicroseconds(openWrite);

    break;

    case CLOSE:

      // Sets the gripper state to closed and writes it to the closed position
      timer.reset();
      gripperState = CLOSE;
      nextGripperState = OPEN;
      servo.writeMicroseconds(closeWrite);

    break;
  }

}

/**
 * Checks to see if the gripper is stuck and sets it to be open if it is
 */
void checkStuck() {
  currentPosition = analogRead(A0);
  if (currentPosition < closePos && timer.isExpired()) {
    setGripperState(OPEN);
  }
}

void loop()
{

  // Sets the gripper to the open position
  if (buttonB.isPressed()) {
    setGripperState(OPEN);
  }

  // Sets the gripper to the closed position
  if (buttonA.isPressed()) {
    setGripperState(CLOSE);
  }

  // Checks stuck when girpper is in closed state
  if (gripperState == CLOSE) {
    checkStuck();
  }
  
  
}
