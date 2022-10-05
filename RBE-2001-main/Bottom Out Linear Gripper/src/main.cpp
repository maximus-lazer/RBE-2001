#include <Arduino.h>
#include <Romi32U4.h>
#include "Timer.h"
#include <servo32u4.h>
Romi32U4ButtonB buttonB;

/*
The FS90R continuous rotation servo converts 
standard RC servo position pulses into continuous
rotation speed. The default rest point is 1.5 ms, but
this can be adjusted by using a small screwdriver
to turn the middle-point adjustment potentiometer.
Pulse widths above the rest point result in counterclockwise 
rotation, with speed increasing as the pulse width increases;
pulse widths below the rest point result in clockwise rotation, 
with speed increasing as the pulse width decreases.
*/

int servoPin = 5;
int linearPotPin = 18;
int servoStop = 1850;  
int servoJawDown = 1000;
int servoJawUp = 2000;
int printDelay = 500;
int linearPotVoltageADCOLD = 0;
int linearPotVoltageADC = 1000;
int linearPotVoltageADCTolerance = 3;
int jawOpenPotVoltageADC = 650; // was 600
int jawClosedPotVoltageADC = 1023; // was 940
Romi32U4ButtonA buttonA;

enum gripperStateChoices{OPEN, CLOSE, STOP} gripperState, nextGripperState;


Servo32U4 jawServo;

Timer printTimer(printDelay);
//timenow - time increases
//oldtime - last time it checked



void openGripper() {
  jawServo.writeMicroseconds(servoJawDown);
  Serial.print("linearPotVoltageADCJawDOWN:     ");
       Serial.println(linearPotVoltageADC);
  linearPotVoltageADC = analogRead(linearPotPin);

  while (linearPotVoltageADC > jawOpenPotVoltageADC) {
         Serial.print("linearPotVoltageADCJawDOWNINSIDE:     ");
      Serial.println(linearPotVoltageADC);
      linearPotVoltageADC = analogRead(linearPotPin);
    if(linearPotVoltageADC == linearPotVoltageADCOLD) {
      linearPotVoltageADCOLD = analogRead(linearPotPin);
 
    }

  }
}

void closeGripper() {
 jawServo.writeMicroseconds(servoJawUp);
  Serial.print("linearPotVoltageADCJawUP:     ");
       Serial.println(linearPotVoltageADC);
  linearPotVoltageADC = analogRead(linearPotPin);

  while (linearPotVoltageADC < jawClosedPotVoltageADC) {
     Serial.print("linearPotVoltageADCJawUPINSIDE:     ");
       Serial.println(linearPotVoltageADC);
    linearPotVoltageADC = analogRead(linearPotPin);
  }
}

void setGripperState(gripperStateChoices STATE) {

  linearPotVoltageADC = analogRead(linearPotPin);
  switch(STATE) {

    case OPEN:

      if (linearPotVoltageADC > jawOpenPotVoltageADC) {
        jawServo.writeMicroseconds(servoJawDown);
      } else {
        gripperState = STOP;
        nextGripperState = CLOSE;
        setGripperState(gripperState);
      } 

    break;

    case CLOSE:

      // bool checkStuck = (abs(linearPotVoltageADCOLD - linearPotVoltageADC) < linearPotVoltageADCTolerance);
      Serial.println(analogRead(linearPotPin));

      if (linearPotVoltageADC < jawClosedPotVoltageADC) {
        jawServo.writeMicroseconds(servoJawUp);
      }
      // Checks to see if gripper has finished closeing, if yes then stop
      else if (linearPotVoltageADC >= jawClosedPotVoltageADC) {
        gripperState = STOP;
        nextGripperState = OPEN;
        setGripperState(gripperState);

      // Checks to see if gripper is jammed, if it is then stop the gripper
      } else if (abs(linearPotVoltageADCOLD - linearPotVoltageADC) < linearPotVoltageADCTolerance) {
        gripperState = STOP;
        nextGripperState = CLOSE;
        
        // setGripperState(gripperState);
      }

      if (printTimer.isExpired()){
        linearPotVoltageADCOLD = linearPotVoltageADC;
      }
      // linearPotVoltageADCOLD = linearPotVoltageADC; // Sets old linPotVoltage to previous

    break;

    case STOP:

      jawServo.writeMicroseconds(servoStop);

    break;
  }

}

void setup()
{
  Serial.begin(9600);
  jawServo.attach();
  setGripperState(STOP);
}

void loop()
{

  Serial.println(analogRead(linearPotPin));
  if (buttonB.isPressed()) {
    setGripperState(OPEN);
  } else if (buttonA.isPressed()) {
    setGripperState(CLOSE);
  } else {
    setGripperState(STOP);
  }

}
