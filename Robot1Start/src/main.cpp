#include <Arduino.h>
#include <Romi32U4.h>
#include "BlueMotor.h"
#include "LinearGripper.h"
#include <RemoteConstants.h>
#include <IRdecoder.h>
#include <Chassis.h>
#include <Rangefinder.h>

// Constants
int leftLineTrackerPin = A2; // A2 pin || 20
int rightLineTrackerPin = A3; // A3 pin || 21
int irRemotePin = 14; // pin 14
float ticksPerCM = 1440 * 1/(PI * 7.0); // 1440 ticks per rev / rev per PI * D
float cmPer90Deg = PI * 14/4; // PI * robotDiameter / 4th of the circle
float ticksPer90Deg = ticksPerCM * cmPer90Deg; // tick/cm * cm / 1 90deg = tick / 90deg

// Creating all controls
Chassis chassis;
BlueMotor motor;
LinearGripper servo;
Rangefinder rangefinder(17, 12);
IRDecoder decoder(irRemotePin);
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

bool armAuto45 = true; // True for 45 false for 25
bool autoStage = true; // True for pickup false for place
long previousTime = 0;
long currentTime = 0;
float baseDriveSpeed = 120;
float baseTurnSpeed = 80;



enum robotState{BEGIN, PICKUPSTART, PLACESTART, CROSSED_1, DRIVESHED, PICKUP, PLACE, BACKDRIVESHED, CROSSED_2, 
                DRIVEPLATFORM, PLATFORMPLACE, ENDPICKUP, PLACETURNTOLINE, DRIVEOTHERSIDE, CROSSED_3, STOP} currentState, nextState;

void setup()
{
  Serial.begin(9600);
  // Initialize and setup systems
  chassis.init();
  decoder.init();
  rangefinder.init();
  motor.setup();
  motor.reset();
  currentState = STOP;
  nextState = STOP;
  delay(3000);
}

/**
 * Converts centimeters to inches
 * @param cm centimeters to be converted
 * @return float inches from cm
 */
float cmToInches(float cm) {
  return cm / 2.54; // cm * 1 inch / 2.54 cm
}

/**
 * Follows the black tape using proportional control
 */
void lineTrack() {
  // Constants
  float kP = 0.5; // Proportional value
  float baseSpeed = 10; // default speed in cm/s

  float error = (analogRead(rightLineTrackerPin) - analogRead(leftLineTrackerPin))/100.; // Right line tracker value minus left and divided by 100 to scale the number down
  // float error = (analogRead(rightLineTrackerPin) - analogRead(leftLineTrackerPin)) / 100; // Right line tracker value minus left
  float effort = error*kP; // Effort at how much the robot wants to turn

  chassis.setWheelSpeeds(baseSpeed+effort, baseSpeed-effort); // Setting the wheel speeds to turn the robot towards the detected line by the effort
}

/**
 * Follows the black tape and goes to a distance both using proportional control
 * @param distance how far away the target should be (in inches)
 */
void lineTrackWithUltraSonic(float distance) {
  float currentDistance = cmToInches(rangefinder.getDistance()); // gets the current distance away

  if (currentDistance > 15) currentDistance = 15;

  // Constants
  float kP_L = 0.5; // LineTrack Proportional value
  float kP_D = 3; // Distance Proportional value // was 1.5

  float errorD = currentDistance - distance; // Current distance minus desired distance
  float errorL = (analogRead(rightLineTrackerPin) - analogRead(leftLineTrackerPin))/100.; // Right line tracker value minus left and divided by 100 to scale the number down

  float effortL = errorL*kP_L; // Effort at how much the robot wants to turn
  float effortD = errorD*kP_D; // Effort at how fast the wheels spin for distance

  // Setting the wheel speeds to turn the robot towards the detected line by the effort and the speed for how fast the robot approaches desired distance
  chassis.setWheelSpeeds(effortD+effortL, effortD-effortL); 
}

float previousEncoderCount = 0; // Previous encoder count used for knowing how far the robot has traveled

/**
 * Returns if the robot has traveled a certain distance
 * @param distance the distance you want the robot to go (in cm)
 * @return true if robot equals or has past the distance else false
 */
bool pastDistance(float distance) {
  // The encoder count traveled from when previously called
  float encoderCountTraveled = (chassis.getLeftEncoderCount(false) + chassis.getRightEncoderCount(false))/2 - previousEncoderCount;

  // Average encoder counts * Diameter of wheel * pi / ticks/rev of motor
  float distanceTraveled = encoderCountTraveled * 7 * 3.14  / 1440;
  return distance <= distanceTraveled;
}

/**
 * Returns whether the robot is within a certain distance using the Ultasonic Sensor
 * @param distance distance the robot is within (in inches)
 * @return true when distance is within tolerance else false
 */
bool withinDistance(float distance) {
  return abs(distance - cmToInches(rangefinder.getDistance())) < 0.25; // gets within 0.25 in
}

bool checkFinished = false; // Flag variable to see if movement is finished

/**
 * Drives the robot towards the shed
 */
void driveToShed() {
  bool checkBlackLine = analogRead(rightLineTrackerPin) > 600 && analogRead(leftLineTrackerPin) > 600; // Checks to see if robot has crossed the black perpendicular line

  // Line tracks when the robot hasn't crossed the black line
  if (!checkBlackLine) {
    lineTrack();
  } else {
    chassis.driveFor(8, 10, true); // Drives to get the turning center is over the crossing line
    chassis.turnFor(-45, 45, true); // Turn 45 degrees towards shed
    // Turns until line is detected
    while (analogRead(rightLineTrackerPin) < 400) {
        chassis.turnFor(-10, 180, true);
    }

    float distance = 5; // Desired distance away from the Shed in inches
    // Line tracks with range finder for the distance specified then sets checkFinished to be true
    while (!withinDistance(distance)) {
      lineTrackWithUltraSonic(distance);
    } 
    checkFinished = true;
  }

}

bool checkLineFound = false; // Flag for checking if the line has been found

/**
 * Turns the robot until it detects the line
 */
void initialDetectLine() {
  chassis.turnFor(-60, 90, true); // Initial turn not checking for line

  // Turns until line is detected
  while (analogRead(rightLineTrackerPin) < 400) {
    chassis.turnFor(-10, 180, true);
  }
  checkLineFound = true;
}

/**
 * Run servo until it should stop
 */
void runServo() {
  if (servo.getState() != servo.STOP) {
    servo.setGripperState(servo.getState());
  } else {
    servo.setGripperState(servo.STOP);
  }
}

bool check = true;
bool distanceFlag = true;
int turnDirection = 1;
bool checkTracker = false;

void setRobotState(robotState STATE) {

  float leftLineTrack = analogRead(leftLineTrackerPin);
  float rightLineTrack = analogRead(rightLineTrackerPin);

  float leftEncoderCount = chassis.getLeftEncoderCount();
  float rightEncoderCount = chassis.getRightEncoderCount();

  float currentEncoderCount = 0;

  float distance = 0;

  currentTime = millis();

  // Checks to see if robot has crossed the black perpendicular line
  bool checkBlackLine = rightLineTrack >= 600 && leftLineTrack >= 600; // was 600
  

  switch(STATE) {

    case BEGIN:
    

    break;

    case PICKUPSTART:

      servo.setGripperState(servo.OPEN);
      runServo();
      if (checkBlackLine) {
        chassis.idle();
        if (servo.getState() == servo.STOP) {
          servo.setGripperState(servo.STOP);
          currentState = STOP;
          nextState = CROSSED_1;
        }
      } else {
        lineTrack();
      }

    break;

    case CROSSED_1:

      if (armAuto45) {
        if (autoStage) {
          motor.setMotorState(motor.PLACE45);
        } else {
          motor.setMotorState(motor.PRE45);
        }
      } else {
        if (autoStage) {
          motor.setMotorState(motor.PLACE25);
        } else {
          motor.setMotorState(motor.PRE25);
        }
      }

      if (armAuto45) {
        checkTracker = leftLineTrack < 300;
        turnDirection = 1;
      } else {
        checkTracker = rightLineTrack < 300;
        turnDirection = -1;
      }

      if (currentTime - previousTime <= 500) {
        chassis.setMotorEfforts(baseDriveSpeed, baseDriveSpeed);
      } else if (checkTracker || currentTime - previousTime <= 1000) {
        chassis.setMotorEfforts(baseTurnSpeed * turnDirection, -baseTurnSpeed * turnDirection);
      } else if (motor.isFinished()) {
        currentState = STOP;
        nextState = DRIVESHED;
      } else {
        chassis.setMotorEfforts(0,0);
      }

    break;

    case DRIVESHED:

      // was 5.5
      distance = 6; // Desired distance away from the Shed in inches
      // Line tracks with range finder for the distance specified then sets checkFinished to be true
      if (!withinDistance(distance)) {
        lineTrackWithUltraSonic(distance);
      } else if (autoStage) {
          chassis.idle();
          motor.setMotorState(motor.STOP);
          currentState = STOP;
          nextState = PICKUP;
      } else {
          chassis.idle();
          motor.setMotorState(motor.STOP);
          currentState = STOP;
          nextState = PLACE;
      }

    break;

    case PICKUP:
    
      servo.setGripperState(servo.CLOSE);
      runServo();

      if (servo.getState() == servo.STOP) {
        if (armAuto45) {
          motor.setMotorState(motor.PRE45);
        } else {
          motor.setMotorState(motor.PRE25);
        }
      }

      if (motor.isFinished() && (motor.motorState == motor.PRE45 || motor.motorState == motor.PRE25)) {
        currentState = STOP;
        nextState = BACKDRIVESHED;
        check = true;
        distanceFlag = true;
        previousTime = currentTime;
      }


    break;

    case BACKDRIVESHED:
    
      distance = 8;
      if (distanceFlag && cmToInches(rangefinder.getDistance()) < distance) {
        chassis.setMotorEfforts(-baseDriveSpeed, -baseDriveSpeed);
        previousTime = currentTime;
      } else {
        distanceFlag = false;
        motor.setMotorState(motor.PREPLATFORM);

        if (armAuto45) {
          checkTracker = rightLineTrack < 300;
          turnDirection = 1;
        } else {
          checkTracker = leftLineTrack < 300;
          turnDirection = -1;
        }

        if (check && checkTracker || currentTime - previousTime <= 500) {
          chassis.setMotorEfforts(baseTurnSpeed * turnDirection, -baseTurnSpeed * turnDirection);
        } else if (checkBlackLine) {
          chassis.idle();
          if (motor.isFinished()) {
            currentState = STOP;
            nextState = CROSSED_2;
            previousTime = currentTime;
          }
        } else {
          check = false;
          lineTrack();
        }
      }

      

    break;

    case CROSSED_2:

    if (currentTime - previousTime <= 500) {
        chassis.setMotorEfforts(baseDriveSpeed, baseDriveSpeed);
      } else if (leftLineTrack < 300 || currentTime - previousTime <= 1000) {
        chassis.setMotorEfforts(-baseTurnSpeed * turnDirection, baseTurnSpeed * turnDirection);
      } else {
        currentState = STOP;
        nextState = DRIVEPLATFORM;
      }

    break;

    case DRIVEPLATFORM:


      distance = 4.7; // 5.5 / 4.5
      // Line tracks with range finder for the distance specified then sets checkFinished to be true
      if (cmToInches(rangefinder.getDistance()) > distance) {
        lineTrack();
      } else {
          chassis.idle();
          if (autoStage) {
            currentState = STOP;
            nextState = PLATFORMPLACE;
          } else {
            currentState = STOP;
            nextState = DRIVEOTHERSIDE;
            if (armAuto45) {
              previousEncoderCount = rightEncoderCount;
            } else {
              previousEncoderCount = leftEncoderCount;
            }
            check = true;
          }
          
      }    

    break;

    case PLATFORMPLACE:

      motor.setMotorState(motor.BOTTOM);

      if (motor.isFinished()) {
        servo.setGripperState(servo.OPEN);
        runServo();

        if (servo.getState() == servo.STOP) {
          currentState = STOP;
          nextState = ENDPICKUP;
        }
      }
  
    break;

    case ENDPICKUP:
    
      distance = 6;
      if (cmToInches(rangefinder.getDistance()) < distance) {
        chassis.setMotorEfforts(-50, -50);
      } else {
        currentState = STOP;
        nextState = PLACESTART;
        autoStage = false;
      }

    break;

    case PLACESTART:
    
      distance = 5;
      if (check && cmToInches(rangefinder.getDistance()) > distance) {
        lineTrack();
      } else {
        check = false;
        chassis.idle();

        servo.setGripperState(servo.CLOSE);
        runServo();

        if (servo.getState() == servo.STOP) {
          autoStage = false;
          check = true;
          currentState = STOP;
          nextState = PLACETURNTOLINE;
          previousTime = currentTime;
        }
      }

    break;

    case PLACETURNTOLINE:

      motor.setMotorState(motor.PREPLATFORM);
    
      if (check && (rightLineTrack < 300 || currentTime - previousTime <= 500)) {
        if (motor.isFinished())
          chassis.setMotorEfforts(baseTurnSpeed, -baseTurnSpeed);
      } else if (checkBlackLine) {
        chassis.idle();
        currentState = STOP;
        nextState = CROSSED_1;
        previousEncoderCount = (chassis.getLeftEncoderCount(false) + chassis.getRightEncoderCount(false))/2;
        previousTime = currentTime;
      } else {
        check = false;
        lineTrack();
      }

    break;

    case PLACE:

      // was 5.5
      distance = 5.5; // Desired distance away from the Shed in inches
      // Line tracks with range finder for the distance specified then sets checkFinished to be true
      if (!withinDistance(distance)) {
        lineTrackWithUltraSonic(distance);
      }

      if (armAuto45) {
        motor.setMotorState(motor.PLACE45);
      } else {
        motor.setMotorState(motor.PLACE25);
      }

      if (motor.isFinished()) {
        servo.setGripperState(servo.OPEN);
        runServo();
        if (servo.getState() == servo.STOP) {
          servo.setGripperState(servo.STOP);
          check = true;
          currentState = STOP;
          nextState = BACKDRIVESHED;
        }
      }

    break;

    case DRIVEOTHERSIDE:

      if (armAuto45) {
        turnDirection = 1;
        currentEncoderCount = rightEncoderCount;
      } else {
        turnDirection = -1;
        currentEncoderCount = leftEncoderCount;
      }

      if (check && currentEncoderCount - previousEncoderCount <= ticksPer90Deg) {
        chassis.setMotorEfforts(-baseTurnSpeed * turnDirection, baseTurnSpeed * turnDirection);
      } else if (!checkBlackLine) {
        chassis.setMotorEfforts(baseDriveSpeed+10, baseDriveSpeed);
      } else {
        currentState = STOP;
        nextState = CROSSED_3;
        previousTime = currentTime;
      }

    break;

    case CROSSED_3:

    if (currentTime - previousTime <= 500) {
        chassis.setMotorEfforts(baseDriveSpeed, baseDriveSpeed);
      } else if (leftLineTrack < 300 || currentTime - previousTime <= 1000) {
        chassis.setMotorEfforts(-baseTurnSpeed * turnDirection, baseTurnSpeed * turnDirection);
      } else {
        currentState = STOP;
      }

    break;

    case STOP:

      chassis.idle();
      servo.setGripperState(servo.STOP);
      motor.setMotorState(motor.STOP);

    break;

  }

}

/**
 * Handles a key press on the IR remote
 * @param keyPress keypress entered
 */
void handleKeyPress(int keyPress)
{
  // Pause
  if (keyPress == remotePlayPause)
  {
    Serial.println("Pause");
    nextState = currentState;
    currentState = STOP;
    setRobotState(currentState);
  }

  // Resume
  if (keyPress == remoteUp)
  {
    Serial.println("Resume");
    currentState = nextState;
    previousTime = currentTime;
    setRobotState(currentState);
  }

  // Start pickup 45
  if (keyPress == remoteLeft)
  {
    Serial.println("Pickup 45");
    currentState = PICKUPSTART;
    armAuto45 = true;
    autoStage = true;
    setRobotState(currentState);
  }

  // Start 25
  if (keyPress == remoteRight)
  {
    Serial.println("Pickup 25");
    currentState = PICKUPSTART;
    armAuto45 = false;
    autoStage = true;
    setRobotState(currentState);
  }
}

bool flag = false;

void loop()
{

  // Check for a key press on the remote
  int keyPress = decoder.getKeyCode(); //true allows the key to repeat if held down
  if(keyPress >= 0) handleKeyPress(keyPress);

  setRobotState(currentState);

  if (!flag && currentState == STOP) {
    Serial.println("STOP");
    flag = true;
  } else if (currentState != STOP) {
    flag = false;
  }

  // setRobotState(currentState);
  // lineTrack(true);


  
  /*
  // Place 45 degree plate
  if (buttonB.isPressed()) {
    motor.setMotorState(motor.PRE45); // Move fourbar to pre pos
    while (!motor.isFinished()) {} // Wait until movement is done
    motor.setMotorState(motor.PLACE45); // Move fourbar to placement position
    while (!motor.isFinished()) {} // Wait until movement is done
    motor.setMotorState(motor.STOP); // Stop fourbar
    servo.setGripperState(servo.OPEN); // Set gripper to open
    while (servo.getState() != servo.STOP) { // Keep opening until it should stop
      servo.setGripperState(servo.OPEN);
    }
  } 
  */

  /*
  // Place 25 degree plate
  if (buttonB.isPressed()) {
    motor.setMotorState(motor.PRE25); // Move fourbar to pre pos
    while (!motor.isFinished()) {} // Wait until movement is done
    motor.setMotorState(motor.PLACE25); // Move fourbar to placement position
    while (!motor.isFinished()) {} // Wait until movement is done
    motor.setMotorState(motor.STOP); // Stop fourbar
    servo.setGripperState(servo.OPEN); // Set gripper to open
    while (servo.getState() != servo.STOP) { // Keep opening until it should stop
      servo.setGripperState(servo.OPEN);
    }
  }
  */
  

  /*
  // Pickup 45 degree plate
  if (buttonB.isPressed()) {
    motor.setMotorState(motor.PLACE45); // Move fourbar to the place/pickup position
    while (!motor.isFinished()) {} // Wait until movement is done
    motor.setMotorState(motor.STOP);
    servo.setGripperState(servo.CLOSE); // Set gripper to close
    while (servo.getState() != servo.STOP) { // Keep opening until it should stop
      servo.setGripperState(servo.CLOSE);
    }
    motor.setMotorState(motor.PRE45); // Move fourbar to pre position
    while (!motor.isFinished()) {} // Wait until movement is done
    motor.setMotorState(motor.STOP); // Stop fourbar
  } 
  */


  /*
  // Pickup 25 degree plate
  if (buttonB.isPressed()) {
    motor.setMotorState(motor.PLACE25); // Move fourbar to the place/pickup position
    while (!motor.isFinished()) {} // Wait until movement is done
    motor.setMotorState(motor.STOP);
    servo.setGripperState(servo.CLOSE); // Set gripper to close
    while (servo.getState() != servo.STOP) { // Keep opening until it should stop
      servo.setGripperState(servo.CLOSE);
    }
    motor.setMotorState(motor.PRE25); // Move fourbar to pre position
    while (!motor.isFinished()) {} // Wait until movement is done
    motor.setMotorState(motor.STOP); // Stop fourbar
  } 
  */

  /*
  if (buttonA.isPressed()) {
    servo.toggleGripper(); // Toggles the gripper state
    motor.setMotorState(motor.BOTTOM); // Move fourbar to bottom
    servo.setGripperState(servo.getState()); // Set gripper to other state
  }

  // Sets the gripper to it's current state
  if (servo.getState() != servo.STOP) {
    servo.setGripperState(servo.getState());
  } else {
    servo.setGripperState(servo.STOP);
  }

  // Stop the fourbar if the motion has finished
  if (motor.isFinished()) {
    motor.setMotorState(motor.STOP);
  }
  */
  
}