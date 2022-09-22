#include <Romi32U4.h>
#include <Chassis.h>


Chassis chassis;

// Creating 
int leftLineTrackerPin = A4; // A4 pin || 22
int rightLineTrackerPin = A3; // A3 pin || 21

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Starts up the romi
  chassis.init(); // Initialize chassis

}

/**
 * Follows the black tape using proportional control
 */
void lineTrack() {
  // Constants
  float kP = 0.5; // Proportional value
  float baseSpeed = 10; // default speed in cm/s

  float error = (analogRead(rightLineTrackerPin) - analogRead(leftLineTrackerPin))/100.; // Right line tracker value minus left and divided by 100 to scale the number down
  float effort = error*kP; // Effort at how much the robot wants to turn

  chassis.setWheelSpeeds(baseSpeed+effort, baseSpeed-effort); // Setting the wheel speeds to turn the robot towards the detected line by the effort
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
    chassis.turnFor(-45, 45, true); // Turn 90 degrees towards shed
    // Turns until line is detected
    while (analogRead(rightLineTrackerPin) < 400) {
        chassis.turnFor(-10, 180, true);
    }

    // sets previous count to be the average of the left and right encoders
    previousEncoderCount = (chassis.getLeftEncoderCount(false) + chassis.getRightEncoderCount(false))/2;

    // Line tracks for the distance specified then sets checkFinished to be true
    while (!pastDistance(20)) {
      lineTrack();
    } 
    checkFinished = true;
  }

}

/**
* Drives the robot to a set polygon
* @param sides number of sides of the polygon
* @param length how long each side length should be (in centemeters)
*/
void drivePolygon(float sides, float length) {
  // If sides is less than the min amount for a polygon then set it to 3
  if (sides < 3) sides = 3;

  // Calculates how many degrees each turn should be based on number of sides
  float degrees = 180 - ((sides-2)*180)/sides;
  
  // Loop by how many sides there are
  for (int i = 0; i < sides; i++) {
    chassis.driveFor(length, 10, true); // Drive straight for how long the length is
    delay(500);
    chassis.turnFor(degrees, 25, true);// Turn by calculated degrees
    delay(500);
  }
}

/**
 * Prints the left and right Line Track values
 */
void printLineTrack() {
  Serial.print(analogRead(leftLineTrackerPin));
  Serial.print("      ");
  Serial.println(analogRead(rightLineTrackerPin));
  delay(100);
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

void loop() {
  // put your main code here, to run repeatedly:

  
  // Stops loop when auto is finished
  if (!checkFinished) {

    if (!checkLineFound) initialDetectLine(); // Drives until robot detects the line

    driveToShed(); // Drives towards shed
  } else
    chassis.idle(); // Stops robot when auto is finished

}