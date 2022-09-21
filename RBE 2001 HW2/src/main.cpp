#include <Romi32U4.h>
#include <Chassis.h>

Chassis chassis;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Starts up the romi
  chassis.init(); // Initialize chassis

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



void loop() {
  // put your main code here, to run repeatedly:

  // Drive in a pentagon shape with 20cm long sides
  drivePolygon(5, 20);
  delay(1000);
  
}