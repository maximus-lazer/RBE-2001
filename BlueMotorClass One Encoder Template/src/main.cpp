#include <Arduino.h>
#include <Romi32U4.h>
#include "BlueMotor.h"


BlueMotor motor;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
long timeToPrint = 0;
long prevTime = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 540; // Ticks per revolution (270 for 1 encoder, 540 for 2)
int motorEffort = 200; // 300 is 75, 200 is 50, 400 is 100

int pos1=0;
int pos2 = 1000;

void setup()
{
  Serial.begin(9600);
  motor.setup();
  motor.reset();
  delay(3000);
  Serial.print("Time (ms)");
  Serial.print("   ");
  Serial.print("Position");
  Serial.print("    ");
  Serial.println("speedInRPM");
  delay(3000);
  buttonA.waitForButton();
}


/**
 * Calculates ticks to RPM of the motor
 * @param ticks how many ticks the motor is at (in ticks/100ms)
 * @return how many revolutions per minute the motor is running at
 */
int ticksToRPM(long ticks) {
  return (ticks*600) / CPR;
}

int ticksPerSecondToRPM(long ticks) {
  return (ticks*60) / CPR;
}

void loop()
{
  if (buttonB.isPressed()) {
    motor.setGripperState(motor.PLACE25);
    // motor.moveTo(-5300);
  }
  if (buttonA.isPressed()) {
    // motor.setGripperState(motor.BOTTOM);
    motor.moveTo(0);
  }
  /*
  // Testing percentage
  timeToPrint = millis() + sampleTime;
  oldPosition = motor.getPosition();
  for (float i = 0; i <= 1; i += 0.05) {
    
    motor.setEffortWithoutDB(i);

    newPosition = motor.getPosition();
    speedInRPM = ticksPerSecondToRPM((newPosition-oldPosition)); 
    Serial.print(i);
    Serial.print("         ");
    Serial.print(newPosition);
    Serial.print("          ");
    Serial.println(speedInRPM);
    delay(1000);
    oldPosition = newPosition;
  }*/

  // Testing using PWM signal 0-400
  /*
  timeToPrint = millis() + sampleTime;
  oldPosition = motor.getPosition();
  for (int i = 0; i <= 400; i += 10) {
    
    motor.setEffort(-i);

    newPosition = motor.getPosition();
    speedInRPM = ticksPerSecondToRPM((newPosition-oldPosition)); 
    Serial.print(i);
    Serial.print("         ");
    Serial.print(newPosition);
    Serial.print("          ");
    Serial.println(speedInRPM);
    delay(1000);
    oldPosition = newPosition;
  }
  */

  
  
  /*
  // Run the motor forwards
  timeToPrint = millis() + sampleTime;
  oldPosition = motor.getPosition();
  while (buttonB.isPressed())
  {
    // The button is currently pressed.
    motor.setEffort(motorEffort);
    if ((now = millis()) > timeToPrint)
    {
      timeToPrint = now + sampleTime;
      newPosition = motor.getPosition();
      speedInRPM = ticksToRPM(newPosition-oldPosition); 
      Serial.print(now);
      Serial.print("          ");
      Serial.print(newPosition);
      Serial.print("          ");
      Serial.println(speedInRPM);
      oldPosition = newPosition;
      
    }
    
  }
  
  // move the motor backwards
  while (buttonA.isPressed())
  {
    // The button is currently pressed.
    motor.setEffort(-motorEffort);
    if ((now = millis()) > timeToPrint)
    {
      timeToPrint = now + sampleTime;
      newPosition = motor.getPosition();
      speedInRPM = ticksToRPM(newPosition-oldPosition); 
      Serial.print(now);
      Serial.print("          ");
      Serial.print(newPosition);
      Serial.print("          ");
      Serial.println(speedInRPM);
      oldPosition = newPosition;
      
    }
    
  }*/

  
 /*
  // Moves the motor using proportional control to set position
  while (buttonA.isPressed()) {
    motor.moveTo(pos2);
    if ((now = millis()) > timeToPrint)
    {
      timeToPrint = now + sampleTime;
      newPosition = motor.getPosition();
      speedInRPM = ticksToRPM(newPosition-oldPosition); 
      Serial.print(now);
      Serial.print("          ");
      Serial.print(newPosition);
      Serial.print("          ");
      Serial.println(speedInRPM);
      oldPosition = newPosition;
      
    }
  } */
  
  
  motor.setEffort(0);
}