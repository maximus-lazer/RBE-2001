#include <Arduino.h>
#include <Romi32U4.h>
#include "BlueMotor.h"


BlueMotor motor;
Romi32U4ButtonB buttonB;
long timeToPrint = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 270; // Ticks per revolution
int motorEffort = 400; // 300 is 75, 200 is 50, 400 is 100

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
}


/**
 * Calculates ticks to RPM of the motor
 * @param ticks how many ticks the motor is at (in ticks/100ms)
 * @return how many revolutions per minute the motor is running at
 */
int ticksToRPM(long ticks) {
  return (ticks*600) / CPR;
}

void loop()
{
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
    
  motor.setEffort(0);
}