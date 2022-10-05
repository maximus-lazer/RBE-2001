#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>
#include <math.h>

long oldValue = 0;
long newValue;
long count = 0;
unsigned time = 0;
int lowerBound = 110;
int upperBound = 400;
enum motorStateChoices{BOTTOM, TOP, PLACE45, PLACE25, PRE25, STOP} motorState, nextMotorState;

BlueMotor::BlueMotor()
{
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE);
    reset();
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}

void BlueMotor::isrA()
{
  if (digitalRead(ENCB) == digitalRead(ENCA)) {
    count++;
  }
  else {
    count--;
  }
}
void BlueMotor::isrB()
{
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    count--;
  }
  else {
    count++;
  }
}

void BlueMotor::isr()
{
    count++;
}

/**
 * Setting the effort without the deadband using percentage
 * @param percent percentage for the motor to run at [-1, 1]
 */
void BlueMotor::setEffortWithoutDB(float percent)
{
  float effort = percent*(upperBound-lowerBound)+lowerBound;
  setEffort(effort);

}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

void BlueMotor::moveTo(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                     //then stop
    float kP = 1;
    float error = -(count - target);
    float effort = kP*error;
    if (abs(error) > 3) {
        setEffort(effort);
    } else setEffort(0);
}

void setGripperState(motorStateChoices STATE) {
  switch(STATE) {

    case TOP:

      

    break;

    case PLACE45:

      

    break;

    case PRE25:

      

    break;

    case PLACE25:

      

    break;

    case BOTTOM:

      

    break;

    case STOP:

      

    break;
  }

}