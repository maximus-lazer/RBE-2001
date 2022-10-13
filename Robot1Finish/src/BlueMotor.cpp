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
long pre45Pos = -4296; // -5000
long place45Pos = -3774;
long pre25Pos = -6506; // -6506
long place25Pos = -6740; // -6740
long prePlatformPos = -1000;
long bottomPos = 0;

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
    count--; //++
  }
  else {
    count++; //--
  }
}
void BlueMotor::isrB()
{
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    count++; //--
  }
  else {
    count--; //++
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
    float kP = 4; // Proportional value was 5
    float error = (target - count); // Error of arm to setpoint
    float effort = kP*error; // Effort for how much the motor to move

    // if (effort < -32000) effort = -32000;
    // Moves the motor if the arm isn't within tolerance
    if (abs(error) > tolerance) {
        setEffort(effort);
    } else {
        setEffort(0);
    }
}

/**
 * Set the arm to move to a certain position based on state (non-blocking)
 * @param STATE the state for the arm to move to
 */
void BlueMotor::setMotorState(motorStateChoices STATE) {
  switch(STATE) {

    case PRE45:

      // Sets motor state and target
      motorState = PRE45;
      motorTarget = pre45Pos;

      // Moves motor to Pre45
      moveTo(motorTarget);

    break;

    case PLACE45:

      // Sets motor state and target
      motorState = PLACE45;
      motorTarget = place45Pos;

      // Moves motor to Place45
      moveTo(motorTarget);

    break;

    case PRE25:

      // Sets motor state and target
      motorState = PRE25;
      motorTarget = pre25Pos;

      // Moves motor to Pre25
      moveTo(motorTarget);

    break;

    case PLACE25:

      // Sets motor state and target
      motorState = PLACE25;
      motorTarget = place25Pos;

      // Moves motor to Place25
      moveTo(motorTarget);

    break;

    case PREPLATFORM:

      // Sets motor state and target
      motorState = PREPLATFORM;
      motorTarget = prePlatformPos;

       // Moves motor to prePlatform
      moveTo(motorTarget);

    break;

    case BOTTOM:

      // Sets motor state and target
      motorState = BOTTOM;
      motorTarget = bottomPos;

       // Moves motor to Bottom
      moveTo(motorTarget);

    break;

    case STOP:

      // Set state to STOP and stops moving the motor
      motorState = STOP;
      setEffort(0);

    break;
  }

}

/**
 * Returns if the motor is finished with it's motion
 * @return true if motor is within tolerance else false
 */
bool BlueMotor::isFinished() {
  return abs(getPosition() - motorTarget) <= tolerance;
}


