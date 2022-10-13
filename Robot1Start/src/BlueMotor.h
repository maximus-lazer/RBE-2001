#pragma once

class BlueMotor
{
public:
    BlueMotor();
    void setEffortWithoutDB(float effort);
    void setEffort(int effort);
    void moveTo(long position);
    enum motorStateChoices{BOTTOM, PREPLATFORM, PRE45, PLACE45, PLACE25, PRE25, STOP} motorState, nextMotorState;
    void setMotorState(motorStateChoices STATE);
    void setMotorState(motorStateChoices STATE, bool blocking);
    bool isFinished();
    long getPosition();
    void reset();
    void setup();

private:
    void setEffort(int effort, bool clockwise);
    static void isr();
    static void isrA();
    static void isrB();
    const int tolerance = 20;
    const int PWMOutPin = 11;
    int motorTarget = 0;
    const int AIN2 = 4;
    const int AIN1 = 13;
    const static int ENCA = 0;
    const static int ENCB = 1;
};