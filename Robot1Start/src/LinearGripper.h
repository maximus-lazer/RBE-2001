#pragma once

class LinearGripper
{
public:
    LinearGripper();
    enum gripperStateChoices{OPEN, CLOSE, STOP} gripperState, nextGripperState;
    void setGripperState(gripperStateChoices STATE);
    void setGripperState(gripperStateChoices STATE, bool blocking);
    void toggleGripper();
    enum gripperStateChoices getState();
    long getPosition();

private:
    const int linearPotPin = 18;
    const int servoStop = 1850;  
    const int servoJawDown = 1000;
    const int servoJawUp = 2000;
    const int printDelay = 500;
    int linearPotVoltageADCOLD = 0;
    int linearPotVoltageADC = 1000;
    const int linearPotVoltageADCTolerance = 4;
    const int jawOpenPotVoltageADC = 350; // was 800
    const int jawClosedPotVoltageADC = 1023; // was 940
};