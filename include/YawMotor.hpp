#ifndef YAW_MOTOR_HPP
#define YAW_MOTOR_HPP

#include <AccelStepper.h>

class YawMotor {
public:
    YawMotor(int stepPin, int dirPin, int hallSensorPin, float maxSpeed);
    void moveToAngle(float angle);
    void homing();
private:
    long angleToSteps(float angle);
    float stepsToAngle(long steps);
    AccelStepper motor;
    int hallSensorPin;
    float maxSpeed;
};

#endif
