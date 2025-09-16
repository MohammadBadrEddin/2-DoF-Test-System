#ifndef PITCH_MOTOR_HPP
#define PITCH_MOTOR_HPP

#include <AccelStepper.h>
#include <AS5600.h>

class PitchMotor {
public:
    PitchMotor(int stepPin, int dirPin, float maxSpeed);
    void moveToAngle(float angle);
    void homing();
    float getAngle();
private:
    long angleToSteps(float angle);
    float stepsToAngle(long steps);
    AccelStepper motor;
    AS5600 sensor;
    float maxSpeed;
};

#endif
