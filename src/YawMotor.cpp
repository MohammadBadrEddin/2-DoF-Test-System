#include "YawMotor.hpp"
#include <Arduino.h>

#define STEPS_PER_REV 200 // Nema 17 stepper calc --> 360 degrees / 200 steps = 1.8 degrees
#define DEGREES_PER_REV 360.0

YawMotor::YawMotor(int stepPin, int dirPin, int hallPin, float maxSpeed)
: motor(AccelStepper::DRIVER, stepPin, dirPin), hallSensorPin(hallPin), maxSpeed(maxSpeed) {
    pinMode(hallSensorPin, INPUT);
    motor.setMaxSpeed(maxSpeed * 10);
    motor.setAcceleration(100);
}
// convert angle to steps 
long YawMotor::angleToSteps(float angle) {
    return (long)((angle / DEGREES_PER_REV) * STEPS_PER_REV);
}
// convert steps to angle
float YawMotor::stepsToAngle(long steps) {
    return (steps / (float)STEPS_PER_REV) * DEGREES_PER_REV;
}
// move to the target angle
void YawMotor::moveToAngle(float angle) {
    motor.moveTo(angleToSteps(angle));
    while (motor.distanceToGo() != 0) motor.run();
}
// homing
void YawMotor::homing() {
    Serial.println("Starting Yaw Homing...");
    motor.moveTo(angleToSteps(-100));
    while (motor.distanceToGo() != 0) motor.run();

    motor.setSpeed(100);
    motor.moveTo(angleToSteps(100));
    while (motor.distanceToGo() != 0) {
        motor.run();
        if (digitalRead(hallSensorPin) == LOW) {
            Serial.println("Sensor detected! Setting this position as 0°.");
            motor.stop();
            motor.setCurrentPosition(0);
            return;
        }
    }
    Serial.println("Error: Hall sensor not detected.");
}
