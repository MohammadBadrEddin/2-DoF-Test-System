#include "YawMotor.hpp"
#include <Arduino.h>

#define STEPS_PER_REV 3108
#define DEGREES_PER_REV 360.0

YawMotor::YawMotor(int stepPin, int dirPin, int hallPin, float maxSpeed)
: motor(AccelStepper::DRIVER, stepPin, dirPin), hallSensorPin(hallPin), maxSpeed(maxSpeed) {
    pinMode(hallSensorPin, INPUT);
    motor.setMaxSpeed(maxSpeed * 10);
    motor.setAcceleration(100);
}

long YawMotor::angleToSteps(float angle) {
    return (long)((angle / DEGREES_PER_REV) * STEPS_PER_REV);
}

float YawMotor::stepsToAngle(long steps) {
    return (steps / (float)STEPS_PER_REV) * DEGREES_PER_REV;
}

void YawMotor::moveToAngle(float angle) {
    motor.moveTo(angleToSteps(angle));
    while (motor.distanceToGo() != 0) motor.run();
}

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
