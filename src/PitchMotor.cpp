#include "PitchMotor.hpp"
#include <Arduino.h>
#include <Wire.h>

#define STEPS_PER_REV 3108
#define DEGREES_PER_REV 360.0
#define AS5600_RAW_TO_DEGREES (360.0 / 4096.0)

PitchMotor::PitchMotor(int stepPin, int dirPin, float maxSpeed)
: motor(AccelStepper::DRIVER, stepPin, dirPin), maxSpeed(maxSpeed) {
    motor.setMaxSpeed(maxSpeed * 10);
    motor.setAcceleration(100);
    Wire.begin();
    if (!sensor.begin()) {
        Serial.println("AS5600 sensor not detected! Check connections.");
        while(1);
    }
}

long PitchMotor::angleToSteps(float angle) {
    return (long)((angle / DEGREES_PER_REV) * STEPS_PER_REV);
}

float PitchMotor::stepsToAngle(long steps) {
    return (steps / (float)STEPS_PER_REV) * DEGREES_PER_REV;
}

float PitchMotor::getAngle() {
    uint16_t rawAngle = sensor.readAngle();
    return rawAngle * AS5600_RAW_TO_DEGREES;
}

void PitchMotor::moveToAngle(float targetAngle) {
    motor.moveTo(angleToSteps(targetAngle));
    while (motor.distanceToGo() != 0) motor.run();
}

void PitchMotor::homing() {
    Serial.println("Starting Pitch Homing...");
    float currentAngle = getAngle();
    if (currentAngle > 80) motor.moveTo(angleToSteps(30));
    else motor.moveTo(angleToSteps(-30));

    while(true) {
        motor.run();
        float newAngle = getAngle();
        if (abs(newAngle - 80) < 1.0) {
            motor.stop();
            motor.setCurrentPosition(0);
            Serial.println("Pitch homing complete. Motor set to 0°.");
            break;
        }
    }
}
