#include <Arduino.h>
#include "YawMotor.hpp"
#include "PitchMotor.hpp"

#define BAUD_RATE 115200

YawMotor yaw(8, 9, 2, 12.0);   // stepPin, dirPin, hallSensorPin, maxSpeed
PitchMotor pitch(10, 11, 10.0); // stepPin, dirPin, maxSpeed

void setup() {
    Serial.begin(BAUD_RATE);
    Serial.println("System initialized! Type 'h' to home both axes or input angles 'yaw,pitch'.");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.equalsIgnoreCase("h")) {
            yaw.homing();
            pitch.homing();
        } else {
            int commaIndex = input.indexOf(',');
            if (commaIndex > 0) {
                int yawAngle = input.substring(0, commaIndex).toInt();
                int pitchAngle = input.substring(commaIndex + 1).toInt();
                if (yawAngle >= -100 && yawAngle <= 100 && pitchAngle >= -30 && pitchAngle <= 30) {
                    Serial.print("Moving Yaw: "); Serial.print(yawAngle);
                    Serial.print(" Pitch: "); Serial.println(pitchAngle);
                    yaw.moveToAngle(yawAngle);
                    pitch.moveToAngle(pitchAngle);
                } else {
                    Serial.println("Invalid input range!");
                }
            } else Serial.println("Invalid format! Use 'yaw,pitch'.");
        }
    }
}
