/*
    actuators.h

    Manages actuators such as servos and motors

    Authors: Orfeas Magoulas
*/

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include <Servo.h>

class Actuators {
    Servo servo1;
    Servo servo2;
    Servo motor1;
    Servo motor2;

    // Initial servo positions
    const int initialPosition1 = 160;
    const int initialPosition2 = 138;

    // ESC timing
    int usMin = 1000;
    int usMax = 2000;

    // Internal state variables
    int base_speed = 0;   // Base speed from 0 to 255
    float torque = 0.0;   // Torque value from -1.0 to 1.0

    bool debug = true;
    bool armed = false;

    void updateMotorSpeeds() {
        // Determine the maximum possible adjustment without exceeding speed limits
        const int max_adjustment = min(base_speed, 255 - base_speed);

        // Calculate the speed adjustment based on torque
        const int delta = (int)(torque * max_adjustment);

        // Calculate new motor speeds with delta
        const int motor1_speed = constrain(base_speed + delta, 0, 255);
        const int motor2_speed = constrain(base_speed - delta, 0, 255);

        // Apply the adjusted speeds to the motors
        setMotor1Speed(motor1_speed);
        setMotor2Speed(motor2_speed);

        // Debugging output
        if (debug) {
            Serial.print(F("[ACTUATORS] Updated Motor Speeds -> Motor1: "));
            Serial.print(motor1_speed);
            Serial.print(F(", Motor2: "));
            Serial.println(motor2_speed);
        }
    }

public:
    Actuators() = default;

    void init() {
        Serial.println(F("[ACTUATORS] Initializing..."));

        // Attach servos to their respective pins
        servo1.attach(36);
        servo2.attach(37);

        // Set initial servo positions
        servo1.write(initialPosition1);
        servo2.write(initialPosition2);

        // Attach ESCs to their respective pins
        motor1.attach(2, usMin, usMax);
        motor2.attach(3, usMin, usMax);

        Serial.println(F("[ACTUATORS] Initialization complete!"));
    }

    // Arm the motors by sending minimum throttle
    void arm() {
        Serial.println(F("[ACTUATORS] Arming motors..."));
        armed = true;
        motor1.writeMicroseconds(usMin);
        delay(1000);
        motor2.writeMicroseconds(usMin);
        delay(1000);
        Serial.println(F("[ACTUATORS] Motors armed!"));
    }

    // Disarm the motors by sending zero throttle
    void disarm() {
        Serial.println(F("[ACTUATORS] Disarming motors..."));
        motor1.write(0);
        motor2.write(0);
        armed = false;
        Serial.println(F("[ACTUATORS] Motors disarmed."));
    }

    // Set angle for Servo 1 with constraints
    void setPitchServoAngle(const float angle) {
        const int relativeAngle = constrain(initialPosition1 + (int)constrain(-angle, -30, 30), 0, 180);
        servo1.write(relativeAngle);
    }

    // Set angle for Servo 2 with constraints
    void setRollServoAngle(const float angle) {
        const int relativeAngle = constrain(initialPosition2 + (int)constrain(-angle, -30, 30), 0, 180);
        servo2.write(relativeAngle);
    }

    // Set speed for Motor 1
    void setMotor1Speed(const int speed) {
        if (armed) {
            motor1.write(constrain(speed, 0, 255));
        } else {
            motor1.write(0);
        }
    }

    // Set speed for Motor 2
    void setMotor2Speed(const int speed) {
        if (armed) {
            motor2.write(constrain(speed, 0, 255));
        } else {
            motor2.write(0);
        }
    }

    // Set the base thrust for both motors
    void setThrust(float thrust) {
        thrust = constrain(thrust, 0.0, 1.0);

        // Map thrust to speed (0-255)
        base_speed = static_cast<int>(thrust * 255.0);

        updateMotorSpeeds();

        if (debug) {
            Serial.print(F("[ACTUATORS] Thrust set to "));
            Serial.println(thrust);
        }
    }

    // Set torque for RCS by adjusting motor speeds oppositely
    void setTorque(float t) {
        torque = constrain(t, -1.0, 1.0);
        updateMotorSpeeds();

        if (debug) {
            Serial.print(F("[ACTUATORS] Torque RCS set to "));
            Serial.println(torque);
        }
    }
};

#endif // ACTUATORS_H
