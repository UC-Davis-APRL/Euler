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
private:
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

    // Private method to update motor speeds based on base_speed and torque
    void updateMotorSpeeds() {
        // Determine the maximum possible adjustment without exceeding speed limits
        int max_adjustment = min(base_speed, 255 - base_speed);

        // Calculate the speed adjustment based on torque
        int delta = (int)(torque * max_adjustment);
        // delta = 0;
        // Calculate new motor speeds with delta
        int motor1_speed = constrain(base_speed + delta, 0, 255);
        int motor2_speed = constrain(base_speed - delta, 0, 255);

        // Apply the adjusted speeds to the motors
        setMotor1Speed(motor1_speed, true);
        setMotor2Speed(motor2_speed, true);

        // Debugging output
        Serial.print(F("[ACTUATORS] Updated Motor Speeds -> Motor1: "));
        Serial.print(motor1_speed);
        Serial.print(F(", Motor2: "));
        Serial.println(motor2_speed);
    }

public:
    Actuators() {}

    // Initialize all actuators
    void init() {
        Serial.println(F("[ACTUATORS] Initializing..."));

        // Attach servos to their respective pins
        servo1.attach(36);
        servo2.attach(37);

        // Set initial servo positions
        servo1.write(initialPosition1);
        servo2.write(initialPosition2);

        // Attach motors (ESCs) to their respective pins
        motor1.attach(2, usMin, usMax);
        motor2.attach(3, usMin, usMax);

        Serial.println(F("[ACTUATORS] Initialization complete!"));
    }

    void armMotor(Servo &motor, int min, int max) {
        motor.writeMicroseconds(min);
        delay(1000);
    }

    // Arm the motors by sending minimum throttle
    void arm() {
        Serial.println(F("[ACTUATORS] Arming motors..."));
        armMotor(motor1, usMin, usMax);
        armMotor(motor2, usMin, usMax);
        Serial.println(F("[ACTUATORS] Motors armed!"));
    }

    // Disarm the motors by sending zero throttle
    void disarm() {
        Serial.println(F("[ACTUATORS] Disarming motors..."));
        motor1.write(0);
        motor2.write(0);
        Serial.println(F("[ACTUATORS] Motors disarmed."));
    }

    // Set angle for Servo 1 with constraints
    void setPitchServoAngle(float angle) {
        int relativeAngle = constrain(initialPosition1 + (int)constrain(-angle, -30, 30), 0, 180);
        servo1.write(relativeAngle);
    }

    // Set angle for Servo 2 with constraints
    void setRollServoAngle(float angle) {
        int relativeAngle = constrain(initialPosition2 + (int)constrain(-angle, -30, 30), 0, 180);
        servo2.write(relativeAngle);
    }

    // Set speed for Motor 1 with safety check
    void setMotor1Speed(int speed, bool armed) {
        if (armed) {
            motor1.write(constrain(speed, 0, 255));
        } else {
            motor1.write(0);
        }
    }

    // Set speed for Motor 2 with safety check
    void setMotor2Speed(int speed, bool armed) {
        if (armed) {
            motor2.write(constrain(speed, 0, 255));
        } else {
            motor2.write(0);
        }
    }

    // Set the base thrust for both motors
    void setThrust(float thrust) {
        // Constrain thrust to [0.0, 1.0]
        thrust = constrain(thrust, 0.0, 1.0);

        // Map thrust to speed (0-255)
        base_speed = (int)(thrust * 255.0);

        // Update motor speeds with the new base_speed
        updateMotorSpeeds();

        // Debugging output
        Serial.print(F("[ACTUATORS] Thrust set to "));
        Serial.println(thrust);
    }

    // Set torque for RCS by adjusting motor speeds oppositely
    void setTorque(float t) {
        // Constrain torque to [-1.0, 1.0]
        torque = constrain(t, -1.0, 1.0);

        // Update motor speeds with the new torque value
        updateMotorSpeeds();

        // Debugging output
        Serial.print(F("[ACTUATORS] Torque RCS set to "));
        Serial.println(torque);
    }
};

#endif // ACTUATORS_H
