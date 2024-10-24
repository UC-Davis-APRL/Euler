/*
    control.h

    Manages vehicle control
    WIP

    @authors 
*/

#ifndef CONTROL_H
#define CONTROL_H

#include <vehicle.h>
#include <nav.h>
#include <guidance.h>
#include <Servo.h>

#define CONTROL_UPDATE_RATE_HZ 10

class Control
{
private:
    Vehicle *vehicle;
    Nav *nav;
    Guidance *guidance;
    Servo servo1;
    Servo servo2;
    int initialPosition1 = 28;
    int initialPosition2 = 150;
    Servo motor1;
    Servo motor2;
    int usMin = 1000;
    int usMax = 2000;

    // PID controller gains
    float Kp_servo = 2.0;
    float Ki_servo = 0.0;
    float Kd_servo = 0.0;

    // PID controller variables
    float integralPitch = 0;
    float integralRoll = 0;
    float prevErrorPitch = 0;
    float prevErrorRoll = 0;

    // Integral saturation limit
    float integralLimit = 5.0;

    // Timing variables
    unsigned long timestamp;
    unsigned long lastTime;

public:
    Control(Vehicle *vehicle, Nav *nav, Guidance *guidance)
        : vehicle(vehicle), nav(nav), guidance(guidance), timestamp(0), lastTime(0)
    {
    }
    void init();
    void run();
    void arm();
    void setServo1Angle(float angle);
    void setServo2Angle(float angle);
    void setMotor1SpeedTest(int speed);
    void setMotor2SpeedTest(int speed);
    void setMotor1Speed(float speed);
    void setMotor2Speed(float speed);
    void armMotor(Servo &motor, int min, int max);

    bool lock_gimbal = false;
    bool motor_armed = false;
};

inline void Control::init()
{
    Serial.println(F("[CONTROL] Initializing..."));
    // Attach the servos to the pins 36 and 37
    servo1.attach(36);
    servo2.attach(37);

    // Set initial positions
    servo1.write(initialPosition1);
    servo2.write(initialPosition2);

    // Attach motors to pins 2 and 3
    motor1.attach(2, usMin, usMax);
    motor2.attach(3, usMin, usMax);

    Serial.println(F("[CONTROL] Initialization complete!"));
}

inline void Control::arm()
{
    Serial.println(F("[CONTROL] Arming motors..."));
    Control::armMotor(motor1, usMin, usMax);
    Control::armMotor(motor2, usMin, usMax);
    motor_armed = true;
    Serial.println(F("[CONTROL] Motors armed"));
}

inline void Control::armMotor(Servo &motor, int min, int max)
{
    motor.writeMicroseconds(min); // send low signal to arm the ESC
    delay(1000);                  // wait for 1 second
    Serial.println("[CONTROL] Motor armed");
}

inline void Control::setMotor1SpeedTest(int speed)
{
    if (motor_armed)
    {
        motor1.write(speed);
    }
}

inline void Control::setMotor2SpeedTest(int speed)
{
    if (motor_armed)
    {
        motor2.write(speed);
    }
}

inline void Control::setServo1Angle(float angle)
{
    int relativeAngle = constrain(initialPosition1 + angle, 0, 180);
    servo1.write(relativeAngle);
}

inline void Control::setServo2Angle(float angle)
{
    int relativeAngle = constrain(initialPosition2 + angle, 0, 180);
    servo2.write(relativeAngle);
}

inline void Control::run()
{
    if ((millis() - timestamp) < (1000 / CONTROL_UPDATE_RATE_HZ))
    {
        return;
    }
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - timestamp) / 1000.0f; // Convert milliseconds to seconds
    if (deltaTime <= 0)
    {
        deltaTime = 0.001f; // Prevent division by zero
    }
    timestamp = currentTime;

    if (lock_gimbal)
    {
        setServo1Angle(0);
        setServo2Angle(0);
    }
    else
    {
        // Get the pitch and roll from the nav
        float pitch = nav->roll;
        float roll = nav->pitch;

        printf("Pitch %lf, Roll: %lf\n", pitch, roll);

        // Compensate for the IMU being at a 45-degree angle (if necessary)
        float adjustedPitch = pitch * cos(PI / 4) - roll * sin(PI / 4);
        float adjustedRoll = pitch * sin(PI / 4) + roll * cos(PI / 4);

        // Compute errors (assuming desired pitch and roll are zero)
        float errorPitch = adjustedPitch;
        float errorRoll = adjustedRoll;

        printf("Error pitch %lf, Error roll: %lf\n", errorPitch, errorRoll);

        // Compute integral terms with saturation
        integralPitch += errorPitch * deltaTime;
        integralRoll += errorRoll * deltaTime;

        // Apply integral saturation limits
        integralPitch = constrain(integralPitch, -integralLimit, integralLimit);
        integralRoll = constrain(integralRoll, -integralLimit, integralLimit);

        // Compute derivative terms
        float derivativePitch = (errorPitch - prevErrorPitch) / deltaTime;
        float derivativeRoll = (errorRoll - prevErrorRoll) / deltaTime;

        // Compute control outputs using the PID controller
        float controlPitch = Kp_servo * errorPitch + Ki_servo * integralPitch + Kd_servo * derivativePitch;
        float controlRoll = Kp_servo * errorRoll + Ki_servo * integralRoll + Kd_servo * derivativeRoll;

        // Limit control outputs to ±25 degrees
        controlPitch = constrain(controlPitch, -25, 25);
        controlRoll = constrain(controlRoll, -25, 25);

        // Set the servo angles
        setServo1Angle(controlPitch);
        setServo2Angle(controlRoll);

        // Update previous errors
        prevErrorPitch = errorPitch;
        prevErrorRoll = errorRoll;
    }
}

#endif
