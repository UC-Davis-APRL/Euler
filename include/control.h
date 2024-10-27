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
    int initialPosition1 = 138;
    int initialPosition2 = 160;
    Servo motor1;
    Servo motor2;
    int usMin = 1000;
    int usMax = 2000;

    // PID controller gains for servos
    float Kp_servo = 0.5;
    float Ki_servo = 0.0;
    float Kd_servo = 0.3;

    // PID controller variables for servos
    float integralPitch = 0;
    float integralRoll = 0;
    float prevErrorPitch = 0;
    float prevErrorRoll = 0;

    // Integral saturation limit for servos
    float integralLimit = 5.0;

    // PID controller gains for altitude
    float Kp_altitude = 30.0;   // Adjusted gain for better response
    float Ki_altitude = 0.5;    // Adjusted gain for integral action
    float Kd_altitude = 0.9;   // Added derivative gain

    // PID controller variables for altitude
    float integralAltitude = 0;
    float prevErrorAltitude = 0;

    // Altitude control variables
    float targetHeight = 0.7;
    int motorSpeed1 = 78;
    int motorSpeed2 = 108;
    bool altitude_control_on = false;
    int motorSpeedChangeLimit = 2; // Max change per control loop (integer)

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
    void setMotor1Speed(int speed);
    void setMotor2Speed(int speed);
    void armMotor(Servo &motor, int min, int max);
    void altitudeControl(bool on_off);

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
    delay(1000);
}

inline void Control::setMotor1Speed(int speed)
{
    if (motor_armed)
    {
        motor1.write(speed);
    }
}

inline void Control::setMotor2Speed(int speed)
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

inline void Control::altitudeControl(bool on_off)
{
    altitude_control_on = on_off;
    if (on_off)
    {
        // Initialize altitude PID variables
        integralAltitude = 0;
        prevErrorAltitude = 0;
        motorSpeed1 = 98;
        motorSpeed2 = 108;

        // Set initial motor speeds
        setMotor1Speed(motorSpeed1);
        setMotor2Speed(motorSpeed2);
    }
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

    if (altitude_control_on)
    {
        // Altitude PID control
        float errorAltitude = targetHeight - nav->height;

        // Update integral term with anti-windup
        integralAltitude += errorAltitude * deltaTime;
        integralAltitude = constrain(integralAltitude, 0, 100);

        // Compute derivative term
        float derivativeAltitude = (errorAltitude - prevErrorAltitude) / deltaTime;

        // Compute control output
        float controlAltitude = Kp_altitude * errorAltitude + Ki_altitude * integralAltitude + Kd_altitude * derivativeAltitude;

        // Update previous error
        prevErrorAltitude = errorAltitude;

        // Adjust motor speeds with rate limiting
        int desiredChange = (int)(controlAltitude);

        // Adjust motor speeds
        motorSpeed1 = 140 + desiredChange;
        motorSpeed2 = 140 + desiredChange;

        // Ensure motor speeds are within acceptable limits (e.g., 0 to 180)
        motorSpeed1 = constrain(motorSpeed1, 0, 170);
        motorSpeed2 = constrain(motorSpeed2, 0, 170);

        printf("Height: %lf, Error: %lf, Control Output: %lf, Motor Speeds: %d, %d \n", nav->height, errorAltitude, controlAltitude, motorSpeed1, motorSpeed2);

        // Set motor speeds
        setMotor1Speed(motorSpeed1);    
        setMotor2Speed(motorSpeed2);
    }

    if (lock_gimbal)
    {
        setServo1Angle(0);
        setServo2Angle(0);
    }
    else
    {
        // Get the pitch and roll from the nav
        float pitch = nav->pitch;
        float roll = nav->roll;

        // Compensate for the IMU being at a 45-degree angle (if necessary)
        // float adjustedPitch = pitch * cos(PI / 4) - roll * sin(PI / 4);
        // float adjustedRoll = pitch * sin(PI / 4) + roll * cos(PI / 4);
        float adjustedPitch = pitch;
        float adjustedRoll = roll;



        // Compute errors (assuming desired pitch and roll are zero)
        float errorPitch = adjustedPitch;
        float errorRoll = adjustedRoll;

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

        // Limit control outputs to ±20 degrees
        controlPitch = constrain(controlPitch, -30, 30);
        controlRoll = constrain(controlRoll, -30, 30);

        // Set the servo angles
        setServo1Angle(controlPitch);
        setServo2Angle(controlRoll);

        // Update previous errors
        prevErrorPitch = errorPitch;
        prevErrorRoll = errorRoll;
    }
}

#endif
