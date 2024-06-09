/*
    control.h

    Manages vehicle control
    WIP

    @authors Orfeas Magoulas
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
    unsigned long timestamp;
    Vehicle *vehicle;
    Nav *nav;
    Servo servo1;
    Servo servo2;
    int initialPosition1 = 80;
    int initialPosition2 = 110;
    Servo motor1;
    Servo motor2;
    int usMin = 1000;
    int usMax = 2000;

public:
    Control(Vehicle *vehicle, Nav *nav, Guidance *guidance) : vehicle(vehicle), nav(nav), timestamp(0)
    {
    }
    void init();
    void run();
    void setServo1Angle(int angle);
    void setServo2Angle(int angle);
    void setMotor1Speed(int speed);
    void setMotor2Speed(int speed);
    void armMotor(Servo motor, int min, int max);

    bool lock_gimbal = true;
    bool motor_armed = true;
};

inline void Control::init()
{
    // Attach the servos to the pins 36 and 37
    servo1.attach(36);
    servo2.attach(37);

    // Set initial positions
    servo1.write(initialPosition1);
    servo2.write(initialPosition2);

    // Attach motors to pins 13 and 14
    motor1.attach(13, usMin, usMax);
    motor2.attach(14, usMin, usMax);
    Control::armMotor(motor1, usMin, usMax);
    Control::armMotor(motor2, usMin, usMax);
}

inline void Control::armMotor(Servo motor, int min, int max)
{
    // motor.writeMicroseconds(min); // send low signal to arm the ESC
    // delay(2000);                  // wait for 2 seconds
    // motor.writeMicroseconds(max); // send high signal to complete arming
    // delay(2000);                  // wait for 2 seconds
    // motor.writeMicroseconds(min); // send low signal again
    // delay(2000);                  // wait for 2 seconds
    motor.writeMicroseconds(0);
    delay(1000);
    Serial.println("Motor Configured");
}

// range is 0-1024, it clamps internally
inline void Control::setMotor1Speed(int speed)
{
    if (motor_armed)
    {
        speed = constrain(speed, 0, 1024);
        motor1.writeMicroseconds(usMin + (int)(((float)speed / 1024.0) * (float)(usMax - usMin)));
    }
}

// range is 0-1024, it clamps internally
inline void Control::setMotor2Speed(int speed)
{
    if (motor_armed)
    {
        speed = constrain(speed, 0, 1024);
        motor2.writeMicroseconds(usMin + (int)(((float)speed / 1024.0) * (float)(usMax - usMin)));
    }
}
inline void Control::setServo1Angle(int angle)
{
    int relativeAngle = constrain(initialPosition1 + angle, 0, 180);
    servo1.write(relativeAngle);
}

inline void Control::setServo2Angle(int angle)
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
    timestamp = millis();

    if (motor_armed) {
        motor1.writeMicroseconds(2000);
        motor2.writeMicroseconds(2000);

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

        // Compensate for the IMU being at a 45-degree angle
        float adjustedPitch = pitch * cos(PI / 4) - roll * sin(PI / 4);
        float adjustedRoll = pitch * sin(PI / 4) + roll * cos(PI / 4);

        // Map to servo angles
        int servo1Angle = map(adjustedPitch, -45, 45, -30, 30);
        int servo2Angle = map(adjustedRoll, -45, 45, -30, 30);

        // Digital hardstop DO NOT DELETE else it blows up
        servo1Angle = constrain(servo1Angle, -30, 30);
        servo2Angle = constrain(servo2Angle, -30, 30);

        setServo1Angle(servo1Angle);
        setServo2Angle(servo2Angle);
    }
}

#endif
