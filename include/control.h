/*
    control.h

    Manages vehicle control

    @authors Orfeas Magoulas
*/

#ifndef CONTROL_H
#define CONTROL_H

#include <vehicle.h>
#include <nav.h>
#include <guidance.h>
#include <Servo.h>


#define ALTITUDE_CONTROL_RATE_HZ 20
#define ATTITUDE_CONTROL_RATE_HZ 30

class Control {
private:
    Vehicle *vehicle;
    Nav *nav;
    Guidance *guidance;

    // Servo and Motor Objects
    Servo servo1;
    Servo servo2;
    Servo motor1;
    Servo motor2;

    // Initial servo positions (zeros)
    int initialPosition1 = 160;
    int initialPosition2 = 138;

    // Initial motor speeds (minimized z torque)
    int motorSpeed1 = 0;
    int motorSpeed2 = 0;

    // ESC timing
    int usMin = 1000;
    int usMax = 2000;

    // Runtime control variables (don't touch) 
    bool motor_armed = false;
    bool attitudeControl = false;
    bool altitudeControl = false;

    /*
        Attitude Control
    */
    // float Kp_servo = 0.12;
    // float Ki_servo = 0.016;
    // float Kd_servo = 0.24;
    float Kp_servo = 3;
    float Ki_servo = 0;
    float Kd_servo = 0;
    float integralLimit = 5.0;

    // Runtime variables (don't touch)
    float integralPitch = 0;
    float integralRoll = 0;
    float prevErrorPitch = 0;
    float prevErrorRoll = 0;
    unsigned long attitudeTimestamp = 0;

    /*
        Altitude Control
    */    
    float Kp_altitude = 40.0;
    float Ki_altitude = 10.0;
    float Kd_altitude = 0.0;
    float targetHeight = 0.6;

    // Runtime variables (don't touch)
    float integralAltitude = 0;
    float prevErrorAltitude = 0;
    unsigned long altitudeTimestamp = 0;

    void runAltitudeController();
    void runAttitudeController();
    void setServo1Angle(float angle);
    void setServo2Angle(float angle);
    void armMotor(Servo &motor, int min, int max);
public:
    Control(Vehicle *vehicle, Nav *nav, Guidance *guidance): vehicle(vehicle), nav(nav), guidance(guidance) {}
    void init();
    void run();
    void arm(bool on_off);
    void setMotor1Speed(int speed);
    void setMotor2Speed(int speed);
    void enableAltitudeControl(bool on_off);
    void enableAttitudeControl(bool on_off);
};

inline void Control::init() {
    Serial.println(F("[CONTROL] Initializing..."));

    servo1.attach(36);
    servo2.attach(37);

    servo1.write(initialPosition1);
    servo2.write(initialPosition2);

    motor1.attach(2, usMin, usMax);
    motor2.attach(3, usMin, usMax);

    Serial.println(F("[CONTROL] Initialization complete!"));
}

inline void Control::arm(bool on_off) {
    if (on_off) {
        Serial.println(F("[CONTROL] Arming motors..."));
        Control::armMotor(motor1, usMin, usMax);
        Control::armMotor(motor2, usMin, usMax);
        motor_armed = true;
        Serial.println(F("[CONTROL] Motors armed!"));
        return;
    }

    Serial.println(F("[CONTROL] Disarming motors..."));
    motor_armed = false;
    motor1.write(0);
    motor2.write(0);
    Serial.println(F("[CONTROL] Motors disarmed."));
}

inline void Control::armMotor(Servo &motor, int min, int max) {
    motor.writeMicroseconds(min); // Arm ESC
    delay(1000);
}

inline void Control::setMotor1Speed(int speed) {
    if (motor_armed) {
        motor1.write(speed);
        return;
    }

    motor1.write(0);
}

inline void Control::setMotor2Speed(int speed) {
    if (motor_armed) {
        motor2.write(speed);
        return;
    }

    motor2.write(0);
}

inline void Control::setServo1Angle(float angle) {
    int relativeAngle = constrain(initialPosition1 + constrain(angle, -30, 30), 0, 180);
    servo1.write(relativeAngle);
}

inline void Control::setServo2Angle(float angle) {
    int relativeAngle = constrain(initialPosition2 + constrain(angle, -30, 30), 0, 180);
    servo2.write(relativeAngle);
}

inline void Control::enableAltitudeControl(bool on_off) {
    if (on_off) {
        altitudeControl = true;
        altitudeTimestamp = micros();
        Serial.println(F("[CONTROL] Enabled altitude control."));
        return;
    }

    altitudeControl = false;
    Serial.println(F("[CONTROL] Disabled altitude control."));
}

inline void Control::enableAttitudeControl(bool on_off) {
    if (on_off) {
        attitudeControl = true;
        attitudeTimestamp = micros();
        Serial.println(F("[CONTROL] Enabled attitude control."));
        return;
    }

    attitudeControl = false;
    Serial.println(F("[CONTROL] Disabled attitude control."));
}

inline void Control::run(){
    if (altitudeControl){
        runAltitudeController();
    }

    if (attitudeControl){
        runAttitudeController();
    } else {
        setServo1Angle(0);
        setServo2Angle(0);
    }
}

inline void Control::runAltitudeController()
{
    unsigned long currentTime = micros();
    unsigned long deltaTimeMicros = currentTime - altitudeTimestamp;

    if (deltaTimeMicros < (1000000 / ALTITUDE_CONTROL_RATE_HZ))
    {
        return;
    }
    altitudeTimestamp = currentTime;
    float deltaTime = deltaTimeMicros / 1000000.0f; // Convert microseconds to seconds

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
    motorSpeed1 = 118 + desiredChange;
    motorSpeed2 = 118 + desiredChange;

    // Ensure motor speeds are within acceptable limits (e.g., 0 to 155)
    motorSpeed1 = constrain(motorSpeed1, 0, 155);
    motorSpeed2 = constrain(motorSpeed2, 0, 155);

    printf("Height: %lf, Error: %lf, Control Output: %lf, Motor Setpoints: %d, %d \n", nav->height, errorAltitude, controlAltitude, motorSpeed1, motorSpeed2);

    // Set motor speeds
    setMotor1Speed(motorSpeed1);
    setMotor2Speed(motorSpeed2);
}

inline void Control::runAttitudeController()
{
    unsigned long currentTime = micros();
    unsigned long deltaTimeMicros = currentTime - attitudeTimestamp;

    // Ensure the controller runs at the specified rate
    if (deltaTimeMicros < (1000000 / ATTITUDE_CONTROL_RATE_HZ))
    {
        return;
    }
    attitudeTimestamp = currentTime;
    float deltaTime = deltaTimeMicros / 1000000.0f; // Convert microseconds to seconds

    float roll = nav->roll;
    float pitch = nav->pitch;
    float yaw = nav->yaw;
    
    float errorPitch = -pitch;
    float errorRoll = -roll;

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

    // Limit control outputs to ±30 degrees
    controlPitch = constrain(controlPitch, -30, 30);
    controlRoll = constrain(controlRoll, -30, 30);

    // Set the servo angles
    setServo1Angle(controlPitch);
    setServo2Angle(controlRoll);

    // Update previous errors
    prevErrorPitch = errorPitch;
    prevErrorRoll = errorRoll;
}

#endif