/*
    control.h

    Manages vehicle control logic

    Authors: Orfeas Magoulas
*/

#ifndef CONTROL_H
#define CONTROL_H

#include <nav.h>
#include "actuators.h"
#include "guidance.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;

#define ALTITUDE_CONTROL_RATE_HZ 20

#define ATTITUDE_CONTROL_RATE_HZ 10
#define RCS_CONTROL_RATE_HZ 10


typedef Matrix<float, 4, 6 > Matrix4x6;
typedef Matrix<float, 6, 1> Vector6;
typedef Matrix<float, 4, 1> Vector4c;

class Control {
private:
    Nav *nav;
    Actuators *actuators;
    Guidance *guidance;

    bool debug = false;

    bool attitudeControl = false;
    bool altitudeControl = false;
    bool rcs = false;

    /*
        Attitude Control
    */
    float setpointRoll = 0.0;
    float setpointPitch = 0.0;
    float setpointYaw = 0.0;
    Matrix4x6 thingu;

    // Runtime variables (don't touch)
    unsigned long attitudeTimestamp = 0;

    /*
        RCS Control
    */
    float Kp_RCS = 0.001f;
    float Kd_RCS = 0.1f;
    float prevErrorRCS = 0.0f;
    unsigned long rcsTimestamp = 0;    

    /*
        Altitude Control
    */    
    // Initial motor speeds 
    int motorSpeed = 0;

    float Kp_altitude = 80.0f;
    float Ki_altitude = 40.0f;
    float Kd_altitude = 0.0f;
    float targetHeight = 0.5f;

    // Runtime variables (don't touch)
    float integralAltitude = 0.0f;
    float prevErrorAltitude = 0.0f;
    unsigned long altitudeTimestamp = 0;    

public:
    Control(Nav *nav, Actuators *actuators, Guidance *guidance): nav(nav), actuators(actuators), guidance(guidance) {}

    void init() {
        Serial.println(F("[CONTROL] Initializing..."));
        actuators->init();

        thingu << -1.7321,-0.0000,0.0000,-1.7425,0.0000,0.0000,0.0000,-1.7321,0.0000,-0.0000,-1.7425,0.0000,0,0,0,0,0,0,-0.0000,-0.0000,1.7321,0.0000,0.0000,1.7339; // K Matrix

        Serial.println(F("[CONTROL] Control initialization complete!"));
    }

    void run() {
        if (altitudeControl){
            this->runAltitudeController();
        }

        if (rcs) {
            this->runRCS();
        }

        if (attitudeControl){
            this->runAttitudeController();
        } else {
            actuators->setPitchServoAngle(0);
            actuators->setRollServoAngle(0);
        }
    }

    void enableAltitudeControl(bool on_off) {
        if (on_off) {
            altitudeControl = true;
            altitudeTimestamp = micros();
            Serial.println(F("[CONTROL] Enabled altitude control."));
            return;
        }

        altitudeControl = false;
        Serial.println(F("[CONTROL] Disabled altitude control."));
    }

    void enableAttitudeControl(bool on_off) {
        if (on_off) {
            attitudeControl = true;
            attitudeTimestamp = micros();
            Serial.println(F("[CONTROL] Enabled attitude control."));
            return;
        }

        attitudeControl = false;
        Serial.println(F("[CONTROL] Disabled attitude control."));
    }

    void enableRCS(bool on_off) {
        if (on_off) {
            rcs = true;
            rcsTimestamp = micros();
            Serial.println(F("[CONTROL] Enabled RCS control."));
            return;
        }

        rcs = false;
        Serial.println(F("[CONTROL] Disabled RCS control."));
    }

    void runAltitudeController()
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
        integralAltitude = constrain(integralAltitude, 0.0f, 100.0f);

        // Compute derivative term
        float derivativeAltitude = (errorAltitude - prevErrorAltitude) / deltaTime;

        // Compute control output
        float controlAltitude = Kp_altitude * errorAltitude + Ki_altitude * integralAltitude + Kd_altitude * derivativeAltitude;

        // Update previous error
        prevErrorAltitude = errorAltitude;

        // Adjust motor speeds with rate limiting
        int desiredChange = static_cast<int>(controlAltitude);

        // Adjust motor speeds
        motorSpeed = 40 + desiredChange;

        // Ensure motor speeds are within acceptable limits (e.g., 0 to 155)
        motorSpeed = constrain(motorSpeed, 0, 255);

        printf("Height: %.2f, Error: %.2f, Control Output: %.2f, Motor Setpoints: %d \n", nav->height, errorAltitude, controlAltitude, motorSpeed);

        // Set motor speeds via Actuators
        actuators->setThrust(motorSpeed / 255.0);
    }

    void runRCS() {
        unsigned long currentTime = micros();
        unsigned long deltaTimeMicros = currentTime - rcsTimestamp;

        if (deltaTimeMicros < (1000000 / RCS_CONTROL_RATE_HZ))
        {
            return;
        }
        rcsTimestamp = currentTime;
        float deltaTime = deltaTimeMicros / 1000000.0f; // Convert microseconds to seconds

        float r = nav->yaw;
        float errorRCS = 0-r;

        float derivativeRCS = (errorRCS - prevErrorRCS) / deltaTime;
        float controlRCS = Kp_RCS * errorRCS + Kd_RCS * derivativeRCS;

        printf("r: %.2f, control: %.2f\n", r, errorRCS * Kp_RCS);
        actuators->setTorque(errorRCS * Kp_RCS);
    }

    void runAttitudeController()
    {
        unsigned long currentTime = micros();
        unsigned long deltaTimeMicros = currentTime - attitudeTimestamp;

        if (deltaTimeMicros < (1000000 / ATTITUDE_CONTROL_RATE_HZ))
        {
            return;
        }
        attitudeTimestamp = currentTime;
        
        float deltaTime = deltaTimeMicros / 1000000.0f;

        float roll = nav->roll; // phi
        float pitch = nav->pitch; // theta 
        float yaw  = nav->yaw; // psi
        float p = nav->p;
        float q = nav->q;
        float r = nav->r;

        Vector6 state;
        Vector4c output;    
        state << roll, pitch, yaw, p, q, r;
        output = -this->thingu * state;
        
        if (debug) {
            Serial.print("Roll: "); Serial.print(roll);
            Serial.print(" Pitch: "); Serial.print(pitch);
            Serial.print(" Yaw: "); Serial.println(yaw);
            Serial.print("p: "); Serial.print(p);
            Serial.print(" q: "); Serial.print(q);
            Serial.print(" r: "); Serial.println(r);
        }
        const float gain = 0.2f;

        actuators->setPitchServoAngle(constrain(output[1] * gain , -30.0f, 30.0f));
        actuators->setRollServoAngle(constrain(output[0] * gain, -30.0f, 30.0f));
        // actuators->setThrust(0.0);
        // actuators->setTorqueRCS(0.0);
    }
};
#endif // CONTROL_H